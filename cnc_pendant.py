#!/usr/bin/env python3
"""CNC Pendant Controller for Raspberry Pi 4B
Implements a manual CNC pendant with a serial USB interface, manual pulse generator (MPG),
spindle control encoder, axis/unit selection switches, and an I2C 20x4 DRO display."""


import sys
import time
import threading
import queue
import os



# Try importing RPi hardware libraries
try:
    import pigpio
except ImportError:
    sys.exit("Fatal: 'pigpio' library not found.")

try:
    import serial
except ImportError:
    sys.exit("Fatal: python 'serial' library not found.")

try:
    from smbus2 import SMBus
except ImportError:
    sys.exit("Fatal: python library 'SMBus' not found.")



def is_systemd_service():
    return "INVOCATION_ID" in os.environ


# --- PIN DEFINITIONS (BCM GPIO NUMBERING) ---
PIN_MPG_A = 17
PIN_MPG_B = 18
PIN_SPINDLE_DUTYA = 22
PIN_SPINDLE_DUTYB = 23
PIN_AXIS_X = 5
PIN_AXIS_Y = 6
PIN_AXIS_Z = 13
PIN_BTN_TURBO = 19
PIN_BTN_ZERO = 26
PIN_SPINDLE_ON = 20
PIN_UNITS_SW = 21
PIN_BTN_RESET = 24
PIN_RESET_LIGHT = 25
PIN_BTN_POWER_LOW = 14
PIN_BTN_POWER_HIGH = 15
PIN_POWER_FEED_LOW_LIGHT = 7
PIN_POWER_FEED_HIGH_LIGHT = 8
PIN_BTN_POWER_FEED_BOUNCE = 9
PIN_FEED_RATE_A = 11
PIN_FEED_RATE_B = 12

# --- SYSTEM CONFIGURATION ---
SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_BAUDRATE = 9600
LCD_I2C_ADDRESS = 0x27
I2C_BUS_NUMBER = 1

# I2C pins stated here for clarity, but these are assumed
# when I2C_BUS_NUMBER = 1.  These two variables are not
# used in this code.
PIN_I2C_SDA = 2 # physical pin 3
PIN_I2C_SCL = 3 # physical pin 5


# --- STATUS BITS ---
#
# Note that the Y-axis limit switches are physically wired backwards
# in my machine.  You go to the max positive Y displacement,
# and it tells you that the "-Y Limit" switch was hit.  Thus
# my constants below for Y are actually reversed from what the
# machine reports.  When you hit the positive Y stop, you can
# only move in the negative direction.  That corresponds to
# value 239 in the status register, so I call it Y_POSITIVE_LIMIT,
# even though the machine says it's the negative limit.
# The other two axes behave normally: going to max X or Z triggers
# the appropriate positive limit switch.  These limit
# switch values are found in the third byte value of the 0x04
# polling request.

X_POSITIVE_LIMIT = 127
X_NEGATIVE_LIMIT = 191
Y_POSITIVE_LIMIT = 239
Y_NEGATIVE_LIMIT = 223
Z_POSITIVE_LIMIT = 247
Z_NEGATIVE_LIMIT = 251
DIRECTION_POSITIVE = 1
DIRECTION_NEGATIVE = -1

E_STOP = 64  # of the 1st byte value

# The spindle being on or off can be detected in the 2nd byte of
# the 0x04 query, but this is not so useful.  It's 1 when the
# spindle is on, 0 when it's off.  We won't bother implementing
# this.


# --- MACHINE CONSTANTS ---
X_TRAVEL = 12.0
Y_TRAVEL = 5.5
Z_TRAVEL = 5.0


# --- MATH CONSTANTS
NANOSECONDS_PER_MINUTE = 60_000_000_000


# --- THREAD-SAFE STATE MANAGEMENT ---

class PendantState:
    def __init__(self):
        self.lock = threading.Lock()

        self.x = 0
        self.y = 0
        self.z = 0
        
        self.current_axis = 'X'        # Options: 'X', 'Y', 'Z'
        self.turbo_active = False      # True if button is depressed
        self.spindle_on = False        # True if switch is toggled ON
        self.spindle_duty = 0          # Tracked in 5% increments (0 to 100)
        self.units = 'mm'              # Options: 'mm', 'inch'

        self.in_reset = False		# are we attempting to reset a e-stop state?
        self.work_offset_x = 0.000
        self.work_offset_y = 0.000
        self.work_offset_z = 0.000

        # The user can specify two X-axis positions for power
        # feeding between
        self.power_feed_low_limit = 0.000
        self.power_feed_high_limit = 0.000

        # Note when we hit a limit switch.  Because this controller only
        # allows movement in a single selected axis at a time, we only need
        # once at_axis_limit variable.
        self.at_axis_limit = ""  	# 'X', 'Y', or 'Z'
        self.direction_limit = 0	# DIRECTION_POSITIVE or DIRECTION_NEGATIVE, with value == 0 being not set
        
        # We will set soft limits if we happen to hit a limit switch.
        self.soft_limit_positive_x = 0.000
        self.soft_limit_negative_x = 0.000
        self.soft_limit_positive_y = 0.000
        self.soft_limit_negative_y = 0.000
        self.soft_limit_positive_z = 0.000
        self.soft_limit_negative_z = 0.000

        self.estop = False

        self.power_feed_mode = False
        self.power_feed_rate = 15  # initial feed rate for power feed; good for aluminum.  Steel would be ~5, wood  20-24

        
    def update_axis(self, axis):
        with self.lock:
            if self.current_axis != axis:
                self.current_axis = axis
                return True
        return False

    def update_turbo(self, active):
        with self.lock:
            if self.turbo_active != active:
                self.turbo_active = active
                return True
        return False

    def update_spindle_switch(self, on):
        with self.lock:
            if self.spindle_on != on:
                self.spindle_on = on
                return True
        return False

    def update_units(self, units):
        with self.lock:
            if self.units != units:
                self.units = units
                return True
        return False

    def change_spindle_duty(self, delta):
        with self.lock:
            if delta != 0:
                self.spindle_duty += delta
                if self.spindle_duty < 0:
                    self.spindle_duty = 0;

                if self.spindle_duty > 100:
                    self.spindle_duty = 100
                    
                return True
        return False

    def change_feed_rate(self, delta):
        with self.lock:
            if delta != 0:
                self.power_feed_rate += delta
                if self.power_feed_rate < 1:
                    self.power_feed_rate = 1

                if self.power_feed_rate > 30:
                    self.power_feed_rate = 30

                return True
            
        return False
        
    def zero_axis(self, x, y, z):
        with self.lock:
            # The absolute machine coordinates are passed to us,
            # so we should update.  We have to write the variables
            # ourselves, as we would deadlock if we called set_position()
            self.x = x
            self.y = y
            self.z = z

            # Set relative work offsets, used for DRO display
            # purposes only
            if self.current_axis == 'X':
                self.work_offset_x = x
            elif self.current_axis == 'Y':
                self.work_offset_y = y
            elif self.current_axis == 'Z':
                self.work_offset_z = z
            return self.current_axis

        
    def hit_limit(self, position, direction):
        """Sets soft limits, whenever a physical limit switch is tripped"""
        with self.lock:

            # We can set both stops for each axis, because we know
            # the total table travel in each dimension.  Once we
            # know one, we can calculate the other.
            
            if self.current_axis == 'X':
                if direction == DIRECTION_POSITIVE:
                    self.soft_limit_positive_x = position
                    self.soft_limit_negative_x = position - X_TRAVEL
                elif direction == DIRECTION_NEGATIVE:
                    self.soft_limit_negative_x = position
                    self.soft_limit_positive_x = position + X_TRAVEL
            elif self.current_axis == 'Y':
                if direction == DIRECTION_POSITIVE:
                    self.soft_limit_positive_y = position
                    self.soft_limit_negative_y = position - Y_TRAVEL
                elif direction == DIRECTION_NEGATIVE:
                    self.soft_limit_negative_y = position
                    self.soft_limit_positive_y = position + Y_TRAVEL
            elif self.current_axis == 'Z':
                if direction == DIRECTION_POSITIVE:
                    self.soft_limit_positive_z = position
                    self.soft_limit_negative_z = position - Z_TRAVEL
                elif direction == DIRECTION_NEGATIVE:
                    self.soft_limit_negative_z = position
                    self.soft_limit_positive_z = position + Z_TRAVEL

            if self.current_axis == 'X':
                print(f"Setting {self.current_axis} limits: {self.soft_limit_positive_x} {self.soft_limit_negative_x}")
            elif self.current_axis == 'Y':
                print(f"Setting {self.current_axis} limits: {self.soft_limit_positive_y} {self.soft_limit_negative_y}")
            elif self.current_axis == 'Z':
                print(f"Setting {self.current_axis} limits: {self.soft_limit_positive_z} {self.soft_limit_negative_z}")
                
            self.at_axis_limit = self.current_axis
            self.direction_limit = direction


    def is_limit_set(self, axis):
        if self.at_axis_limit == axis:
            return True

        return False
    
    def movement_allowed(self, proposed_movement, direction):
        """Check if proposed motion command would exceed soft limits"""

        # First, if e-stop is set, no motion allowed
        if self.estop == True and self.in_reset == False:
            return False
        
        # Next, check to make sure we haven't hit a limit switch.
        # We can't go any further in that direction, if we have hit one.
        if self.at_axis_limit == self.current_axis:
            if direction == self.direction_limit:
                return False
                
        # Then check to make sure we haven't exceeded any soft limits.
        # Note that the initialized value of 0.000 for all of these
        # limits will be treated as false by Python in the if
        # conditions below.  In other words, we are hoping that the
        # soft limits never happen to occur at 0.000 in the machine's
        # absolute coordinates.  Also as a consequence, we can't set the
        # soft limits until and unless the hard limit switches are
        # tripped.
        allowed = True
        reason = "Unknown reason for rejecting motion"
        if self.current_axis == 'X':
            if direction == DIRECTION_POSITIVE:
                if self.soft_limit_positive_x and self.x + proposed_movement > self.soft_limit_positive_x:
                    allowed = False
                    reason = f"X movement {proposed_movement} exceeds {self.soft_limit_positive_x}"
            elif direction == DIRECTION_NEGATIVE:
                if self.soft_limit_negative_x and self.x - proposed_movement < self.soft_limit_negative_x:
                    allowed = False
                    reason = f"X movement {proposed_movement} goes below {self.soft_limit_negative_x}"
        elif self.current_axis == 'Y':
            if direction == DIRECTION_POSITIVE:
                if self.soft_limit_positive_y and self.y + proposed_movement > self.soft_limit_positive_y:
                    allowed = False
                    reason = f"Y movement {proposed_movement} exceeds {self.soft_limit_positive_y}"
            elif direction == DIRECTION_NEGATIVE:
                if self.soft_limit_negative_y and self.y - proposed_movement < self.soft_limit_negative_y:
                    allowed = False
                    reason = f"Y movement {proposed_movement} goes below {self.soft_limit_negative_y}"
        elif self.current_axis == 'Z':
            if direction == DIRECTION_POSITIVE:
                if self.soft_limit_positive_z and self.z + proposed_movement > self.soft_limit_positive_z:
                    allowed = False
                    reason = f"Z movement {proposed_movement} exceeds {self.soft_limit_positive_z}"
            elif direction == DIRECTION_NEGATIVE:
                if self.soft_limit_negative_z and self.z - proposed_movement < self.soft_limit_negative_z:
                    allowed = False
                    reason = f"Z movement {proposed_movement} goes below {self.soft_limit_negative_z}"

        if allowed:
            # We've passed all the tests, so we must allow the movement
            # This also means we must be out of reset.
            self.in_reset = False
            self.at_axis_limit = ""
            return True
        else:
            print(f"Movement rejected: {reason}")
            return False

    
    def set_position(self, x, y, z):
        with self.lock:
            self.x = x
            self.y = y
            self.z = z


    def set_estop(self, bool):
        """Set the stopped/reset needed flag"""
        with self.lock:
            self.estop = bool
            

    def set_power_feed_low_limit(self, pos):
        """Sets the low-side X-axis power feed limit"""
        with self.lock:
            self.power_feed_low_limit = pos

    def set_power_feed_high_limit(self, pos):
        """Sets the high-side X-axis power feed limit"""
        with self.lock:
            self.power_feed_high_limit = pos
            

    def delta_x(self, pos1, pos2):
        # less than a tenth of an inch of separation returns True
        if abs(pos1 - pos2) < 0.1:
            return True

        return False
        
# Initialize global state instance
state = PendantState()


# --- HARDWARE DECODER FOR ROTARY ENCODERS ---

class QuadratureDecoder:
    """
    Decodes standard 2-bit gray code quadrature signals from an encoder or MPG wheel
    using high-accuracy hardware-timed edge interrupts via the pigpio daemon.
    """
    def __init__(self, pi, pin_a, pin_b):
        """
        Initializes the encoder tracking configurations.
        
        Parameters:
            pi (pigpio.pi): An initialized and connected pigpio daemon client instance.
            pin_a (int): The Broadcom (BCM) GPIO pin number allocated to Encoder Channel A.
            pin_b (int): The Broadcom (BCM) GPIO pin number allocated to Encoder Channel B.
        """
        self.pi = pi
        self.pin_a = pin_a
        self.pin_b = pin_b
        
        # Thread-safe primitives to shield count mutations from simultaneous lookups
        self.lock = threading.Lock()
        self._count = 0
        
        # Configure underlying hardware pins to default input states with weak pull-up resistors engaged
        self.pi.set_mode(self.pin_a, pigpio.INPUT)
        self.pi.set_mode(self.pin_b, pigpio.INPUT)
        self.pi.set_pull_up_down(self.pin_a, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.pin_b, pigpio.PUD_UP)
        
        # Take an initial baseline snapshot reading of both lines
        self.last_a = self.pi.read(self.pin_a)
        self.last_b = self.pi.read(self.pin_b)
        
        # Register hardware interrupt service edge callbacks for both lines
        self.cb_a = self.pi.callback(self.pin_a, pigpio.EITHER_EDGE, self._encoder_callback)
        self.cb_b = self.pi.callback(self.pin_b, pigpio.EITHER_EDGE, self._encoder_callback)

    def _encoder_callback(self, gpio, level, tick):
        """
        Internal interrupt service routine callback fired automatically by pigpiod 
        whenever an electrical state transition (rising or falling edge) occurs.
        """
        # Read the absolute realtime state profile of both input lines
        current_a = self.pi.read(self.pin_a)
        current_b = self.pi.read(self.pin_b)
        
        with self.lock:
            if gpio == self.pin_a:
                # State checking for Channel A transitions
                if level != self.last_a:
                    self.last_a = level
                    # If A changes, check against the static position of B to isolate direction
                    if level == 1 and current_b == 0:
                        self._count += 1
                    elif level == 0 and current_b == 1:
                        self._count += 1
                    elif level == 1 and current_b == 1:
                        self._count -= 1
                    elif level == 0 and current_b == 0:
                        self._count -= 1
                        
            elif gpio == self.pin_b:
                # State checking for Channel B transitions
                if level != self.last_b:
                    self.last_b = level
                    # If B changes, check against the static position of A to isolate direction
                    if level == 1 and current_a == 1:
                        self._count += 1
                    elif level == 0 and current_a == 0:
                        self._count += 1
                    elif level == 1 and current_a == 0:
                        self._count -= 1
                    elif level == 0 and current_a == 1:
                        self._count -= 1

    def get_count(self):
        """
        Thread-safely extracts and returns the current aggregated directional step accumulation count.
        
        Returns:
            int: Total accumulation tally offset from startup baseline zero.
        """
        with self.lock:
            return self._count

    def reset_count(self):
        """Resets the internal tracking accumulator step register back to 0."""
        with self.lock:
            self._count = 0

    def shutdown(self):
        """
        Gracefully releases and tears down the registered background daemon edge callbacks 
        to prevent memory or reference leaks inside pigpiod.
        """
        if hasattr(self, 'cb_a') and self.cb_a:
            self.cb_a.cancel()
        if hasattr(self, 'cb_b') and self.cb_b:
            self.cb_b.cancel()



# --- SERIAL COMMUNICATION INTERFACE ---
class SerialInterface:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.connect()

    def connect(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=2.0
            )
            
            # Ensure DTR line is explicitly asserted HIGH to engage mill controller communication
            self.ser.dtr = True
            print(f"Successfully connected to machine serial port: {self.port} with DTR=HIGH")
            return True

        except Exception as e:
            print(f"Serial Connection Error on {self.port}: {e}. Running with mock logging.")
            self.ser = None

        return False

    def send_command(self, cmd_string):
        formatted_cmd = f"{cmd_string}\r\n"
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(formatted_cmd.encode('ascii'))
                self.ser.flush()
                self.ser.readline()   # discard the echo
            except Exception as e:
                print(f"Serial write failure: {e}")

                
    def send_motor_reset(self):
        """Needed on power on or when the E-stop or a limit switch is hit"""
        self.send_command("ALLDRVON\r\n")
        self.ser.reset_input_buffer()
        raw_line = self.ser.readline()
        response_str = raw_line.decode('ascii').strip()
        if not response_str:
            return None

        if "All" in response_str:
            return response_str

        return None

    def poll_servos(self):
        """Send the 0x02 polling command, and format the return values"""
        coordinates = []
        axis_labels = ['X', 'Y', 'Z']
        
        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()
            self.ser.write(bytes([0x02]))
            self.ser.flush()
            self.ser.readline()  # discard the echoed 0x02
            got_coords = 0
            while got_coords < 3:
                # readline() reads up to the \n character, honoring the \r\n boundary
                raw_line = self.ser.readline()
                    
                # Check if the read timed out (returns empty bytes if timeout occurs)
                if not raw_line:
                    continue
                
                # Decode raw bytes to ASCII and strip out structural whitespace/newlines (\r\n)
                clean_str = raw_line.decode('ascii').strip()
                    
                # Parse the string text into a floating-point number
                try:
                    pos_value = float(clean_str)
                    coordinates.append(pos_value)
                    got_coords += 1
                except Exception as err:
                    continue
                
            # A 4th value comes when polling.  Don't know what it is,
            # but we'll discard it
            self.ser.readline()
                
            # Return a structured tuple containing all 3 positions
            return tuple(coordinates)
                
        else:
            coordinates.append(0)
            coordinates.append(0)
            coordinates.append(0)
            return tuple(coordinates)

    def poll_status(self):
        """Send the 0x04 status request command, and format the return values"""
        status_regs = []
        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()
            self.ser.write(bytes([0x04]))
            self.ser.flush()
            self.ser.readline()  # discard the echo
            got_vals = 0
            while got_vals < 3:
                raw_line = self.ser.readline()

                if not raw_line:
                    continue

                clean_str = raw_line.decode('ascii').strip()
                
                try:
                    status_byte = int(clean_str)
                    status_regs.append(status_byte)
                    got_vals += 1
                except Exception as err:
                    continue

            return tuple(status_regs)
    
    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

            
# Instantiate global serial hardware controller
serial_comm = SerialInterface(SERIAL_PORT, SERIAL_BAUDRATE)

# --- SERIAL EMITTERS ---
def emit_jog(axis, rel_move, feed_rate):
    """Generates ASCII commands corresponding to mechanical MPG adjustments."""
    # Example high level command: G1X[X+2.3] F25
    command = f"G1{axis}[{axis}{rel_move:+}]F{feed_rate}."
    serial_comm.send_command(command)

def toggle_spindle(is_on):
    if is_on:
        command = "1ON"
        serial_comm.send_command(command)
        time.sleep(1)
    else:
        command = "DUTYA0.0"
        serial_comm.send_command(command)
        time.sleep(4)  # wait for spin down
        command = "1OFF"
        serial_comm.send_command(command)
        time.sleep(1)
        
    
def emit_spindle(is_on, duty_percent):
    """Generates ASCII commands setting operational state and duty configurations."""
    if is_on and duty_percent != 0:
        command = f"DUTYA{duty_percent}.0"
        serial_comm.send_command(command)
        time.sleep(1)
        

def emit_power_feed(direction):
    if direction == DIRECTION_POSITIVE:
        command = f"G1 X{state.power_feed_high_limit} F{state.power_feed_rate}"
        serial_comm.send_command(command)
    elif direction == DIRECTION_NEGATIVE:
        command = f"G1 X{state.power_feed_low_limit} F{state.power_feed_rate}"
        serial_comm.send_command(command)
    
  

# --- HD44780 I2C 2004 LCD DRIVER CLASS ---
#
# We don't need to state the specific GPIO pins in this code,
# because Linux assigns GPIOs 2 and 3 automatically.
# These two pins become SMBus 1, and you do see that in
# the initializer call with I2C_BUS_NUMBER.
#
# The "backpack" on the LCD panel (usually a PCF8574 controller chip)
# takes an address on the I2C bus.  By default, this is 0x27, which
# is given in LCD_I2C_ADDRESS.

    
class LiquidCrystalI2C:
    # Commands
    LCD_CLEARDISPLAY = 0x01
    LCD_RETURNHOME = 0x02
    LCD_ENTRYMODESET = 0x04
    LCD_DISPLAYCONTROL = 0x08
    LCD_CURSORSHIFT = 0x10
    LCD_FUNCTIONSET = 0x20
    LCD_SETCGRAMADDR = 0x40
    LCD_SETDDRAMADDR = 0x80

    # Flags for display entry mode
    LCD_ENTRYLEFT = 0x02
    LCD_ENTRYSHIFTDECREMENT = 0x00

    # Flags for display on/off control
    LCD_DISPLAYON = 0x04
    LCD_CURSOROFF = 0x00
    LCD_BLINKOFF = 0x00

    # Flags for function set (forcing 4-bit mode)
    LCD_4BITMODE = 0x00
    LCD_2LINE = 0x08
    LCD_5x8DOTS = 0x00

    # Flags for backlight control
    LCD_BACKLIGHT = 0x08
    LCD_NOBACKLIGHT = 0x00

    # Control bits for the PCF8574 backpack
    En = 0b00000100  # Enable bit
    Rw = 0b00000010  # Read/Write bit
    Rs = 0b00000001  # Register Select bit

    # Row offsets for a standard 20x4 display matrix
    ROW_OFFSETS = [0x00, 0x40, 0x14, 0x54]

    def __init__(self, i2c_addr=0x27, bus_num=1):
        """
        Initializes the LCD. 
        On Raspberry Pi 4B, GPIO 2 (SDA) and GPIO 3 (SCL) map to bus_num=1.
        Common I2C addresses for PCF8574 backpacks are 0x27 or 0x3F.
        """
        self.address = i2c_addr
        self.bus = SMBus(bus_num)
        self.backlight_state = self.LCD_BACKLIGHT
        
        # HD44780 4-bit initialization sequence
        time.sleep(0.05)
        self._write_four_bits(0x03 << 4)
        time.sleep(0.005)
        self._write_four_bits(0x03 << 4)
        time.sleep(0.001)
        self._write_four_bits(0x03 << 4)
        self._write_four_bits(0x02 << 4)  # Set to 4-bit interface

        # Function configuration
        self.command(self.LCD_FUNCTIONSET | self.LCD_4BITMODE | self.LCD_2LINE | self.LCD_5x8DOTS)
        self.command(self.LCD_DISPLAYCONTROL | self.LCD_DISPLAYON | self.LCD_CURSOROFF | self.LCD_BLINKOFF)
        self.command(self.LCD_ENTRYMODESET | self.LCD_ENTRYLEFT | self.LCD_ENTRYSHIFTDECREMENT)
        self.clear()

    def _write_four_bits(self, data):
        self.bus.write_byte(self.address, data | self.backlight_state)
        self._strobe(data)

    def _strobe(self, data):
        # Pulse Enable high then low to latch data
        self.bus.write_byte(self.address, data | self.En | self.backlight_state)
        time.sleep(0.0005)
        self.bus.write_byte(self.address, (data & ~self.En) | self.backlight_state)
        time.sleep(0.0001)

    def _send(self, value, mode):
        high_nibble = value & 0xF0
        low_nibble = (value << 4) & 0xF0
        self._write_four_bits(high_nibble | mode)
        self._write_four_bits(low_nibble | mode)

    def command(self, value):
        self._send(value, 0)

    def write_char(self, value):
        self._send(value, self.Rs)

    def clear(self):
        self.command(self.LCD_CLEARDISPLAY)
        time.sleep(0.002)

    def set_cursor(self, col, row):
        if row < 0 or row >= len(self.ROW_OFFSETS):
            row = 0
        self.command(self.LCD_SETDDRAMADDR | (col + self.ROW_OFFSETS[row]))

    def write_string(self, string, row):
        """Writes a string to a specific row, padded to 20 characters."""
        self.set_cursor(0, row)
        # Pad or truncate string to ensure exactly 20 characters overwrite existing text
        padded_string = string[:20].ljust(20)
        for char in padded_string:
            self.write_char(ord(char))

    def display_lines(self, line1="", line2="", line3="", line4=""):
        """
        Updates all 4 lines of the display.
        Takes 4 strings as arguments.
        """
        lines = [line1, line2, line3, line4]
        for idx, text in enumerate(lines):
            self.write_string(text, idx)


# Instantiate global user viewing interface component
lcd_display = LiquidCrystalI2C(LCD_I2C_ADDRESS, I2C_BUS_NUMBER)



# --- SYSTEM SETUP AND POLLING TASK LOOP ---
def main():
    pi = None

    if pigpio:
        pi = pigpio.pi()
        if not pi.connected:
            print("CRITICAL: Failed to link with 'pigpio' background service daemon. Ensure 'sudo pigpiod' is running.")
            sys.exit(1)
            
        # Configure standard non-encoder discrete switch lines with safe pull-ups
        for pin in [PIN_AXIS_X, PIN_AXIS_Y, PIN_AXIS_Z, PIN_BTN_TURBO, PIN_BTN_ZERO, PIN_SPINDLE_ON, PIN_UNITS_SW, PIN_BTN_RESET, PIN_BTN_POWER_LOW, PIN_BTN_POWER_HIGH, PIN_BTN_POWER_FEED_BOUNCE]:
            pi.set_mode(pin, pigpio.INPUT)
            pi.set_pull_up_down(pin, pigpio.PUD_UP)

        # the reset and x power feed limit lights use positive logic; pull them down
        pi.set_mode(PIN_RESET_LIGHT, pigpio.OUTPUT)
        pi.set_pull_up_down(PIN_RESET_LIGHT, pigpio.PUD_DOWN)
        pi.set_mode(PIN_POWER_FEED_LOW_LIGHT, pigpio.OUTPUT)
        pi.set_pull_up_down(PIN_POWER_FEED_LOW_LIGHT, pigpio.PUD_DOWN)
        pi.set_mode(PIN_POWER_FEED_HIGH_LIGHT, pigpio.OUTPUT)
        pi.set_pull_up_down(PIN_POWER_FEED_HIGH_LIGHT, pigpio.PUD_DOWN)

        pi.write(PIN_POWER_FEED_LOW_LIGHT, 0)
        pi.write(PIN_POWER_FEED_HIGH_LIGHT, 0)
        
    # Initialize input rotational hardware interfaces
    mpg_encoder = QuadratureDecoder(pi, PIN_MPG_A, PIN_MPG_B)
    spindle_encoder = QuadratureDecoder(pi, PIN_SPINDLE_DUTYB, PIN_SPINDLE_DUTYA)  # backward because wired backwards
    power_feed_rate_encoder = QuadratureDecoder(pi, PIN_FEED_RATE_A, PIN_FEED_RATE_B)
    
    mpg_count = 0
    last_mpg_count = 0
    old_x = 0
    spindle_duty_count = 0
    last_spindle_duty_count = 0
    last_spindle_state = False
    last_power_feed_rate_count = 0
    last_poll_time = time.time_ns()

    initialized = False
    connection_attempts = 0;
    # Try every 10 seconds until we get a connection to the mill
    while True:
        connected = False
        connection_attempts += 1
        conn_str = f"Connecting... {connection_attempts}"
        lcd_display.display_lines("CNC Manual Pendant", conn_str, "", "")

        if not connected:
            connected = serial_comm.connect()
            
        if connected:
            lcd_display.display_lines("CNC Manual Pendant", conn_str, "", "")
            time.sleep(1)
            
            initialized = serial_comm.send_motor_reset()
            if initialized:
                conn_str = "Connected via RS-232"
                lcd_display.display_lines("CNC Manual Pendant", conn_str, initialized, "")
                time.sleep(2)
                break

        time.sleep(10)

        
    try:
        while True:
            this_poll_time = time.time_ns();

            # 1. Read Discrete Input Configurations from Hardware Matrix
            # Active LOW logic verification for switches tied directly to Ground
            sw_x = pi.read(PIN_AXIS_X) == 0
            sw_y = pi.read(PIN_AXIS_Y) == 0
            sw_z = pi.read(PIN_AXIS_Z) == 0
                
            btn_turbo = pi.read(PIN_BTN_TURBO) == 0
            btn_zero = pi.read(PIN_BTN_ZERO) == 0
                
            sw_spindle = pi.read(PIN_SPINDLE_ON) == 0
            sw_units = pi.read(PIN_UNITS_SW) == 0

            sw_reset = pi.read(PIN_BTN_RESET) == 0
        
            sw_pwr_feed_low = pi.read(PIN_BTN_POWER_LOW) == 0
            sw_pwr_feed_high = pi.read(PIN_BTN_POWER_HIGH) == 0
    

        
            # 2. Re-evaluate Active Axis State Properties
            if sw_x: state.update_axis('X')
            elif sw_y: state.update_axis('Y')
            elif sw_z: state.update_axis('Z')
            

            # 3. Re-evaluate Functional Modification Controls
            state.update_turbo(btn_turbo)
            state.update_units('inch' if sw_units else 'mm')

            # is user attempting to clear e-stop/limit?
            if sw_reset:
                # wait until user lets go
                while pi and pi.read(PIN_BTN_RESET) == 0:
                    time.sleep(0.05)
                
                state.in_reset = True    # signal that we're attempting to clear

                reset_result = serial_comm.send_motor_reset()
                if "All" in reset_result:
                    state.set_estop(False)
                    state.power_feed_mode = False   # hitting reset also clears power feed mode
                    state.power_feed_low_limit = 0.000
                    state.power_feed_high_limit = 0.000
                    pi.write(PIN_POWER_FEED_LOW_LIGHT, 0)
                    pi.write(PIN_POWER_FEED_HIGH_LIGHT, 0)

            if state.estop == True:
                pi.write(PIN_RESET_LIGHT, 1)
            else:
                pi.write(PIN_RESET_LIGHT, 0)

                    
            # 4. Spindle Operation Toggle Check
            if sw_spindle != state.spindle_on:
                state.update_spindle_switch(sw_spindle)
                toggle_spindle(sw_spindle)   # handle state change with 1ON/1OFF to make motor ready
                if sw_spindle and state.spindle_duty:
                    emit_spindle(sw_spindle, state.spindle_duty)  # and send a duty cycle, if on
                
                
            # 5. Process Zeroing Requests
            if btn_zero:
                # re-poll to get exact position
                x_pos, y_pos, z_pos = serial_comm.poll_servos()
                
                zeroed_axis = state.zero_axis(x_pos, y_pos, z_pos)

                # Implement debounce delay constraint until button clearance to minimize command flooding
                while pi and pi.read(PIN_BTN_ZERO) == 0:
                    time.sleep(0.05)

        
            # 6a. Send any accumulated MPG steps
            jog_needed = False
            spindle_change_needed = False
            
            mpg_count = mpg_encoder.get_count()
            current_mpg = mpg_count - last_mpg_count
                    
            if current_mpg:
                step_size = 0.001
                if state.turbo_active:
                    step_size = 0.01
                        
                rel_move = current_mpg * step_size  # can be positive or negative

                # X and Z axes moves in the wrong direction, subjectively, so invert them
                if state.current_axis == 'Z' or state.current_axis == 'X':
                    rel_move *= -1
                    
                interval = this_poll_time - last_poll_time;  # result in nanoseconds

                movement_direction = 0
                if rel_move > 0:
                    movement_direction = DIRECTION_POSITIVE
                elif rel_move < 0:
                    movement_direction = DIRECTION_NEGATIVE

                # check to make sure the move won't exceed soft limits
                # or continue in same direction that a limit switch
                # was encountered
                    
                if state.movement_allowed(abs(rel_move), movement_direction):
                    # feed_rate is {state.units} per minute
                    feed_rate = int(abs(rel_move / interval) * NANOSECONDS_PER_MINUTE)
                    
                    if feed_rate == 0:
                        feed_rate = 1
                            
                    if feed_rate > 300:
                        feed_rate = 300

                    jog_needed = True
                else:
                    jog_needed = False
            
            # 6b. Send any accumulated spindle rotary steps
            spindle_duty_count = spindle_encoder.get_count() >> 1  # the encoder double counts clicks, so divide by 2
            current_spindle = spindle_duty_count - last_spindle_duty_count
            spindle_delta = current_spindle * 5
            
            if current_spindle != 0:
                state.change_spindle_duty(spindle_delta)
                spindle_change_needed = True
                    
            last_spindle_duty_count = spindle_duty_count


            # 6c. Track any power feed rate rotary steps
            power_feed_rate_count = power_feed_rate_encoder.get_count() >> 1
            current_power_feed_rate_count = power_feed_rate_count - last_power_feed_rate_count
            
            if current_power_feed_rate_count != 0:
                state.change_feed_rate(current_power_feed_rate_count)

            last_power_feed_rate_count = power_feed_rate_count
            

            # Emit the rotary encoder changes
            if jog_needed:
                emit_jog(state.current_axis, rel_move, feed_rate)

            if spindle_change_needed and state.spindle_on:
                emit_spindle(state.spindle_on, state.spindle_duty)


            # 7. Periodic position synchronization
            x_pos, y_pos, z_pos = serial_comm.poll_servos()
            state.set_position(x_pos, y_pos, z_pos)

            # note whether X-axis is moving, for power feed calculations
            if x_pos != old_x:
                state.x_axis_moving = False
            else:
                state.x_axis_moving = True


            # 8. Poll status registers.  Don't put us back in reset if we're
            # attempting to get out of reset!
            if not state.in_reset:
                estop, unused2, limit = serial_comm.poll_status()
                if estop == 64:
                    state.set_estop(True)
                
                if limit != 255:
                    # we've hit a limit switch
                    if limit == X_POSITIVE_LIMIT:
                        state.hit_limit(x_pos, DIRECTION_POSITIVE)
                    elif limit == X_NEGATIVE_LIMIT:
                        state.hit_limit(x_pos, DIRECTION_NEGATIVE)
                    elif limit == Y_POSITIVE_LIMIT:
                        state.hit_limit(y_pos, DIRECTION_POSITIVE)
                    elif limit == Y_NEGATIVE_LIMIT:
                        state.hit_limit(y_pos, DIRECTION_NEGATIVE)
                    elif limit == Z_POSITIVE_LIMIT:
                        state.hit_limit(z_pos, DIRECTION_POSITIVE)
                    elif limit == Z_NEGATIVE_LIMIT:
                        state.hit_limit(z_pos, DIRECTION_NEGATIVE)
                        
                    state.set_estop(True)


            # 9. Process x-axis power feed buttons 
            if sw_pwr_feed_low:
                if not state.x_axis_moving:
                    state.set_power_feed_low_limit(x_pos)
                    pi.write(PIN_POWER_FEED_LOW_LIGHT, 1)
                    time.sleep(0.1)
                    pi.write(PIN_POWER_FEED_LOW_LIGHT, 0)
                    time.sleep(0.1)
                    pi.write(PIN_POWER_FEED_LOW_LIGHT, 1)
                    time.sleep(0.1)
                    pi.write(PIN_POWER_FEED_LOW_LIGHT, 0)
                    time.sleep(0.1)
                    pi.write(PIN_POWER_FEED_LOW_LIGHT, 1)

            if sw_pwr_feed_high:
                if not state.x_axis_moving:
                    state.set_power_feed_high_limit(x_pos)
                    pi.write(PIN_POWER_FEED_HIGH_LIGHT, 1)
                    time.sleep(0.1)
                    pi.write(PIN_POWER_FEED_HIGH_LIGHT, 0)
                    time.sleep(0.1)
                    pi.write(PIN_POWER_FEED_HIGH_LIGHT, 1)
                    time.sleep(0.1)
                    pi.write(PIN_POWER_FEED_HIGH_LIGHT, 0)
                    time.sleep(0.1)
                    pi.write(PIN_POWER_FEED_HIGH_LIGHT, 1)

            # if both high and low limits are set, allow the user to use the 'push to cycle' button (here called 'bounce')
            if state.power_feed_low_limit and state.power_feed_high_limit:
                state.power_feed_mode = True
                if state.power_feed_high_limit <= state.power_feed_low_limit:
                    # the user has them backwards; cannot use them
                    state.power_feed_high_limit = 0.000
                    state.power_feed_low_limit = 0.000

                    pi.write(PIN_POWER_FEED_LOW_LIGHT, 0)
                    pi.write(PIN_POWER_FEED_HIGH_LIGHT, 0)
                    
                    state.power_feed_mode = False

                if state.power_feed_mode:
                    okay_to_bounce = 0
                    if not state.x_axis_moving:
                        if state.delta_x(x_pos, state.power_feed_low_limit):
                            okay_to_bounce = DIRECTION_POSITIVE
                        if state.delta_x(x_pos, state.power_feed_high_limit):
                            okay_to_bounce = DIRECTION_NEGATIVE
                        
                        if pi.read(PIN_BTN_POWER_FEED_BOUNCE) == 0 and okay_to_bounce != 0:
                            emit_power_feed(okay_to_bounce)

                        # you can also use the turbo button on the hand unit to initiate cycle bounce
                        if btn_turbo and okay_to_bounce != 0:
                            emit_power_feed(okay_to_bounce)
                            
                

            # 10. Set the coordinate system and update display
            x_pos -= state.work_offset_x;
            y_pos -= state.work_offset_y;
            z_pos -= state.work_offset_z;

            multiplier = 1
            if state.units == "mm":
                multiplier = 25.4 
                x_pos *= multiplier
                y_pos *= multiplier
                z_pos *= multiplier

            
            # Formulate text arrangements complying with exactly 20x4 constraints

            # Line 1 of the display shows either the X coordinate or the X power
            # feed status
            if state.power_feed_mode:
                line_1 = f"{state.power_feed_low_limit * multiplier:.2f} - {state.power_feed_high_limit * multiplier:.2f}"
                length = len(line_1)
                line_end = length - 16
                if line_end >= 1:
                    line_1 += " " * line_end
                line_1 += "FEED"
            else:
                if state.current_axis == 'X':
                    line_1 = f">X: {x_pos:+.3f} {state.units}"
                else:
                    line_1 = f" X: {x_pos:+.3f} {state.units}"

                if state.is_limit_set('X'):
                    line_1 += " LIMIT"

            if state.current_axis == 'Y':
                line_2 = f">Y: {y_pos:+.3f}"
            else:
                line_2 = f" Y: {y_pos:+.3f}"

            if state.is_limit_set('Y'):
                line_2 += " LIMIT"

            # line 2 also shows the power feed rate, if in that mode
            if state.power_feed_mode:
                length = len(line_2)
                line_end = 18 - length
                line_2 += " " * line_end
                line_2 += f"{state.power_feed_rate}"
                
            if state.current_axis == 'Z':
                line_3 = f">Z: {z_pos:+.3f}"
            else:
                line_3 = f" Z: {z_pos:+.3f}"

            if state.is_limit_set('Z'):
                line_3 += " LIMIT"

            if state.estop:
                line_4 = "E-STOP"
            else:
                # Spindle is 0 - 5000 RPM, so we multiply by 50 from duty cycle percentage
                spindle_status = "ON " if state.spindle_on else "OFF"
                spindle_rpm = state.spindle_duty * 50
                line_4 = f"SPINDLE {spindle_status} {spindle_rpm} RPM"

            lcd_display.display_lines(line_1, line_2, line_3, line_4)

            last_mpg_count = mpg_count
            last_poll_time = this_poll_time
            old_x  = x_pos
            
    except KeyboardInterrupt:
        print("\nHalting CNC Pendant Control Program Execution Threads Safely...")
    finally:
        # Tear down connection pins and listeners gracefully
        mpg_encoder.shutdown()
        spindle_encoder.shutdown()
        if pi:
            pi.stop()

        serial_comm.close()
        lcd_display.clear()
        print("Pendant shutdown sequence complete.")


if __name__ == '__main__':
    main()


