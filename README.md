# prolight-plm2000-manual-control

INTRO

Implements a manual control pendant for the ProLight PLM2000 CNC mill.
Runs on a Raspberry Pi (4 or 5 recommended), and makes use of a
USB-serial adapter to communicate with the mill.  The pendant actually
comes in two pieces, at least the way I built it.  First is a base
unit that has a DRO display, a mm/in units selector, a reset button,
and some controls to operate an X-axis power feed that is implemented
in this codebase.  Second is the hand pendant, which contains the MPG
wheel, a spindle on/off switch, an axis selector, and zero and "turbo"
buttons.

In this main directory of this repo are the Python control program,
which implements the functionality of a CNC pendant and handles the
mill's command language.  There are also two .service programs, which
can be added to the Raspberry Pi's systemd scripts in order to post
some status messages as the computer starts up and shuts down.  Change
'/home/user' in those .service scripts to match the user account that
you use on the Pi.

In the 'cad' subdirectory, I have put the two FreeCAD files that I
used to 3D print the base unit and the pendant.  Depending on the
buttons, wheels, and rotary encoders you choose, these CAD files will
probably have to be modified.  I have included a JPG photo of my
unit, so you can get a sense of the hardware needed and where it goes.

Information on the Internet about these machines is sketchy.  There
may be two or more different brands of motion controllers that were
used by ProLight during the product lifetime of this mill.  The code
presented in this repo obviously only works on whatever controller my
machine is equipped with.  Now, I'd tell you which controller I have,
but that would require opening the "backpack" on the back side of the
mill.  Unfortunately, I installed my mill close to the wall, so I
don't have any access back there.  And since the mill weighs 400 lbs,
it's not easy to shift it around in order to get access.  So I'm
not going to do it.  But here's what you can do: connect to your mill
over the serial port (9600 8N1), and issue the command "ALLDRVON".  If
the mill responds with "All drives turned on.", then this controller
will work for you.

On the electronics side, this project uses almost every GPIO pin on
the Pi for something.  So the wiring gets pretty dense.  I used a
20-pin "aviation connector" to run between the DRO base unit and the
handheld pendant.  Check the photo.  That seems to work well, so I do
recommend it.  A cheap 4-line LCD display (available on AdaFruit or
Amazon) acts as the DRO.  The MPG wheel was also bought on Amazon.
Check the Python code for the pin assignments.  The Python code does
depend on the pigpio library, which you can install via the package
manager on the Pi.  There are a few other library requirements; check
the 'import' statements in the code.  But they're all common things.

Google Gemini wrote the main hardware interfacing classes
(QuadratureDecoder and LCDDisplay) and provided an initial sketch of
the main loop.  The motion and button processing logic was written by
me.


HOW TO USE

The pendant allows you to use the MPG wheel to move around the table
on the mill.  This lets you turn a dedicated CNC mill into something
much closer to a manual mill.  Which in turn means that you don't have
to spend time writing G-code, if you just want to square up the six
sides of a block of metal.

In practice, what you'll do is move the cutter so that it is just
outside of the material on the left hand side.  Press the left-side
power feed limit button.  It will light up.  Move the table so that
the cutter is off of the right side of the part.  Then press the
right-side feed limit button.  Now both of them will light up.  Use
the feed rate rotary control to select a feed rate.  Getting into
feeds and speeds is way out of scope for this document, but around 10
IPM is probably appropriate for aluminum, whereas steel might need 5
or less and plastic could be around 20.  Note that the feed rate is
always in inches per minute, regardless of whether the main units
switch is set on mm or inches.  Turn on the spindle on the hand
pendant, and set the spindle speed with the rotary encoder there.  If
you now either press the feed rate rotary switch inward or use the
Turbo button on the hand pendant, the machine will power feed across
the part at the feed rate indicated.  Walk the cutter across your part
using the MPG wheel (with the axis selector set to 'Y'), hitting the
cycle start button each time to "bounce" the machine between the
two X-axis limits you set.	

You can do a lot of manual machining with this pendant, especially
given that it has the DRO display.  So drilling precision holes, for
example, is easy enough.  The DRO does not have some of the features
of commercial units, such as a mode for finding the halfway point.
This is mainly a limitation in real estate: there's not enough room on
the pendant or the DRO unit to have every button I might like to have.
Coding up such a feature would be trivial though.


RESETS and E-STOPS

The reset button on the main DRO unit will light up if the mill is in
a halted state.  This could be caused by hitting the e-stop button on
the machine itself, or by running into a limit switch.  If this
happens, you have to do two things.  You have to clear the underlying
problem condition, and you have to get out of reset on the DRO unit.

For the e-stop button on the machine itself, obviously you have to pop
the button back out.  Then hit the reset button on the DRO unit, and
you should be good.

For a limit switch, you have a chicken-and-egg problem.  You need to
move the table to get off the limit switch, but you can't move the
table because the machine is on a limit switch.  So the code
implements the same little dance that I've seen on other CNC units:
you hit the reset button on the main DRO unit.  This will allow the
MPG wheel to move the table a little bit.  But immediately, the
machine will detect that the limit switch is still being depressed, so
it halts again.  You press the reset button again, allowing another
small movement.  After couple of times of doing this, you will hear
the limit switch clicking free, and now you can move the machine
freely again.

Under the hood, whenever the E-stop or a limit switch is hit, the mill
turns off its motor drivers.  To get out of reset, all the pendant
has to do is send the command "ALLDRVON".  


NOTES ON THE PROTOCOL

This was actually the easiest reverse engineering job I've ever done.
The DOS software that runs this machine is still available for
download on the Internet, from whatever company acquired the company
that acquired the company that was ProLight, Inc.  If you sniff the
serial line, you'll see that the protocol between DOS sofware and mill
is essentially just G-Code, with a bunch of inquiry packets thrown in
to check the servo positioning and limit switch status.  And I was
delighted to find that the machine has a easy (but non-standard!)
syntax for relative motions:

G1X[X+0.25]F10

That code, sent just like I wrote it in plain ASCII text, tells the
machine to move +0.25 inches from its current position, at a feed rate
of 10 IPM, on the X axis.  Other axes work just as you'd expect:

G1Y[Y-4.3]F8
G1Z[Z+1.0]F15

Since a hand pendant does nothing but relative motions, that meant all
I had to do was count the number of clicks on the MPG dial the user
made since the last poll, convert that to a relative motion in inches,
and send it to the machine using that exact G-code syntax.  Connect to
your machine over serial and try out those commands.  It really is as
simple as that.

The main other thing you have to implement are the spindle commands,
which are this easy sequence:

1ON
DUTYA50.0
DUTYA0.0
1OFF

In order, that means turn motor 1 on, then set the duty cycle of the motor
to 50% (which corresponds to 2500 RPM; at 100%, the motor is running
at its maximum of 5000 RPM).  Setting the duty cycle to 0 spins the motor
down, then you turn motor 1 off.  Again, really simple.

The binary status requests are 0x02, 0x04, and 0x01.  

0x02: returns a 4-line data block that has the current position of the
X, Y, Z, and one other axis, all listed in ASCII.  The "one other
axis" is probably a 4th rotation axis, as I believe ProLight sold
such a thing.  I don't have one, so my 4th value is always zero.  Note
that when the machine starts up, all servo positions are set to zero.
You'd have to run a homing cycle to change this, but the hand pendant
has no need to do this, because it never cares where absolute (0, 0, 0)
is.

0x04: returns a 3-line data block.  The first line seems to concern
the e-stop.  If the e-stop is set, it will report a value of 64.  The
magnetic switch sensor that detects the debris shield being out of
place may also have some bitcode value in this line, but my machine's
magnetic sensor is disabled, so I didn't implement any such detection.

The second line is a mystery.

The third line is a bitcoded value that tells if a limit switch has
been set.  The code, near the comment line "STATUS BITS", has the
values that are returned for each axis' limit switches being
triggered, if you're interested.  IMPORTANT: Note that the Y-axis
limit switches were wired in reverse on my machine.  So I go
travelling in the positive direction in Y until I hit the limit
switch, and my mill reports (in ASCII, over serial): "-Y limit hit".
If your machine is wired correctly, then you will need to swap the
values Y_POSITIVE_LIMIT and Y_NEGATIVE_LIMIT near the top of the
Python code.  I would suggest connecting to your machine over serial
and running the relative G-Code motion commands listed above to drive
the machine to all 6 limit switches, just so you can check that the
limit switches are wired correctly.  If any limit switch reports -
when you were driving it in the + direction (or vice versa), then you
need to swap that axis' POSITIVE and NEGATIVE LIMIT values.

0x01: returns unknown data.  It may be acceleration parameters from
the servos.  I really don't know, but it does not appear to be
particularly useful for my hand pendant.

You can connect to the mill using serial software (e.g., picocom under
Linux) and issue all these commands, and the machine will respond back
to you in plaintext ASCII.


