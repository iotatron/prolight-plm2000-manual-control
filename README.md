# prolight-plm2000-manual-control

Implements a manual control pendant for the ProLight PLM2000 CNC mill.
Runs on a Raspberry Pi (4 or 5 recommended), and makes use of a
USB-serial adapter to communicate with the mill.  The pendant actually
comes in two pieces, at least the way I built it.  First is a base
unit that has a DRO display, a mm/in units selector, a reset button,
and some controls to operate the X-axis power feed.  Second is the
hand pendant, which contains the MPG wheel, a spindle on/off switch,
an axis selector, and zero and "turbo" buttons.

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
machine is equipped with.  If you can connect to your mill over the
serial port (9600 8N1), and issue the command "ALLDRVON", and the mill
in response reports "All drives turned on.", then this controller will
work for you.
