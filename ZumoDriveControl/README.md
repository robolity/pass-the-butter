ZumoDriveControl.ino is the Arduino file for the Zumo32U4 robot so that it 
can receive and interpret the serial commands from the Zumo Simulator.

1. Connect your computer via USB to the Zumo32U4 robot
2. Open the file with Arduino IDE
3. Select Tools -> Board -> Pololu A-Star 32U4
4. Select Port -> COM_ (the COM port the A-Star is listed against)
5. Upload to the Zumo32U4

The Zumo Simulator transmits serial commands via the USB serial interface.

These commands are the speed and direction each wheel should turn so that
the Zumo32U4 robot follows the same path as shown in the simulator.
