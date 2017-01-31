# Changelog for Pass the Butter
*This project does not use semantic versioning*

## Issues
- The controller heading suddenly goes haywire and the robot goes in circles
- Only way to detect that Entry boxes have had their text change is by hitting the Enter key

## Unreleased
- Investigate why the encoder readings received by the simulator are one cycle behind  being read by the robot
- Update function(s) for receiving encoder readings to be more efficient and not slow down simulation
- Add to GUI feedback if robot serial connection successfully found or not
- Add function(s) to receive proximity sensor readings from the physical robot

## 2017-01-31
- Unsuccessful trying to find the reason for the controller heading going haywire

## 2017-01-30
- Unsuccessful trying to find the reason for the controller heading going haywire
- Changed so that if robot isn't connected, simulated encoder readings are used instead (with simulated encoder the controller heading is stable)

## 2017-01-29
- Changed supervisor so reads encoder values from teh physical robot
- Add function to convert / compare received encoder values to previous step encoder values
- Add function to compare received encoder values with simulation encoder values, and adjust wheel vel to correct so robot matches simulation

## 2017-01-23
- Fixed conversion of encoder readings from char to decimal
- Updated encoder read function so that it is converting the received encoder readings to the correct value from the serial stream
- Added to robot's step_motion function a comparison between the simulation encoder ticks and the physical robot encoder tick value

## 2017-01-19
- Changed ZumoDriveControl.ino to print encoder value to the LCD (encoder value of previous read is printed, so that when simulator stopped the last encoder reading on the screen matches the last encoder reading printed from the serial read function in the simulator
- Verified that the encoder readings received as hex via the serial stream matches the encoder readings on the robot (they are split into chars to send them)

## 2017-01-18
- Added function(s) to receive encoder readings from the physical robot, read encoder value thru RobotPhysicalInterface via serial
- Changed ZumoDriveControl.ino to send back encoder value of left and right wheel when receive char 'E'

## 2017-01-17
- Updated RobotComm to be a bit more efficient (more work still to be done)
- Added world time clock to GUI

## 2017-01-16
- Fixed save and play button images are not displaying/updating properly
- updated control configuration parameters to load into their respective text entry boxes when control config file loaded.
- Changed control config parameters so they're linked to the controller class they are used by

## 2017-01-15
- Added ability to edit controller PID values and the refresh rate to the main GUI window

## 2017-01-14
- Changed how robot parameter values are setup so they are a list that can be updated when changes in the text entry boxes are applied
- Discovered that the avoid obstacles controller wasn't working because it doesn't react to obstacles if it approaches the obstacle's corner.

## 2017-01-11
- Changed GUI so robot parameters entered through a pop-up window as access to change them isn't needed often

## 2017-01-10
- Added the proximity sensor parameters to the GUI
- Added the current control state to the GUI

## 2017-01-09
- Added the sensor parameters to the GUI
- Updated the parameters to be stored in dynamically filled tables and referenced with an id rather than by name to make it easier to add and remove them later

## 2017-01-07
- Added the rest of the robot config parameters to the GUI and to the save/load config functions

## 2017-01-06
- Added save and load buttons for robot configuration parameters - these save and then restore values into the text entry boxes
- Updated the UML file for the project (renamed as pass-the-butter.xml) to correctly reflect the class structure of the project code

## 2017-01-05
- Added text input boxes to GUI for robot parameters - robot parameters are loaded from them during simulator load and reset

## 2017-01-04
- Fixed step button so that only 1 time period passes between clicks
- Fixed pause button so that time is reset when simulation resumes
- Added vertical box to right side of simulator gui for loading physical robot parameters

## 2016-09-25
- Changed name of the project to 'Pass the Butter' to reflect the new direction and aim of the project
- Changed README to describe the project aims and direction going forward
- Changed robot parameter constants at beginning of robot.py to match physical Zumo32U4 robot
- Changed ZumoDriveControl.ino to more efficiently decode serial speed commands by receiving them as 2 chars that are bitshifted back to speeds in the range 0 to 400 rpm
- Changed robot.py to send serial speed commands encoded as 2 chars using bitshifting
- Added docstrings so now docstrings are complete for all included python modules

## 2016-07-14
- Added Controller parent class for common attributes and methods, especially print_vars
- Added licence & contact details to all files
- Added docstrings to environment.py, gui.py, simulator.py and
utilities.py
- Added the matching arduino file for the Zumo32U4 robot
controller
- Fixed minor errors in controller.py

## 2016-06-27
- Initial WIP upload of the project to github with the changes made so far after forking sobot-rimulator
