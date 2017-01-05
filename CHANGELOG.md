# Changelog for Pass the Butter
*This project does not use semantic versioning*

## Unreleased
- Add ability to load physical robot parameters from a config file
- Add world time clock to GUI
- Receive proximity sensor and encoder readings from the physical robot

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
