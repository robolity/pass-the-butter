## Pass the Butter - A Robot Programming Tool

by Justin Clarke (2016) github.com/robolity/pass-the-butter

Inspired by Butter Robot in the tv show Rick and Morty, I want to a build a low cost robot that can pass the butter. As in, you can tell your robot "Go pass me the butter", and it will drive along your kitchen table searching for the butter. And when it finds the butter, it'll push it back along the table to you, ready for your toast in the morning.

This will require interesting and challenging autonomous navigation techniques, and implementing them to work with a real-world robot using sensors for feedback. Also, a key aim is to make it work with really basic robots that cost less than $100 to build. My motivation for this project is to learn all this myself, and hopefully develop an open source robot kit and software that is super accessible and gets people into robotics!

If you want to help develop this project, or have any questions at all, please contact me at robolity@gmail.com. I'll be super happy to hear from you!

### Project Aims

For a human, passing the butter is a really simple task. But for a robot, just passing the butter is a huge, huge task. Especially for a low cost robot with basic sensors. But that doesn't mean it's impossible and the task can be broken down in the following way:

#### Step 1: Searching for the butter
Robot is given a direction to search in for the butter and drives in a straight line in that direction. If it encounters an obstacle along the way, it drives around it and tries to return to the straight line path it was travelling in before seeing the obstacle. If possible, the boundaries of the obstacles will be saved into the robots memory, so that it remembers the layout of obstacles that were in its way on the kitchen table.

#### Step 2: Find the butter
A real challenge will be how the robot will know when it's found the butter. My ideas so far are:
- RFID tag on the butter that the robot reads - limited range of RFID may make this impractical
- Using a small camera on the robot, visually identify the butter - more expensive option

#### Step 3: Holding onto the butter
Once the robot has found the butter, it will need to push it along the kitchen table towards you. It may needs to hold onto the butter while pushing it, and adding prongs to the front of the robot that sit either side of the butter should keep the butter in place. The robot will then head back to where it was when you first sent it off in search of the butter. It will autonomously navigate back there in the same way it did when searching for the butter.

#### Step 4: Butter your toast!
With the robot back next to, holding onto the butter in triumph, it's now time for you to reward it's hard work and butter up your toast.

### Project Overview

At this stage, the project is at that stage of developing Step 1 above. A Zumo 32U4 robot is being used for testing the control software, which is written in Python. However, the software will be for any differential drive robot, eventually using a config file to load individual robot parameters. The software provides a GUI interface showing the location of the robot, obstacles, and goal location. The robot then navigates past the obstacles to the goal location. The control software talks to the robot via serial commands, and reads the proximity sensor and encoder values of the robot.

#### Robot Specs

The project uses an arduino based differential drive (2 wheeled) robot with infr-red proximity sensors and encoders on the wheels. The control algorithms for differential drive robots are fairly straight forward, and are covered in the excellent Coursera course 'Control of Mobile Robots'. The proximity sensors will provide distance measurements to any nearby obstacles as the robot drives along. The wheel encoders will be used to check if the wheels are turning as much as the controller thinks they have, so that the position of the robot can be tracked.

![Zumo Robot](/Images/zumo_robot.jpg)
*Zumo robot with Raspberry Pi to run the software and its power supply*

### Software Specs

The control software for this project is forked from github.com/nmccrea/sobot-rimulator, which took inspiration from the Coursera course Control of Mobile Robots by Georgia Institute of Technology.

This control software is written in Python and creates a GUI screen showing a robot, or robots, in a to scale representation of a room. The room contains red obstacles and a green goal location, and the robot will navigate towards the goal location, avoiding any obstacles in its path. The controller can be used for any 2 wheeled, differential drive robots. You will need to edit the constants at the top of the robot.py file to suit different robots.

Obstacles can be drawn to match real-world obstacles and the program will use infra-red proximity sensor readings to avoid the obstacles. Go to goal, avoid obstacle, and follow wall path planning procedures are currently implemented, with the robot switching between them while on it's journey to the goal.

![Zumo Robot](/Images/control_software.png)
*Screenshot of the software showing the visualisation of the control*

### Requirements
*Pass the Butter* is cross-platform compatible. In order to run, the following two items must be installed on your computer:
- Python 2.6.1 or higher
- PyGTK 2.7 or higher

To install Python: The latest Python interpreter can be found at http://www.python.org/download/.

To install PyGTK: The latest PyGTK distributions can found at http://www.pygtk.org/downloads.html.

Alternatively, both pieces of software should be available through package managers such as Apt-Get (for Linux/Unix) or Homebrew (Recommended for Mac)

### To Run
Connect to your robot via a serial connection to the computer running the python control software. This can either be a wired connection such as USB, or a wireless connection such as Bluetooth. The control software searches through all the serial ports sending the command 'V'. If it sees the response 'butter' then it knows it's found the right serial port. So you need to program your robot to send back the string 'butter' when it receives the character 'V'. If using a Zumo32U4 robot, you can upload the included ZumoDriveControl.ino arduino file to your robot.

To run the control software, open a command prompt (terminal) and navigate to the *pass-the-butter* directory. Then type:

    python simulator.py

The control software GUI shown above should appear and you control the robot using the following buttons:
- Play: Starts the robot driving
- Pause: Stops the robot (resume it driving by hitting Play again)
- Step: Steps the control software through one calculation cycle
- Reset: Resets the control software to before the robot started moving and stops the robot

- Save Map: You can save the currently shown obstacles and goal as a map to be loaded later
- Load Map: Load a map of obstacles and goal as the current environment
- Random Map: Generates an environment with randomly placed obstacles and goal

- Show Invisibles: Shows the currently traversed path of the robot and its sensor readings
