# Repair On Board Bot - ROBB
Code repo for BYU Robotics Advanced Team competing in USU's SpaceBot competion 2024. Project comprises building a 6-DOF robotic arm, contolled by ROS noetic via a remote station.

The purpose of this repo is allow for version control and for an easier method to get code onto the raspberry pi controlling the bot. 
Contact me if you or someone you knows wants to know how to apply this project or has questions, I would be happy to help.

## Installation
1. Clone this repo to the local machine connected to your robot system (i.e. rasberrry pi)
2. Install ROS Noetic (if not already installed)
3. Install other packages and drivers as necessary (will create a list later in the project)
4. Celebrate

### Notes on setting up and connecting
To share ros info, add the following to your slave machine (control station) and master machine (robot) respectively

For slave:

`export ROS_MASTER_URI=http://XXX.XXX.X.XXX:11311`

`export ROS_IP=*YOUR_IP*`

For master:

`export ROS_MASTER_URI=http://XXX.XXX.X.XXX:11311`

`export ROS_IP=XXX.XXX.X.XXX`

Note: XXX.XXX.X.XXX is the IP of your master, in my case the raspberry pi

## Contributing
Contributions are welcome! Please follow these steps:
1. Fork this repository.
2. Create a new branch for your feature/fix.
3. Submit a pull request.

## License
This project is licensed under the MIT License - see the LICENSE file for details.
