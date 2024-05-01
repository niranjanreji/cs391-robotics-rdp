# cs391-robotics-rdp
A bunch of files from a robotics project done in Spring 2024. 

The drone (most of its parts, at least) can be purchased from https://holybro.com/products/px4-development-kit-x500-v2.

Instructions to put the parts together once they arrive can be found here: https://www.youtube.com/watch?v=27rbxCeCq4Y&t=558s
Some slight modifications need to be made to the process of assembly. 
3D print the step file that is provided in this github repository. It is an attachment for the drone.
Drill holes on both large planar faces of the mount. Put the Pico on one end, and the LIDAR sensor on the other. 
Wire them up together following the wiring diagram provided in the report.
Power the Pico using TELEM2 (or POWER2) on the drone - look up ways to hook them up.
Hook up one of the telemetry radios to the Pixhawk, and the other one to your ground station/ROS running laptop.

The ROS package used to control the drone is provided in the repository.
flight.py contains all of the code needed to achieve flight.
However, asyncio does not work with ROS. To achieve flight, it will be necessary to get the asyncio loop out.
Remove all of the ROS references within the asyncio main function to get flight.py to work.
Remember to select the COM port that the telemetry radio is attached to on your laptop.
To do that, you need to modify flight.py. 

publisher.py contains the code used to collect data from the Pico on the drone over wifi.
It collects data from the Pico and publishes it to a ROS topic called 'lidar_data'.
It only publishes data if the ROS topic 'flight_status' had 'True' published to it recently.
You can get around the need for having flight.py interacting with flight_status by simply removing the ROS code,
and sending the data array being published by publisher.py to subscriber.py instead.

subscriber.py pulls data from 'lidar_data' in the form of an array and plots it live.

flight.py uses MAVSdk, a high level abstraction of MavLink.
MAVSDK is a high-level library that abstracts away the underlying MAVLink communication between drones and ground stations. 
MAVLink is a lightweight messaging protocol for communicating with drones and between onboard drone components.
