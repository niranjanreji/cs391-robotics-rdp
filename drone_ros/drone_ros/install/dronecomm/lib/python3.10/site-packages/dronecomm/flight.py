import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import asyncio
from mavsdk import System

class Pub(Node):
    def __init__(self):
        super().__init__('dronecomm_status')
        self.status_out = self.create_publisher(Bool, 'flight_status',10)

    def update_status(self, stat):
        msg = Bool()
        msg.data = stat
        self.status_out.publish(msg)
        self.get_logger().info(f"Publishing {msg} for flight status")

roll = ""
pitch = ""
throttle = ""
yaw = ""

async def manual_controls(drone):
    global roll, pitch, yaw, throttle
    while True:
        await drone.manual_control.set_manual_control_input(pitch, roll, throttle, yaw)

async def main():
    """Main function to connect to the drone and input manual controls"""
    rclpy.init()
    p = Pub()
    p.update_status(False)
    global roll, pitch, yaw, throttle
    # Connect to the Simulation
    drone = System(mavsdk_server_address='localhost', port=50051)
    await drone.connect(system_address="serial:///COM4:57600")

    # This waits till a mavlink based drone is connected
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    # Checking if Global Position Estimate is ok
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    # Arming the drone
    print("-- Arming")
    await drone.action.arm()

    # Take off
    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    print("-- Set manual control")
    roll = 0.0
    pitch = 0.0
    throttle = 0.5
    yaw = 0.0
    # set the manual control input after arming
    manual = asyncio.create_task(manual_controls(drone))
    print("-- wait")
    await asyncio.sleep(1)

    p.update_status(True)

    # start manual control
    print("-- Starting manual control")
    await drone.manual_control.start_position_control()
    print("-- wait")
    await asyncio.sleep(1)
    print("-- Change manual control")
    for _ in range(2):
        roll = 0.1
        pitch = 0.6
        throttle = 0.6
        yaw = 0.0
        await asyncio.sleep(3) #forward

        roll = -0.3
        pitch = 0
        throttle = 0.5
        yaw = 0.0
        await asyncio.sleep(2) #left


        roll = -0.1
        pitch = -0.6
        throttle = 0.6
        yaw = 0.0
        await asyncio.sleep(4) #backward

        roll = 0.3
        pitch = 0
        throttle = 0.5
        yaw = 0.0
        await asyncio.sleep(2) #right
    
    roll = 0.1
    pitch = 0.6
    throttle = 0.6
    yaw = 0.0
    await asyncio.sleep(3) #finishing by forward

    p.update_status(False)
    await drone.action.land()
    print("-- end")
    p.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(main())

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(main())

