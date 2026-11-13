#!/usr/bin/env python3

import asyncio
import math

from mavsdk_grpc import System
from mavsdk_grpc.offboard import PositionNedYaw, VelocityBodyYawspeed, OffboardError


async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Arming")
    await drone.action.arm()

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    async def print_z_velocity(drone):
        async for odom in drone.telemetry.position_velocity_ned():
            print(f"up: {-odom.velocity.down_m_s} m/s")

    async def print_yaw_rate(drone):
        async for rate in drone.telemetry.attitude_angular_velocity_body():
            print(f"yaw rate: {math.degrees(rate.yaw_rad_s)} deg/s")

    _tasks = {
        asyncio.create_task(print_z_velocity(drone)),
        asyncio.create_task(print_yaw_rate(drone)),
    }

    print("-- Go up to 3m")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -3.0, 0.0))
    await asyncio.sleep(10)

    print("-- Spin at 20 deg/s for 30 seconds")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 20.0))
    await asyncio.sleep(30)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    print("-- Landing")
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("-- Landed")
            break


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
