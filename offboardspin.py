#!/usr/bin/env python3

import asyncio

from mavsdk_grpc import System
from mavsdk_grpc.offboard import PositionNedYaw, VelocityBodyYawspeed, OffboardError
from mavsdk_grpc.telemetry import FlightMode

TAKEOFF_ALT_M = 3.0
SPIN_RATE_DEG_S = 20.0
SPIN_TIME_S = 30.0


async def get_position_ned(drone):
    async for pos_vel in drone.telemetry.position_velocity_ned():
        return pos_vel.position


async def get_yaw_deg(drone):
    async for attitude in drone.telemetry.attitude_euler():
        return attitude.yaw_deg


async def watch_for_pilot_takeover(drone, pilot_took_over):
    """Set the flag if the drone leaves offboard mode after entering it."""
    seen_offboard = False
    async for mode in drone.telemetry.flight_mode():
        if mode == FlightMode.OFFBOARD:
            seen_offboard = True
        elif seen_offboard:
            pilot_took_over.set()
            return


async def wait_or_takeover(pilot_took_over, seconds):
    """Wait for `seconds`. Returns True if the pilot took over during the wait."""
    try:
        await asyncio.wait_for(pilot_took_over.wait(), timeout=seconds)
        return True
    except asyncio.TimeoutError:
        return False


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

    # Let PX4 do the takeoff itself. It lifts off straight up from where the
    # drone is sitting and keeps its current heading.
    print(f"-- Arming and taking off to {TAKEOFF_ALT_M} m")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
    await drone.action.arm()
    await drone.action.takeoff()

    async for position in drone.telemetry.position():
        if position.relative_altitude_m >= TAKEOFF_ALT_M - 0.3:
            print("-- Reached takeoff altitude")
            break
    await asyncio.sleep(3)

    # Hold exactly where we are now: current position and current heading.
    here = await get_position_ned(drone)
    yaw_deg = await get_yaw_deg(drone)
    print(f"-- Holding at N {here.north_m:.1f} m, E {here.east_m:.1f} m, "
          f"D {here.down_m:.1f} m, yaw {yaw_deg:.0f} deg")
    await drone.offboard.set_position_ned(
        PositionNedYaw(here.north_m, here.east_m, here.down_m, yaw_deg))

    pilot_took_over = asyncio.Event()
    watcher = asyncio.create_task(watch_for_pilot_takeover(drone, pilot_took_over))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Landing")
        await drone.action.land()
        return

    if await wait_or_takeover(pilot_took_over, 5):
        print("-- Pilot took over. Script is stopping and will send no more commands.")
        return

    print(f"-- Spin at {SPIN_RATE_DEG_S} deg/s for {SPIN_TIME_S} seconds")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, SPIN_RATE_DEG_S))
    if await wait_or_takeover(pilot_took_over, SPIN_TIME_S):
        print("-- Pilot took over. Script is stopping and will send no more commands.")
        return

    watcher.cancel()

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
    asyncio.run(run())
