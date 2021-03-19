#!/usr/bin/env python3

import asyncio
from mavsdk import System


async def run():
    drone = System()
    # await drone.connect(system_address="udp://:14540")
    await drone.connect(system_address="serial:///dev/ttyACM0")
    # await drone.connect(system_address="serial:///dev/ttyACM1")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    info = await drone.info.get_version()

    while True:
        print("Waiting for drone to have a global position estimate...")
        async for health in drone.telemetry.health():
            print(health)
            # if health.is_global_position_ok:
            #     print("Global position estimate ok")
            #     break


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
