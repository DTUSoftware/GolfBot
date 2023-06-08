import asyncio
import os
import aiohttp
import requests


ROBOT_API_ENDPOINT = os.environ.get('API_ENDPOINT', "http://localhost:8069/api/v1")

async def set_robot_start(session: aiohttp.ClientSession):
    print("Setting robot start")
    async with session.post(f"{ROBOT_API_ENDPOINT}/start") as response:
        print(await response.text())
        if response.status != 200:
            print(response.status)

def set_robot_stop():
    response = requests.post(f"{ROBOT_API_ENDPOINT}/stop")
    print(response.text)
    if response.status_code != 200:
        print(response.status_code)
async def get_robot_status(session: aiohttp.ClientSession):
    print("Getting robot status")
    async with session.get(f"{ROBOT_API_ENDPOINT}/status") as response:
        print(await response.text())


async def set_robot_position(session: aiohttp.ClientSession, x: int, y: int):
    print("Setting robot position")
    async with session.post(f"{ROBOT_API_ENDPOINT}/position?x={x}&y={y}") as response:
        print(await response.text())
        if response.status != 200:
            print(response.status)
        else:
            # Let the robot drive a lil' bit
            await asyncio.sleep(1)


async def set_speeds(session: aiohttp.ClientSession, speed_left, speed_right):
    async with session.post(
            f"{ROBOT_API_ENDPOINT}/drive?speed_left={speed_left}&speed_right={speed_right}") as response:
        if response.status != 200:
            print(f"Error on adjusting speed: {response.status}")

async def toggle_fans(session: aiohttp.ClientSession):
    async with session.post(
            f"{ROBOT_API_ENDPOINT}/toggle_fans") as response:
        if response.status != 200:
            print(f"Error on toggling fans: {response.status}")