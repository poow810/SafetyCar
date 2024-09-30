from fastapi import FastAPI
import httpx

app = FastAPI()

SPRING_BOOT_URL = 'https://j11b209.p.ssafy.io/api'


async def send_coordinate(x, y):
    async with httpx.AsyncClient() as client:
        response = await client.post(SPRING_BOOT_URL + '/coordinate', json={"x": x, "y": y})
    try:
        return {"status": "success", "response": response.json()}
    except ValueError:
        print("Error parsing JSON:", response.content)
        return {"status": "error", "response": response.content}