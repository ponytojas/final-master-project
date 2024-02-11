import asyncio
import json
from aiocoap import *
from aiocoap.resource import Resource, Site
import random

class BasicResource(Resource):
    def __init__(self):
        super().__init__()
        self.data = {
            "speed": 0,
            "position": [0.0, 0.0],
            "bearing": 0.0
        }

    async def render_get(self, request):
        payload = json.dumps(self.data).encode('utf-8')
        return Message(content_format=50, payload=payload)

async def update_resource_data(resource):
    while True:
        resource.data["speed"] = random.randint(1, 100)
        resource.data["position"] = [random.uniform(-100, 100), random.uniform(-100, 100)]
        resource.data["bearing"] = random.uniform(0, 360)
        print("Datos actualizados:", resource.data)  # Opcional, para verificación
        await asyncio.sleep(5) 

async def main():
    site = Site()
    basic_resource = BasicResource()
    site.add_resource(['resource'], basic_resource)

    await Context.create_server_context(site)

    task = asyncio.create_task(update_resource_data(basic_resource))

    await asyncio.get_running_loop().create_future()

if __name__ == "__main__":
    asyncio.run(main())
