import asyncio
import json
from aiocoap import *

DELAY_BETWEEN_REQUESTS = 1 
DELAY_BETWEEN_ERRORS = 2

async def fetch_resource(protocol):
    request = Message(code=GET)
    request.set_request_uri("coap://localhost/resource")

    try:
        response = await protocol.request(request).response
        print('Result: %s\n%r' % (response.code, json.loads(response.payload.decode('utf-8'))))
        return True  # Indica que la solicitud fue exitosa
    except Exception as e:
        print('Failed to fetch resource:', e)
        return False

async def main():
    while True:
        protocol = await Context.create_client_context()
        success = await fetch_resource(protocol)
        await protocol.shutdown()

        if not success:
            print("Retrying in", DELAY_BETWEEN_ERRORS, "seconds...")
            await asyncio.sleep(DELAY_BETWEEN_ERRORS)
        else:
            await asyncio.sleep(DELAY_BETWEEN_REQUESTS)

if __name__ == "__main__":
    asyncio.run(main())
