# llm_utils/phi2_client_async.py

import aiohttp
import asyncio

class AsyncPhi2Client:
    def __init__(self, endpoint="http://localhost:8000/generate", timeout=10):
        self.endpoint = endpoint
        self.timeout = timeout

    async def query(self, prompt, temperature=0.7, max_tokens=128):
        payload = {
            "prompt": prompt,
            "temperature": temperature,
            "max_tokens": max_tokens
        }
        try:
            async with aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=self.timeout)) as session:
                async with session.post(self.endpoint, json=payload) as response:
                    response.raise_for_status()
                    data = await response.json()
                    return data.get("response", "")
        except Exception as e:
            print(f"[Phi2Async Error] {e}")
            return "[ERROR]"
