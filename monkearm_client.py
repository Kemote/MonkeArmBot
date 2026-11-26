
"""Smoke test: subscribe to Joint1/Position and print updates for a few seconds."""
import asyncio
 
from asyncua import Client
 
URL = "opc.tcp://0.0.0.0:4840/monkeArm"
NS = "urn:monkearm:arm"


# TODO rozkimninc

 
class Handler:
    def __init__(self):
        self.count = 0
 
    def datachange_notification(self, node, val, data):
        self.count += 1
        if self.count <= 5:
            print(f"update {self.count}: {val:.3f}")
 
 
async def main():
    async with Client(url=URL) as client:
        idx = await client.get_namespace_index(NS)
        joints_list = [f"Joint{i}" for i in range(1, 7)]
        while(True):
            for joint in joints_list:
                pos_deg = await client.nodes.objects.get_child([f"{idx}:Arm", f"{idx}:{joint}", f"{idx}:pos_deg"])            
                handler = Handler()
                sub = await client.create_subscription(50, handler)
                await sub.subscribe_data_change(pos_deg)
                await asyncio.sleep(0.5)
                # print(f"total updates in 3 s: {handler.count}")
 

asyncio.run(main())
 
