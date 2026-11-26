import asyncio
import logging

from typing import Dict
from driver import monkearm_driver, sts
from asyncua.server import Server


ENDPOINT = "opc.tcp://0.0.0.0:4840/monkeArm"
NAMESPACE_URI = "urn:monkearm:arm"
UPDATE_HZ =30
JOINTS = [f"Joint{i}" for i in range(1, 7)]
FIELDS = ["pos_deg", "offset", "temp"]


log = logging.getLogger("monkearm.server")


async def build_ua_server():
    server = Server()
    await server.init()
    server.set_endpoint(ENDPOINT)
    server.set_server_name("MonkeArm OPC UA server")
    idx = await server.register_namespace(NAMESPACE_URI)
    arm = await server.nodes.objects.add_object(idx, "Arm")
    nodes: Dict[str, Dict[str, object]] = {}

    # wee need to register joints and varaibles in namespace
    for joint in JOINTS:
        jnode = await arm.add_object(idx, joint)
        nodes[joint] = {}
        for field in FIELDS:
            nodes[joint][field] = await jnode.add_variable(idx, field, 0.0)

    return server, nodes


async def main():
    server, nodes = await build_ua_server()
    loop = asyncio.get_running_loop()
    period = 1.0 / UPDATE_HZ

    async with server:
        log.info("Serving on %s", ENDPOINT)
        while True:
            started = loop.time()

            try:
                state = await loop.run_in_executor(None, monkearm_driver.get_info_all)
            except Exception:
                log.exception("driver.read_state failed")
                await asyncio.sleep(0.5)
                continue
            
            for joint, values in state.items():
                for field, value in values.items():
                    node = nodes.get(joint, {}).get(field)
                    if node is not None:
                        await node.write_value(float(value))

            await asyncio.sleep(max(0.0, period - (loop.time() - started)))


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    asyncio.run(main())