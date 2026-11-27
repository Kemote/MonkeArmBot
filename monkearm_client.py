"""Smoke test: find every arm on the server, subscribe to all joint positions for a few
seconds, and print per arm how many updates came and the last positions."""
import asyncio

from asyncua import Client, ua

URL = "opc.tcp://0.0.0.0:4840/monkeArm"
NS = "urn:monkearm:arm"
SECONDS = 1


class Handler:
    """Counts the updates of each arm and keeps the last value of each node."""

    def __init__(self, arm_of):
        self.arm_of = arm_of                          # {position node: arm name}
        self.count = dict.fromkeys(arm_of.values(), 0)
        self.last = {}

    def datachange_notification(self, node, val, data):
        self.count[self.arm_of[node]] += 1
        self.last[node] = val


async def main():
    async with Client(url=URL) as client:
        idx = await client.get_namespace_index(NS)
        arms = {}                                     # {arm name: (MAC, virtual, {joint: position node})}
        while True:
            for arm in await client.nodes.objects.get_children(nodeclassmask=ua.NodeClass.Object):
                name = await arm.read_browse_name()
                if name.NamespaceIndex != idx:
                    continue                              # the server's own objects
                mac = await (await arm.get_child(f"{idx}:MacAddress")).read_value()
                virtual = await (await arm.get_child(f"{idx}:Virtual")).read_value()
                positions = {}
                for joint in await arm.get_children(nodeclassmask=ua.NodeClass.Object):
                    positions[(await joint.read_browse_name()).Name] = await joint.get_child(f"{idx}:position")
                arms[name.Name] = (mac, virtual, positions)

            arm_of = {node: arm for arm, (_, _, positions) in arms.items() for node in positions.values()}
            handler = Handler(arm_of)
            sub = await client.create_subscription(50, handler)
            await sub.subscribe_data_change(list(arm_of))
            await asyncio.sleep(SECONDS)
            await sub.delete()

            print(f"{len(arms)} arms, position updates in {SECONDS} s:")
            for arm, (mac, virtual, positions) in arms.items():
                last = " ".join("-" if handler.last.get(n) is None else f"{handler.last[n]:+.3f}"
                                for n in positions.values())
                kind = "virtual" if virtual else "real"
                print(f"  {arm} {mac} {kind:<7} {handler.count[arm]:>4} updates, last [rad]: {last}")


asyncio.run(main())
