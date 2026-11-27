"""OPC UA server for MonkeArm arms: publishes every value of every servo of every arm.

It serves the real arm on a serial port and any number of virtual arms, which move at
random and make up their values (driver/virtual_arm.py), for testing clients with
many arms:

    python ua_server.py                         # the real arm on /dev/ttyUSB0 and 9 virtual arms
    python ua_server.py --virtual 0             # only the real arm
    python ua_server.py --no-real --virtual 3   # three virtual arms, no hardware

Address space: Objects/<arm>/<JointN>/<field>. Each arm is named after the MAC address
of its driver board (Arm_2805A5C4EF34) and has the properties MacAddress
(28-05-A5-C4-EF-34) and Virtual. Virtual arms have the MACs 02-00-00-00-00-01 and up.
Each joint has a ServoId property and one variable for each entry of
driver.fields.FIELDS, with its OPC UA data type and description.

Node IDs are strings built from the path, so they stay the same between runs and
when arms or joints are added: ns=<urn:monkearm:arm>;s=Arm_2805A5C4EF34.Joint1.position.
Properties add their name: Arm_2805A5C4EF34.Joint1.position.EngineeringUnits.

Values with a unit are AnalogUnitType variables (OPC UA Part 8) with an
EngineeringUnits property; those with a known range also have EURange, which for
positions is the joint's limits.

Each arm is read by its own loop on its own thread, so a slow or failed arm doesn't
hold up the others. Settings are published once at start. Positions are read every
cycle, the other live values every SLOW_PERIOD seconds, because they change slowly
and every extra register costs bus time. Each value carries the time it was read as
its SourceTimestamp. When reading an arm fails, that arm's live values get the status
BadCommunicationError until a read works again. OPC UA makes a Bad value null, so a
client sees no value, and the SourceTimestamp stays at the last good read.
"""
import argparse
import asyncio
import logging
import math
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from functools import partial

from asyncua import Node, Server, ua

from driver.fields import CONFIG_FIELDS, FIELDS, LIVE_FIELDS, RAD_PER_STEP
from driver.monkearm_driver import MonkeArmDriver
from driver.servo import OFFSET_MAX
from driver.virtual_arm import VirtualArm, virtual_mac

ENDPOINT = "opc.tcp://0.0.0.0:4840/monkeArm"
NAMESPACE_URI = "urn:monkearm:arm"
PORT = "/dev/ttyUSB0"
VIRTUAL_ARMS = 9        # with the real arm, ten arms
UPDATE_HZ = 15          # six positions take about 64 ms to read through the ESP32 board
FAST_FIELDS = ["position"]                                      # read every cycle
SLOW_FIELDS = [n for n in LIVE_FIELDS if n not in FAST_FIELDS]  # read every SLOW_PERIOD
SLOW_PERIOD = 1.0       # seconds
RETRY_PERIOD = 0.5      # seconds between reads after one failed
UNITS_URI = "http://www.opcfoundation.org/UA/units/un/cefact"

# EURange of each field that has a known range: (low, high), or "limits" for the joint's own limits
RANGES = {
    "position": "limits",
    "goal_position": "limits",
    "min_limit": (-math.pi, math.pi),
    "max_limit": (-math.pi, math.pi),
    "offset": (-OFFSET_MAX * RAD_PER_STEP, OFFSET_MAX * RAD_PER_STEP),
    "load": (-100.0, 100.0),
    "max_torque": (0.0, 100.0),
    "torque_limit": (0.0, 100.0),
}

COMM_ERROR = ua.StatusCode(ua.StatusCodes.BadCommunicationError)

log = logging.getLogger("monkearm.server")


def engineering_units(field):
    """OPC UA EUInformation for a field's unit. UnitId packs the UNECE code (OPC UA Part 8)."""
    unit_id = 0
    for ch in field.unece:
        unit_id = (unit_id << 8) | ord(ch)
    return ua.EUInformation(NamespaceUri=UNITS_URI, UnitId=unit_id,
                            DisplayName=ua.LocalizedText(field.unit), Description=ua.LocalizedText(field.unit))


def eu_range(field, servo):
    """The field's EURange for this joint, or None."""
    r = RANGES.get(field.name)
    if r == "limits":
        low, high = servo.config["min_limit"], servo.config["max_limit"]
        return ua.Range(Low=low, High=high) if low < high else None
    return ua.Range(Low=r[0], High=r[1]) if r else None


async def add_variable(parent, nodeid, name, variant, type_definition, description, standard=False):
    """A read-only variable, created with its type definition and description.

    A property (PropertyType) hangs off its parent by HasProperty, any other variable
    by HasComponent. standard marks a property defined by OPC UA, such as
    EngineeringUnits, whose browse name is in namespace 0; other browse names are in
    the namespace of nodeid.
    """
    item = ua.AddNodesItem()
    item.RequestedNewNodeId = nodeid
    item.BrowseName = ua.QualifiedName(name, 0 if standard else nodeid.NamespaceIndex)
    item.NodeClass = ua.NodeClass.Variable
    item.ParentNodeId = parent.nodeid
    prop = type_definition == ua.ObjectIds.PropertyType
    item.ReferenceTypeId = ua.NodeId(ua.ObjectIds.HasProperty if prop else ua.ObjectIds.HasComponent)
    item.TypeDefinition = ua.NodeId(type_definition)
    attrs = ua.VariableAttributes()
    attrs.DisplayName = ua.LocalizedText(name)
    attrs.Description = ua.LocalizedText(description)
    attrs.Value = variant
    attrs.DataType = (variant.Value.data_type if variant.VariantType == ua.VariantType.ExtensionObject
                      else ua.NodeId(variant.VariantType.value))
    attrs.ValueRank = ua.ValueRank.Scalar
    attrs.AccessLevel = attrs.UserAccessLevel = ua.AccessLevel.CurrentRead.mask
    item.NodeAttributes = attrs
    result, = await parent.session.add_nodes([item])
    result.StatusCode.check()
    return Node(parent.session, result.AddedNodeId)


async def add_arm(objects, idx, arm):
    """The arm's object under objects, with one object per joint and one variable per field.

    Returns {joint name: {field: (variable node, variant type)}}.
    """
    def nid(*path):
        return ua.NodeId(".".join(path), idx)

    a = arm.name
    arm_node = await objects.add_object(nid(a), ua.QualifiedName(a, idx))
    await add_variable(arm_node, nid(a, "MacAddress"), "MacAddress",
                       ua.Variant(arm.mac, ua.VariantType.String), ua.ObjectIds.PropertyType,
                       "MAC address of the arm's driver board")
    await add_variable(arm_node, nid(a, "Virtual"), "Virtual",
                       ua.Variant(arm.virtual, ua.VariantType.Boolean), ua.ObjectIds.PropertyType,
                       "True for a simulated arm, whose values are made up")
    nodes = {}
    for name, servo in arm.joints.items():
        jnode = await arm_node.add_object(nid(a, name), ua.QualifiedName(name, idx))
        await add_variable(jnode, nid(a, name, "ServoId"), "ServoId",
                           ua.Variant(servo.id, ua.VariantType.Byte), ua.ObjectIds.PropertyType,
                           "Servo ID on the bus")
        nodes[name] = {}
        for field in FIELDS.values():
            vtype = ua.VariantType[field.ua_type]
            initial = ua.Variant({"Boolean": False, "Double": 0.0}.get(field.ua_type, 0), vtype)
            analog = field.unece is not None
            path = (a, name, field.name)
            var = await add_variable(jnode, nid(*path), field.name, initial,
                                     ua.ObjectIds.AnalogUnitType if analog else ua.ObjectIds.BaseDataVariableType,
                                     field.description)
            if analog:
                await add_variable(var, nid(*path, "EngineeringUnits"), "EngineeringUnits",
                                   ua.Variant(engineering_units(field)), ua.ObjectIds.PropertyType,
                                   "Unit of the value", standard=True)
                rng = eu_range(field, servo)
                if rng:
                    await add_variable(var, nid(*path, "EURange"), "EURange", ua.Variant(rng),
                                       ua.ObjectIds.PropertyType, "Range of the value in normal operation",
                                       standard=True)
            nodes[name][field.name] = (var, vtype)
    return nodes


async def build_ua_server(arms):
    """The server with one object per arm.

    Returns the server and {arm name: {joint name: {field: (variable node, variant type)}}}.
    """
    server = Server()
    await server.init()
    server.set_endpoint(ENDPOINT)
    server.set_server_name("MonkeArm OPC UA server")
    idx = await server.register_namespace(NAMESPACE_URI)
    nodes = {arm.name: await add_arm(server.nodes.objects, idx, arm) for arm in arms}
    return server, nodes


async def publish(nodes, state, timestamp):
    """Write the values read at timestamp, with status Good."""
    for joint, values in state.items():
        for field, value in values.items():
            var, vtype = nodes[joint][field]
            await var.write_value(ua.DataValue(ua.Variant(value, vtype), SourceTimestamp=timestamp))


async def mark_failed(nodes, fields):
    """Give the fields of every joint the status BadCommunicationError. The server drops the
    value (OPC UA Part 4, 7.7.1); the SourceTimestamp keeps the time of the last good read."""
    for joint in nodes.values():
        for field in fields:
            var, _ = joint[field]
            last = await var.read_data_value()
            await var.write_value(ua.DataValue(ua.Variant(), StatusCode=COMM_ERROR,
                                               SourceTimestamp=last.SourceTimestamp))


async def serve_arm(arm, nodes, run):
    """Publish the arm's settings, then read its live values over and over and publish them.

    nodes is the arm's part of build_ua_server()'s result. run(f, *args) calls f on the
    arm's own thread.
    """
    loop = asyncio.get_running_loop()
    # settings come from the cache, read when the arm started
    await publish(nodes, arm.read(CONFIG_FIELDS), datetime.now(timezone.utc))
    period, last_slow, failed = 1.0 / UPDATE_HZ, None, False
    while True:
        started = loop.time()
        slow = failed or last_slow is None or started - last_slow >= SLOW_PERIOD
        fields = FAST_FIELDS + SLOW_FIELDS if slow else FAST_FIELDS
        try:
            state = await run(arm.read, fields)
        except Exception:
            if not failed:
                log.exception("%s: reading the servos failed; its live values are marked bad until a read works",
                              arm.name)
                await mark_failed(nodes, LIVE_FIELDS)
                failed = True
            await asyncio.sleep(RETRY_PERIOD)
            continue
        if failed:
            log.info("%s: reading the servos works again", arm.name)
            failed = False
        if slow:
            last_slow = started
        await publish(nodes, state, datetime.now(timezone.utc))
        await asyncio.sleep(max(0.0, period - (loop.time() - started)))


async def main(port=PORT, virtual=VIRTUAL_ARMS):
    """Serve the real arm on port (None for none) and virtual arms 1 .. virtual."""
    loop = asyncio.get_running_loop()
    arms = []   # (arm, the one thread that uses it); the real arm's thread owns the serial port
    try:
        if port:
            thread = ThreadPoolExecutor(max_workers=1)
            arms.append((await loop.run_in_executor(thread, MonkeArmDriver, port), thread))
        for n in range(1, virtual + 1):
            # seed=n: virtual arm n has the same settings in every run
            arms.append((VirtualArm(virtual_mac(n), seed=n), ThreadPoolExecutor(max_workers=1)))
        server, nodes = await build_ua_server([arm for arm, _ in arms])
        async with server:
            log.info("Serving %d arms on %s: %s", len(arms), ENDPOINT,
                     ", ".join(f"{arm.name}{' (virtual)' if arm.virtual else ''}" for arm, _ in arms))
            async with asyncio.TaskGroup() as tasks:
                for arm, thread in arms:
                    tasks.create_task(serve_arm(arm, nodes[arm.name], partial(loop.run_in_executor, thread)))
    finally:
        for arm, thread in arms:
            await loop.run_in_executor(thread, arm.close)
            thread.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="OPC UA server for the real MonkeArm and virtual ones.")
    parser.add_argument("--port", default=PORT, help=f"serial port of the real arm (default {PORT})")
    parser.add_argument("--no-real", action="store_true", help="serve no real arm, only virtual ones")
    parser.add_argument("--virtual", type=int, default=VIRTUAL_ARMS, metavar="N",
                        help=f"number of virtual arms (default {VIRTUAL_ARMS})")
    args = parser.parse_args()
    if args.virtual < 0:
        parser.error("--virtual must be 0 or more")
    if args.no_real and args.virtual == 0:
        parser.error("nothing to serve: --no-real needs at least one virtual arm")
    logging.basicConfig(level=logging.DEBUG)
    asyncio.run(main(None if args.no_real else args.port, args.virtual))
