"""Robot power sequencing using the installed web protocol and passive DSM feedback.

No SDK connection or motor command is made on import. Protocol provenance is in
integrations/astribot_web/README.md. A command acknowledgement is never completion.
"""
import base64
import asyncio
from dataclasses import dataclass
from functools import lru_cache
import hashlib
import json
from pathlib import Path
import re
import time
import urllib.error
import urllib.request


DSM_TOPIC = '/astribot/system_dsm_result'
NORMAL = {'power': 12000000, 'driver': 13000000}
PREFIX = {'power': '12', 'driver': '13'}
DESCRIPTOR_SHA256 = 'c156755c20ba4030227eab325269a47d8d23e752111736518e106096eb26356f'


class LifecycleError(RuntimeError):
    pass


@dataclass(frozen=True)
class Status:
    codes: tuple
    source_ns: int
    received: float
    seq: int = 0

    def module_codes(self, module):
        return tuple(code for code in self.codes if str(code).startswith(PREFIX[module]))

    def errors(self, prefixes=('12', '13', '16', '18')):
        return tuple(code for code in self.codes
                     if str(code).startswith(prefixes) and str(code)[2] == '2')

    def state(self, module):
        codes = self.module_codes(module)
        if not codes:
            return 'off'
        if any(str(code)[2] == '2' for code in codes):
            return 'fault'
        if NORMAL[module] in codes:
            return 'on'
        return 'unconfirmed'

    def summary(self):
        return {'power': self.state('power'), 'driver': self.state('driver'),
                'codes': list(self.codes), 'source_time_ns': self.source_ns, 'sequence': self.seq}


@lru_cache(maxsize=1)
def diagnostic_type():
    """Load the robot's exported descriptors without legacy generated-code imports."""
    from google.protobuf import descriptor_pb2, descriptor_pool, message_factory
    path = Path(__file__).resolve().parents[1] / 'integrations/astribot_web/dsm_descriptor.b64'
    raw = base64.b64decode(''.join(path.read_text().split()), validate=True)
    if hashlib.sha256(raw).hexdigest() != DESCRIPTOR_SHA256:
        raise LifecycleError('Robot diagnostic descriptor checksum mismatch')
    files = descriptor_pb2.FileDescriptorSet()
    files.ParseFromString(raw)
    pool = descriptor_pool.DescriptorPool()
    for file in files.file:
        pool.AddSerializedFile(file.SerializeToString())
    definition = pool.FindMessageTypeByName('astribot.proto.error_code.DsmResult')
    if hasattr(message_factory, 'GetMessageClass'):
        return message_factory.GetMessageClass(definition)
    return message_factory.MessageFactory(pool).GetPrototype(definition)


def decode_status(payload, received, wall_time):
    if not payload or len(payload) > 1024 * 1024:
        raise LifecycleError('Empty or oversized diagnostic payload')
    message = diagnostic_type()()
    message.ParseFromString(payload)
    stamp = message.header.time_pub
    if stamp <= 0:
        raise LifecycleError('Diagnostic message has no source timestamp')
    age = wall_time - stamp / 1e9
    if age > 3 or age < -2:
        raise LifecycleError(f'Diagnostic timestamp is stale or clocks disagree (age {age:.2f}s)')
    codes = tuple(sorted(set(message.itcs)))
    if any(len(str(code)) != 8 or str(code)[2] not in '012' for code in codes):
        raise LifecycleError(f'Unrecognized diagnostic codes: {codes}')
    return Status(codes, stamp, received, message.header.seq)


class DiagnosticMonitor:
    """Subscribe only; never register a web client or acquire SDK control."""
    def __init__(self):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from astribot_msgs.msg import AstribotSerializedProtoMsg
        diagnostic_type()  # Fail before any lifecycle command if decoding is unavailable.
        self.rclpy = rclpy
        rclpy.init(args=[])
        self.node = rclpy.create_node('astribot_robot_power_monitor')
        self.latest = None
        self.problem = None
        self.subscription = self.node.create_subscription(
            AstribotSerializedProtoMsg, DSM_TOPIC, self.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def receive(self, message):
        try:
            status = decode_status(bytes(message.data), time.monotonic(), time.time())
            if self.latest is None or status.source_ns > self.latest.source_ns:
                self.latest = status
        except Exception as exc:
            self.problem = str(exc)

    def read(self, timeout=3.0):
        previous = self.latest.source_ns if self.latest else 0
        deadline = time.monotonic() + timeout
        while self.rclpy.ok() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=min(0.1, max(0, deadline-time.monotonic())))
            if self.problem:
                raise LifecycleError(self.problem)
            if self.latest and self.latest.source_ns > previous:
                if time.monotonic() - self.latest.received > 1:
                    raise LifecycleError('Diagnostic receive loop is delayed')
                return self.latest
        raise LifecycleError('No fresh DSM diagnostics; a missing heartbeat is not confirmation of power-off')

    def control_owners(self):
        owners = []
        for name, namespace in self.node.get_node_names_and_namespaces():
            try:
                services = self.node.get_service_names_and_types_by_node(name, namespace)
                if any(service == '/astribot/control_rights' for service, _ in services):
                    owners.append(namespace.rstrip('/') + '/' + name)
            except Exception:
                continue
        return sorted(set(owners))

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


class RobotWeb:
    """Verified power/drive commands and WEB-to-PC handoff; never retry blindly."""
    def __init__(self, host='192.168.0.10', port=5141, timeout=5.0, control_port=5143):
        if not host or any(character not in '0123456789.' for character in host):
            raise ValueError('Use the robot IPv4 address')
        import ipaddress
        ipaddress.IPv4Address(host)
        if not 1 <= port <= 65535 or not 1 <= control_port <= 65535:
            raise ValueError('Invalid robot port')
        self.url = f'http://{host}:{port}/msg'
        self.control_state_url = f'http://{host}:{control_port}/ws'
        self.timeout = timeout
        # The robot LAN must not be routed through an HTTP proxy.
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def command(self, module, enabled):
        if module not in NORMAL or not isinstance(enabled, bool):
            raise ValueError('Expected power/driver and a boolean state')
        message = {'msg_id': 'robot_power' if module == 'power' else 'robot_driver',
                   'data': {'start': enabled}}
        self._post(message, module)

    def select_pc_control(self):
        self._post({'msg_id': 'set_control_source', 'data': {'source': 'PC'}}, 'WEB-to-PC handoff')

    def _post(self, message, operation):
        request = urllib.request.Request(self.url, data=json.dumps(message).encode(),
                                         headers={'Content-Type': 'application/json'}, method='POST')
        try:
            with self.opener.open(request, timeout=self.timeout) as response:
                result = json.loads(response.read(65536))
        except (OSError, ValueError, urllib.error.URLError) as exc:
            raise LifecycleError(f'{operation} acknowledgement failed; do not retry blindly: {exc}') from exc
        if not isinstance(result, dict) or result.get('suc') is not True:
            raise LifecycleError(f'{operation} command rejected: {result}')

    @staticmethod
    def parse_control_source(payload):
        messages = json.loads(payload)
        if not isinstance(messages, list):
            raise LifecycleError('Malformed control-state stream')
        for message in messages:
            if isinstance(message, dict) and message.get('msg_id') == 'control_source':
                data = message.get('data')
                source = data.get('source') if isinstance(data, dict) else None
                if source not in ('WEB', 'PC', 'VR_CONTROL', 'JOYSTICK', 'DARG', 'WEB_DRAG'):
                    raise LifecycleError(f'Unknown control source: {source!r}')
                return source
        return None

    async def _read_control_source(self):
        import aiohttp
        async with aiohttp.ClientSession(trust_env=False,
                timeout=aiohttp.ClientTimeout(total=self.timeout)) as session:
            socket = await session.ws_connect(self.control_state_url, max_msg_size=1024*1024)
            try:
                async def receive():
                    while True:
                        message = await socket.receive()
                        if message.type != aiohttp.WSMsgType.TEXT:
                            raise LifecycleError('Control-state stream closed or returned non-text data')
                        source = self.parse_control_source(message.data)
                        if source is not None:
                            return source
                return await asyncio.wait_for(receive(), timeout=self.timeout)
            finally:
                # This vendor endpoint sends updates but does not read CLOSE frames.
                # Bound the handshake explicitly so passive reads cannot hang on exit.
                try:
                    await asyncio.wait_for(socket.close(), timeout=0.25)
                except asyncio.TimeoutError:
                    pass

    def control_source(self):
        try:
            return asyncio.run(self._read_control_source())
        except LifecycleError:
            raise
        except Exception as exc:
            raise LifecycleError(f'Cannot read passive control source: {exc or type(exc).__name__}') from exc


class Sequencer:
    def __init__(self, monitor, web, home, *, timeout=60.0, stable_seconds=2.0,
                 cooldown=10.0, settle_seconds=5.0, clock=time.monotonic, report=print):
        self.monitor, self.web, self.home = monitor, web, home
        self.timeout, self.stable_seconds, self.cooldown = timeout, stable_seconds, cooldown
        self.settle_seconds = settle_seconds
        self.clock, self.report = clock, report

    def wait(self, description, predicate, *, seconds=None, guard=None):
        seconds = self.stable_seconds if seconds is None else seconds
        deadline = self.clock() + self.timeout + seconds
        stable_since, samples, last_ns, previous_state = None, 0, 0, None
        while self.clock() < deadline:
            status = self.monitor.read(timeout=min(3.0, max(0.01, deadline-self.clock())))
            if status.source_ns <= last_ns:
                raise LifecycleError('Repeated diagnostic data cannot confirm completion')
            last_ns = status.source_ns
            if guard:
                guard(status)
            if predicate(status):
                current_state = (status.state('power'), status.state('driver'))
                if current_state != previous_state:
                    stable_since, samples = None, 0
                previous_state = current_state
                stable_since = status.received if stable_since is None else stable_since
                samples += 1
                if samples >= 3 and status.received-stable_since >= seconds:
                    self.report(f'[ROBOT] Confirmed {description}: {status.summary()}')
                    return status
            else:
                stable_since, samples, previous_state = None, 0, None
        raise LifecycleError(f'Timed out waiting for {description}; no later command was sent')

    @staticmethod
    def safe_for_start(status):
        if status.errors():
            raise LifecycleError(f'Robot reports faults; resolve them before startup: {status.errors()}')
        if status.state('power') == 'off' and status.state('driver') != 'off':
            raise LifecycleError('Driver is reported active while joint power is unconfirmed')
        if 'unconfirmed' in (status.state('power'), status.state('driver')):
            raise LifecycleError(f'Robot state is unconfirmed: {status.summary()}')

    @staticmethod
    def power_stays_on(status):
        if status.state('power') != 'on':
            raise LifecycleError(f'Power is not healthy during deactivation: {status.summary()}')

    def active_for_home(self, status):
        self.safe_for_start(status)
        if status.state('power') != 'on' or status.state('driver') != 'on':
            raise LifecycleError('Power or activation was lost before home; no home command was sent')

    def control_state_for_start(self):
        source = self.web.control_source()
        owners = self.monitor.control_owners()
        if source not in ('WEB', 'PC'):
            raise LifecycleError(f'Control source is {source}; exit teleoperation/drag mode before startup')
        others = [owner for owner in owners if re.fullmatch(r'/web_astribot_[0-9]+', owner) is None]
        if others:
            raise LifecycleError(f'Another SDK controller owns the robot: {others}; release it before startup')
        return source, owners

    def prepare_home_control(self):
        source, owners = self.control_state_for_start()
        self.report(f'[ROBOT] Control source: {source}; SDK owners: {owners or "none"}.')
        if source == 'WEB':
            self.report('[ROBOT] Requesting the vendor WEB-to-PC control handoff.')
            self.web.select_pc_control()

        def released(status):
            source, owners = self.control_state_for_start()
            return source == 'PC' and not owners

        self.wait('PC control selected and web SDK control released', released, guard=self.active_for_home)

    def start(self):
        status = self.wait('initial state', lambda state: True, guard=self.safe_for_start)
        self.control_state_for_start()  # Check the passive endpoint and other owners before power commands.
        if status.state('power') == 'off':
            self.report('[ROBOT] Observing power-off for 10 seconds before allowing power-on.')
            status = self.wait('power-cycle cooldown',
                               lambda state: state.state('power') == state.state('driver') == 'off',
                               seconds=self.cooldown, guard=self.safe_for_start)
            self.report('[ROBOT] Requesting joint power ON.')
            self.web.command('power', True)
            status = self.wait('joint power ON', lambda state: state.state('power') == 'on',
                               guard=self.safe_for_start)
        if status.state('driver') == 'off':
            self.report('[ROBOT] Requesting drive activation.')
            self.web.command('driver', True)
            status = self.wait('drives ACTIVE',
                               lambda state: state.state('power') == state.state('driver') == 'on',
                               guard=self.safe_for_start)
        self.report(f'[ROBOT] Waiting {self.settle_seconds:g} additional seconds with power and drives healthy.')
        self.wait('post-activation settling', lambda state: True,
                  seconds=self.settle_seconds, guard=self.active_for_home)
        self.prepare_home_control()
        self.report('[ROBOT] Power, activation and control handoff confirmed; running the SDK home helper.')
        if self.home() != 0:
            raise LifecycleError('Home was not confirmed. No automatic power-off was attempted.')
        self.wait('powered and active after home',
                  lambda state: state.state('power') == state.state('driver') == 'on',
                  guard=self.safe_for_start)
        self.report('[ROBOT] Startup complete: powered on, active, home confirmed.')

    def stop(self):
        status = self.wait('initial state', lambda state: True)
        if status.state('power') == 'off' and status.state('driver') == 'off':
            self.report('[ROBOT] Already deactivated and powered off.')
            return
        self.power_stays_on(status)
        if status.state('driver') != 'off':
            self.report('[ROBOT] Requesting drive deactivation.')
            self.web.command('driver', False)
        # Fresh, stable inactive feedback is mandatory, even if already inactive.
        self.wait('drives INACTIVE before power-off', lambda state: state.state('driver') == 'off',
                  guard=self.power_stays_on)
        self.report('[ROBOT] Deactivation confirmed; requesting joint power OFF.')
        self.web.command('power', False)
        self.wait('joint power OFF', lambda state: state.state('power') == state.state('driver') == 'off')
        self.report('[ROBOT] Shutdown complete: deactivated and powered off.')
