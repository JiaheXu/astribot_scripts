# Robot power protocol verified on the installed S1 controller

Inspected on 2026-11-18 at `192.168.0.10`, from the receiving PC. This is the
installed vendor protocol, not a claim of a supported, stable public API.

## Command transport

The UI sends `POST /msg` on TCP port 5141 with JSON:

| Operation | Request |
| --- | --- |
| Joint power on | `{"msg_id":"robot_power","data":{"start":true}}` |
| Activate drives | `{"msg_id":"robot_driver","data":{"start":true}}` |
| Deactivate drives | `{"msg_id":"robot_driver","data":{"start":false}}` |
| Joint power off | `{"msg_id":"robot_power","data":{"start":false}}` |
| Release WEB control for PC SDK | `{"msg_id":"set_control_source","data":{"source":"PC"}}` |

The server starts background ROS service calls, returning `suc: true` without
waiting for their result. Therefore HTTP 200 or `suc: true` is only an
acknowledgement. There is no automatic command retry after a timeout.

The controller's source, inspected without modification:

- `/opt/astribot_ros/meta/astribot_deployment/astribot_web_server/astribot_web_server/com/com_web_server.py`
  (`on_handle_msg`, `on_robot_power`, `on_robot_driver`, `on_request_msg`).
- `.../node/node_robot_status.py`: power maps to `/hardware_request_service`
  with `power_on` / `power_off`; drive state maps to `/raw_request_service`
  with `active` / `deactive`.
- Served `/astribot_sdk.js`, SHA-256:
  `2c7cda8bec8ecbb934788fc8662e101690f039c36c3e38473a8aebc0c217f828`.

The UI requires 10 seconds after joint power-off before the next power-on.
The automation conservatively observes an off state for 10 seconds before
powering on, even when no previous local shutdown time is known.

## Web-controller ownership after activation

The failed startup on 2026-11-18 at 16:42 China time was rejected because an
SDK controller already owned `/astribot/control_rights`. The receiving PC's
`python3_1180008_1794991329342.log` records that refusal. The robot's
`web_control_client_redirect/astribot_control_client_20261118_095830.log`
shows `web_astribot_2230` reacquiring control at 16:42:08 and retaining WEB
control throughout the home attempt. This is not a home-trajectory timeout.

Verified in the installed `node/node_robot_control.py`:

- `_handle_error_code` acquires WEB SDK control when normal drive code
  `13000000` appears. A fixed delay alone does not release this ownership.
- `on_set_control_source` accepts `data.source = "PC"`; the WEB-to-PC transition
  calls `release_robot_control`, which invokes the vendor SDK release routine.
  The 5141 `/msg` handler proxies this request to the control process.
- `GET /ws` on port 5143 streams JSON arrays including
  `{"msg_id":"control_source","data":{"source":"WEB"}}` or `"PC"`.
  This endpoint only adds a streaming subscriber; it does not register an
  operator or alter control permission. Its close handshake needs a bounded
  client timeout because the vendor handler sends updates without reading
  client frames. The implementation bounds close to 0.25 seconds.

Startup checks the passive stream before power commands, waits five additional
healthy seconds after drive confirmation, requests only a WEB-to-PC transition,
then confirms PC mode and no SDK owner for at least three samples/two seconds.
Other SDK owners and VR/joystick/drag modes halt startup. The home helper's
independent unowned-control check remains in force. No automatic transition
back to WEB or forced takeover occurs after home or a failed step.

## Passive completion feedback

Do **not** poll `request_server_msg` merely to monitor: it registers a new UI
client and changes the active client's permission. Its timeout also triggers
web-disconnect handling. The automation does not use that endpoint.

Instead, it subscribes to `/astribot/system_dsm_result`:
`astribot_msgs/msg/AstribotSerializedProtoMsg`, published by
`astribot_error_code_internal`. The payload is the vendor's protobuf `DsmResult`.
Its `header.time_pub` is nanoseconds, `itcs` contains diagnostic codes, and
`module_status` is currently empty. The installed UI derives module state from
the same codes: prefix `12` is power, prefix `13` is robot drives, the third
digit is 0=normal, 1=warning, 2=fault. Normal codes are 12000000 and 13000000.
An absent module prefix in continuing, fresh DSM messages is the UI's off state;
this is software feedback, not an independent motor-voltage measurement.

The script rejects stale/future timestamps, duplicate source timestamps, unknown
code formats, faults during startup and loss of diagnostics. It requires at
least three new samples and two stable seconds for a completed transition.
Deactivation must be confirmed while power is still healthy before power-off.
Missing diagnostics never count as inactive/off. Normal controller error
handling, emergency stops, brakes and motion-mode limits are not changed.

`dsm_descriptor.b64` is a base64 FileDescriptorSet exported from the controller's
own `ast_header_pb2`, `ast_error_code_pb2`, and `ast_dsm_result_pb2` modules in
`/opt/astribot_ros/software/astribot_common_proto/lib/pyproto`. It includes the
original schema dependencies and enum definitions, loaded into a private
descriptor pool to avoid legacy protobuf generated-code/version conflicts.
Decoded descriptor SHA-256:
`c156755c20ba4030227eab325269a47d8d23e752111736518e106096eb26356f`.

Re-inspect the installed protocol/schema before using this integration after a
controller firmware or web-server upgrade. A failed phase stops the sequence;
it does not automatically release brakes, clear errors or force power off.
