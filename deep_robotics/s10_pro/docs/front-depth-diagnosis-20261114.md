# S10 Pro raw cloud reachability, 2026-11-14

The Jetson DDS depth input is blocked by the current factory publisher's host-only
transport scope. A local-code SSH fallback now bypasses this boundary for the
depth card; it has been tested temporarily without deploying the business driver.
This finding is separate from missing camera extrinsics.
No robot configuration, service, TLS setting, DDS profile, or permission was
changed; no code was deployed.

## Observed boundary

- Driver Jetson: SSH `192.168.123.183`, robot interface `10.21.33.200`.
- Factory AOS: `10.21.33.103`; factory NOS: `10.21.33.106`.
- Domain 0 native subscriptions on both factory hosts receive
  `/LIDAR/POINTS`, `sensor_msgs/msg/PointCloud2`, `lidar_link`, about 10 Hz.
- The Jetson driver container sees the topic through remote subscriptions,
  but no raw-cloud writer or raw samples. It receives other remote DDS
  discovery, including `/ALIGNED_POINTS` writers.
- Paired 18-second passive RTPS captures show the raw writers on NOS loopback.
  Their endpoint GUIDs exactly match the native ROS publisher enumeration:

| Writer GUID | Advertised user-data locators |
| --- | --- |
| `010fa2e54b0d95df0000000000000503` | SHM, plus UDP `127.0.0.1:7457` |
| `010fa2e5b40e65600000000000000503` | SHM, plus UDP `127.0.0.1:7463` |

These writer announcements were captured on `lo` to `127.0.0.1`.
No raw-writer announcement appeared in the simultaneous Jetson physical-NIC
capture. Remote `/LIDAR/POINTS` **reader** announcements did reach Jetson;
reader discovery is not proof of an accessible writer. The actual ROS graph
agrees with this distinction.

The visible `dds_topic_scope.conf` documents a default of `multi`, but no
`/LIDAR/POINTS` entry is present. That file does not establish the actual
writer's scope: the wire locators do. The exact factory call/configuration
selecting host-only transport was not accessible. Do not claim this was caused
by the Jetson's wrong domain, incompatible requested reliability, or TLS.

Changing only Jetson configuration cannot make a loopback locator remote.
`cloud_relay.py` now implements a temporary in-memory NOS subscriber over the
existing authenticated SSH path. The card automatically selects it after three
seconds of silent local ROS input. Samples enter the card directly; no new
raw-cloud publisher is added to the ROS graph.

## Coordinate and camera evidence

Loaded `rsLidar` configuration is
`/var/opt/robot/conf/rslidar/config.yaml`, HWI model `CD1`, with
`send_separately: false`, `publish.frame_id: lidar_link`. Startup logs confirm
CD1 installation transforms and two RSAIRY lidar inputs. The source has already
been transformed and merged; do not reapply the front-lidar mount transform.
Layout is little-endian, 26 bytes/point: XYZ FLOAT32 offsets 0/4/8, intensity
FLOAT32 offset 12, ring UINT16 offset 16, timestamp FLOAT64 offset 18.

Front camera binding is AOS `/dev/usb_cam_head0` → `usb_camera_3` →
`rtsp://10.21.33.103:8554/video4`. Loaded logs show raw 1280×720, rotation 0,
crop `[128,72,1024,576]`, H.264 1024×576/30 fps. Existing driver K/D were
measured on that cropped RTSP stream; do not subtract the crop a second time.

The operator has now confirmed that the camera faces mechanical front and the
image is upright, with no relative pitch, yaw or roll. Under mechanical axes
X front/Y left/Z up, optical coordinates therefore use
`(X_optical,Y_optical,Z_optical)=(-Y_body,-Z_body,X_body)` for rotation.
This is a basis conversion, not an identity optical rotation. The operator also
confirmed coincident `lidar_link`/manual-body origins and identical mechanical
axes. Combined with the manual camera position, this determines:

```text
p_optical = [ 0 -1  0  0       ] p_lidar_link
            [ 0  0 -1  0.04673 ]
            [ 1  0  0 -0.27065 ]
            [ 0  0  0  1       ]
```

The record is saved in
`resource/calibration/s10_pro_cd1_inspected_20261114.json`, with translation in
metres and provenance distinguishing the manual position from operator
confirmations. Tests map the optical centre to zero, body-front into optical
+Z, body-right into image-right and body-up into image-up. This is derived
mounting geometry, not an image/cloud reprojection calibration. It is scoped
to this inspected robot and is not installed as a model-wide image default.
Future deployment must provision the record at the supported data path; this
task did not write it onto the robot.

[Official hardware manual §1.9](https://support.deeprobotics.cn/docs/?lang=zh-CN#/259676014),
checked again on this date, lists front wide camera `(270.65,0,46.73) mm` but
still has `—` in the rotation cell. Its front-lidar nominal position also
differs from the loaded CD1 lidar configuration. The mounting table alone
therefore does not supply the full chain; the operator's frame/orientation
confirmations are explicit additional premises for the generated record.

No camera optical TF or CameraInfo source was found. `/DEPTH_IMAGE` was
registered but yielded no samples in complete observation windows. Spatial
calibration loading and sparse projection do not establish exposure pairing;
the output remains unsynchronized with RTSP RGB.

## Reproduction artifacts

Local inspection artifacts are in
`/Users/4paradigm/Desktop/shixi/diagnostics/s10-depth-development-20261114/`:

- `evidence/01-jetson-passive-discovery.txt`: physical-NIC discovery summary.
- `evidence/02-jetson-active-subscription.txt`: Jetson subscription result.
- `evidence/03-nos-discovery-interface.txt`: paired loopback/physical capture.
- `evidence/04-nos-native-confirmation.txt`: valid native raw-cloud samples.
- `evidence/07-raw-writer-locators.json`: two matching raw writer locators.
- `evidence/06-official-installation-excerpt.html`: current installation table.

The captures retain only sensor discovery metadata, not full point buffers or
credentials. The production change adds status/error reporting, a validated
read-only deployment calibration interface and an SSH fallback, without
changing factory scope.

## SSH fallback verification

New local artifacts are in
`/Users/4paradigm/Desktop/shixi/diagnostics/s10-cloud-relay-20261114/`.
`evidence/live-transport-final.json` records the final bounded run through the
actual Jetson container and existing NOS credentials: 15 frames, 5.14 Hz,
increasing source stamps, `lidar_link`, 26 bytes per point, and clean shutdown.
The module ran from memory on both hosts; no source file was installed and no
service was restarted. The probe checks exact buffer length and hashes only
the first point buffer instead of saving or printing points. It also asserts
that the NOS node has no publishers, services or clients and that its ROS
context has file-logging initialization disabled.

Early transport probes exposed cold-start timeouts and a ROS-default side
effect: ordinary ROS initialization may create diagnostic log files and
parameter events. Final code uses a separate startup budget, disables logging
initialization and suppresses the constructor's parameter-event publisher.
Existing remote diagnostic logs were not deleted or changed. No factory
configuration was modified during any probe. The final run discarded one
delayed startup sample (`CLOUD_RELAY_STALE`), then received 15 fresh samples
without a disconnect/retry error.

The current deployed container still has the legacy explicit-matrix prompt,
no `cloud_relay.py`, and no deployment calibration JSON. This explains the
reported startup screenshot; the running application has not received these
local changes. Future deployment needs both the updated image and the already
generated, robot-specific calibration record at the supported data path.
Canvas users will not need to enter a topic, frame or matrix. This task does
not deploy either artifact, and does not claim image/cloud reprojection or
RTSP exposure synchronization.

Final local regression: **293 passed, 9 skipped** across relay, front-depth,
projection, undistortion, control/audio, registration and camera-info tests.
`focused-regression.log` records the command result. Repository-wide pytest
still stops with the previously observed 80 collection errors from unavailable
optional SDKs, Python 3.9 incompatibilities and duplicate module names; see
`repository-pytest.log`. No dependency was installed. Independent review
findings were fixed and rechecked with no remaining actionable findings.

## Follow-up: new loader running, deployment calibration absent

A subsequent startup screenshot reports `CALIBRATION_MISSING`. Read-only
inspection now confirms the current container includes `cloud_relay.py` but
still has no `/opt/phanthy-motus/data/s10_pro/camera_front_extrinsics.json`.
The initial Dockerfile omitted `resource/calibration/`, and the loader had no
normal-image path to the already generated record. Pushing code alone did not
provision that external file. This is a calibration delivery defect, before
any point-cloud subscription or hole filling can begin.

The fix packages the existing record as image data and binds it to SHA-256 of
the inspected NOS `/etc/machine-id`. A default start with an empty data volume
queries that identity over existing strict SSH, without ROS or file writes,
and selects exactly one matching bundled record. Deployment files and complete
maintenance settings retain precedence. An explicit missing override, invalid
file, unknown or duplicate identity, or failed SSH query never substitutes
another robot's matrix. Existing translation, rotation and stream binding
checks remain unchanged. Identity is rechecked on each fresh start.

This identifies the NOS installation, not immutable physical hardware;
replacement/reimaging requires checking the binding and calibration again.
No IP address or model-wide mounting transform is used as identity.

New artifacts are in
`/Users/4paradigm/Desktop/shixi/diagnostics/s10-depth-bundled-calibration-20261114/`.
`live-loader.json` confirms the real NOS identity matches the generated record
and the unchanged matrix loads from `bundled:` while the deployment file
remains absent. The temporary probe executes the new loader and record from
memory; it does not deploy a source file, start a business service, or publish
ROS data. Regression tests stage the calibration using the actual Docker
`COPY` source and reproduce startup with an empty volume. This verifies source
packaging and loading, not a rebuilt/deployed image. Focused regression:
**304 passed, 9 skipped**. Independent review reports no actionable findings.
Repository-wide pytest retains the known 80 collection errors; logs record
the unavailable optional SDKs, Python 3.9 incompatibilities and module-name
collisions. No dependencies or robot configuration were changed.
