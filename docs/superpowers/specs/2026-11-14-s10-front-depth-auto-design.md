# S10 front depth automatic configuration

The user wants `camera_front__depth` to start from the canvas without entering
topics, frames, or a matrix. Missing extrinsics must fail clearly; this change
includes code, tests, and read-only diagnosis, with no robot deployment or
configuration changes.

Keep the existing projection, worker, timestamp, and lifecycle contracts.
Use `/LIDAR/POINTS` as the verified default, never switch to a map/display cloud.
Hide calibration/transport fields from canvas configuration. Retain the existing
maintenance configuration API for compatibility.

Load a deployment-owned, read-only JSON calibration at start from
`/opt/phanthy-motus/data/s10_pro/camera_front_extrinsics.json`, overridable by
`S10_FRONT_CAMERA_EXTRINSICS`. Version 1 records must bind the logical front
camera, configured RTSP stream, source topic/frame, target optical frame,
metre translation, optical axes, and the directed rigid matrix. Validate the
record and actual message frame; never infer a matrix from mounting positions.
The file is a new supported deployment interface, not a discovered factory
calibration source. Do not create a sample or populated calibration on the robot.
An invalid present record must not fall back to stale/manual values. Existing
complete maintenance configurations remain usable when no record is present.

Report calibration source and distinguish validation from reprojection. Start
failures persist in status. After three seconds without valid input, distinguish
no publisher from no data; expose waiting/live/stale input status and clean up
the owned watchdog on stop/reconfigure. Do not claim synchronization.

Alternatives considered: guessed mounting transforms violate the explicit user
constraint; automatic TF lookup has no verified manufacturer target frame;
changing factory DDS scope violates this task's robot constraints. Use the
deployment calibration interface and preserve a clear blocked state instead.

Read-only paired RTPS captures confirm raw publishers advertise only SHM and
loopback on NOS; Jetson receives remote subscribers but no raw writers.
Document this boundary and a future authorized factory change or read-only
host-side forwarding option, without implementing an unrequested relay.

During implementation the user confirmed that this robot's camera faces body
+X with upright image and no relative rotation, and that lidar_link coincides
with the manual body origin and X front/Y left/Z up axes. Generate a local
deployment calibration record using the manual position and these explicit
premises; test directed projection and optical origin invariants. Scope the
record to this inspected robot, preserve provenance and unverified reprojection
status, and do not deploy it or make it a model-wide fallback.

## Delivery bug fix: missing calibration in normal images

A subsequent screenshot and read-only container check show the new loader is
running but the deployment data file is absent. The previous Dockerfile did not
include the generated per-robot record. Bundle that record as read-only image
data, bind it to the inspected NOS `/etc/machine-id` SHA-256 read over existing
strict SSH, and automatically select it only when no deployment or maintenance
calibration is provided. Do not use IP, model or mounting position as identity.
An explicit override, corrupt present file, unknown identity or failed query
must not silently use this robot's profile. Reload identity on every fresh
start; no filesystem writes or factory changes. Translation/rotation values
and unverified reprojection status remain unchanged. A universal built-in
transform is rejected; manual data-volume provisioning caused this defect.
