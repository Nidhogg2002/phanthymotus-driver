# S10 front depth implementation plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Start the depth card without canvas calibration fields and report missing calibration or inaccessible point clouds explicitly.

**Architecture:** Extend the existing plugin with a validated deployment record and an owned input watchdog. Keep projection and publishing unchanged except for the validated target frame.

**Tech Stack:** Python standard library, existing ROS 2 runtime and pytest.

**Spec:** docs/superpowers/specs/2026-11-14-s10-front-depth-auto-design.md

## Global Constraints

- No guessed extrinsics, no map-cloud fallback, no synchronized RGB-D claim.
- No robot configuration changes, dependency installs, deployment, or service restart.
- Preserve source timestamps, bounded worker queues, and stale-result/lifecycle guards.

## Review Focus

- Invalid or changed records must not reuse a previous valid automatic matrix.
- Maintenance settings must not override a present deployment record.
- Stream/frame mismatches must reject output without fabricating geometry.
- Repeated start, stop, and reconfigure must not leak watchdogs or workers.
- Visible publishers without messages must differ from absent publishers.

### Task 1: Automatic calibration and canvas contract

**Files:** Modify `front_depth.py`, `tests/test_s10_front_depth.py`, and README.
**Interfaces:** Add keyword `extrinsics_path` to FrontDepthPlugin for isolated tests; deployment record fields: version, camera_id, front_camera_stream, cloud_topic, cloud_frame_id, camera_optical_frame_id, translation_unit, camera_axes, camera_optical_from_cloud.

- [x] Add failing tests for no canvas calibration inputs, missing/invalid/bound deployment records, nonidentity directed projection, read-only loading, reload on restart, and maintenance compatibility.
- [x] Run the new tests; expect failures against af3669d7.
- [x] Add the smallest loader and runtime configuration snapshot; persist startup errors and provenance.
- [x] Run the full front-depth and projection tests; expect no new failures.

### Task 2: Input health and Jetson diagnosis

**Files:** Modify `front_depth.py`, `tests/test_s10_front_depth.py`, README; add a focused diagnostic report under `deep_robotics/s10_pro/docs/`.
**Interfaces:** Expose source status, publisher count, valid-cloud age, structured error code, and owned timer cleanup.

- [x] Add failing tests for missing publisher, silent publisher, stale input, recovery, late callbacks, and cleanup.
- [x] Run new tests; expect failures before implementing the watchdog.
- [x] Implement a three-second source timeout without switching topics or touching DDS configuration.
- [x] Record paired native subscriptions and passive RTPS locator evidence; do not deploy a workaround.
- [x] Run relevant S10 tests and repository pytest; report pre-existing failures and unavailable dependencies separately.
- [x] Review the final diff, verify documented file/schema contracts and build inclusion, and leave code locally reviewable without pushing or deployment.

### Task 3: Empty-volume deployment regression

- [x] Reproduce missing bundled record and identity-query errors before changes.
- [x] Bind the existing record to the inspected NOS identity; package it in Docker.
- [x] Reuse strict NOS SSH and automatically load only an exact identity match.
- [x] Verify override precedence, unknown/error rejection, restart identity, image packaging and focused regression.
- [x] Verify bounded in-memory loader against the actual NOS, review and document the deployment boundary.
