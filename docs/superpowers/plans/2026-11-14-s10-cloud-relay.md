# S10 read-only cloud relay implementation plan

> **For agentic workers:** Use superpowers:executing-plans in the current isolated worktree.

**Goal:** Automatically receive factory-local raw clouds over existing SSH when Jetson DDS cannot reach them.
**Architecture:** One standard-library relay module contains the framed protocol, temporary remote subscriber and local SSH lifecycle. The existing depth watchdog selects it only after direct ROS input times out.
**Tech Stack:** Python standard library, existing NOS ROS Jazzy and existing SSH client credentials.
**Spec:** docs/superpowers/specs/2026-11-14-s10-cloud-relay-design.md

## Global Constraints

- No deployment, remote file writes, factory changes, dependency installation or new user settings.
- Preserve exact point bytes, original frame/stamp and all field/layout metadata.
- No ROS republishing into the factory domain, service calls or motion commands.

## Review Focus

- Partial/truncated/oversized frames cannot desynchronize or grow unbounded buffers.
- Slow or broken SSH cannot queue old frames or block card stop.
- ROS/SSH callbacks cannot both submit frames after a transport switch.
- Missing calibration must never start SSH; late callbacks cannot revive stopped sessions.
- Secrets stay in process environment, never logs/arguments/errors.

### Task 1: Bounded relay protocol and process lifecycle

Files: create `deep_robotics/s10_pro/cloud_relay.py` and `tests/test_s10_cloud_relay.py`.
Interfaces: `CloudRelay(config, topic, rate_hz, on_message, on_error)`, start/is_alive/close/status; read/write framed packets; executable remote reader.

- [x] Write failing codec, remote-read-only, request/response, stale/stall and process cleanup tests.
- [x] Implement protocol and newest-only remote subscription; implement SSH launch, reconnect and process-group cleanup.
- [x] Run relay tests and verify no original data or source timestamp is altered.

### Task 2: Automatic card integration and verification

Files: modify `front_depth.py`, Dockerfile, README and front-depth tests.
Interfaces: existing source watchdog chooses SSH after three seconds; expose input transport/relay status without new config fields.

- [x] Add failing tests for automatic fallback, healthy local ROS, absent calibration, single selected transport, errors/recovery and stop/reconfigure cleanup.
- [x] Implement guarded integration and include the module in the image's source-copy list.
- [x] Run focused S10 regression and repository pytest, retaining known baseline limitations.
- [x] Perform a bounded temporary SSH data-read test if the robot is reachable; no source/config is copied to disk or services restarted.
- [x] Obtain independent read-only review and fix important findings with regression tests.
