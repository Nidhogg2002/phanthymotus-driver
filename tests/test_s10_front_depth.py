"""Front LiDAR depth card contracts and lifecycle without ROS or a robot."""
from __future__ import annotations

import importlib
import json
import os
import signal
import struct
import sys
import threading
import time
import zlib
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "deep_robotics" / "s10_pro"
IDENTITY = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def module():
    return importlib.import_module("deep_robotics.s10_pro.front_depth")


class FakeWorker:
    def __init__(self, on_result, on_error):
        self.on_result, self.on_error = on_result, on_error
        self.jobs = []
        self.alive = False

    def start(self):
        self.alive = True

    def is_alive(self):
        return self.alive

    def submit(self, job):
        self.jobs.append(job)
        return True

    def complete(self, index=-1):
        from deep_robotics.s10_pro.depth_projection import project_depth
        job = self.jobs[index]
        packet, stats = project_depth(job["cloud"], job["K"], job["transform"],
                                      min_depth_m=job["min_depth_m"],
                                      max_depth_m=job["max_depth_m"])
        self.on_result(job, packet, stats)

    def close(self):
        self.alive = False


@pytest.fixture
def rig(tmp_path, monkeypatch):
    depth = module()
    from deep_robotics.s10_pro.undistortion import CameraUndistortion
    controller = CameraUndistortion(("camera_front",), config_path=tmp_path / "rgb.json",
                                   map_dir=tmp_path / "maps")
    publications, subscriptions, destroyed = [], [], []

    class CompressedImage:
        def __init__(self):
            self.header = SimpleNamespace(frame_id="", stamp=SimpleNamespace(sec=0, nanosec=0))
            self.format, self.data = "", b""

    sensor = ModuleType("sensor_msgs.msg")
    sensor.CompressedImage = CompressedImage
    sensor.PointCloud2 = type("PointCloud2", (), {})
    groups = ModuleType("rclpy.callback_groups")
    groups.MutuallyExclusiveCallbackGroup = type("MutuallyExclusiveCallbackGroup", (), {})
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor)
    monkeypatch.setitem(sys.modules, "rclpy.callback_groups", groups)
    monkeypatch.setattr(depth, "DepthWorker", FakeWorker)

    def subscribe(message_type, topic, callback, qos, **kwargs):
        handle = SimpleNamespace(topic=topic, callback=callback, qos=qos)
        subscriptions.append(handle)
        return handle

    publisher = SimpleNamespace(publish=publications.append)
    nodes = SimpleNamespace(
        robot=SimpleNamespace(create_subscription=subscribe, destroy_subscription=destroyed.append),
        bridge=SimpleNamespace(create_publisher=lambda *args, **kw: publisher),
        camera_undistortion=controller, _sensor_qos=object())
    plugin = depth.FrontDepthPlugin({}, "robot", nodes, config_path=tmp_path / "depth.json")
    return SimpleNamespace(plugin=plugin, nodes=nodes, controller=controller,
                           publications=publications, subscriptions=subscriptions,
                           destroyed=destroyed, path=tmp_path / "depth.json")


def configure(rig, **values):
    result = rig.plugin.dispatch("config", {
        "cloud_topic": "/raw/front", "cloud_frame_id": "front_lidar",
        "camera_optical_from_cloud": IDENTITY, **values})
    assert result["state"] == "configured", result
    return result


def cloud(sec=10, nanosec=123, frame="front_lidar"):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
        width=1, height=1, is_bigendian=False, point_step=12, row_step=12,
        fields=[SimpleNamespace(name=name, offset=i * 4, datatype=7, count=1)
                for i, name in enumerate(("x", "y", "z"))],
        data=struct.pack("<fff", 0, 0, 2))


def test_catalogue_and_runtime_advertise_exactly_one_new_card(monkeypatch, tmp_path):
    depth = module()
    monkeypatch.syspath_prepend(str(DRIVER))
    monkeypatch.setenv("S10_FRONT_DEPTH_CONFIG", str(tmp_path / "depth.json"))
    monkeypatch.setenv("S10_CAMERA_UNDISTORT_CONFIG", str(tmp_path / "rgb.json"))
    device = importlib.import_module("device")
    # No interface enumeration, network service, or native controller is needed
    # to inspect card definitions. Spatial construction has unrelated host I/O.
    monkeypatch.setattr(device, "S10ControlledSpatialPlugin", lambda *a: SimpleNamespace(
        get_tool=lambda: {"name": "controlled_spatial"}))
    config = yaml.safe_load((DRIVER / "config.yaml").read_text())
    plugins = device.build_plugins(config)
    cards = [card for p in plugins for card in
             (p.get_tools() if hasattr(p, "get_tools") else [p.get_tool()])]
    assert sum(c["name"] == "camera_front__depth" for c in cards) == 1
    manifest = yaml.safe_load((DRIVER / "driver.yaml").read_text())
    assert sum(c["name"] == "camera_front__depth" for c in manifest["cards"]) == 1
    assert any(isinstance(p, depth.FrontDepthPlugin) for p in plugins)


def test_card_wire_contract_geometry_and_time_provenance(rig):
    card = rig.plugin.get_tool()
    assert card["name"] == "camera_front__depth" and card["type"] == "sensor"
    assert card["topic_out"] == [{"topic": "/robot/camera_front/depth", "format": "image/depth-zlib"}]
    assert not card.get("topic_in")
    assert set(card["inputSchema"]["x-action-params"]) == {"start", "stop", "info", "config"}
    info = card["camera_info"][0]
    assert (info["width"], info["height"]) == (640, 480)
    cal, _ = rig.controller.calibration("camera_front")
    assert info["K"][4] == pytest.approx(cal["K"][4] * 4 / 3)
    assert info["K"][5] == pytest.approx((cal["K"][5] + .5) * 4 / 3 - .5)
    assert info["D"] == [0.] * 5 and info["distortion_model"] == "pinhole"
    vendor = info["vendor"]
    assert vendor["depth_scale_m"] == .001 and vendor["invalid_depth_value"] == 0
    assert vendor["depth_quantity"] == "optical_z" and vendor["sparse"] is True
    assert vendor["depth_aligned_to_rgb"] is False
    assert vendor["timestamp_source"] == "lidar_header_unverified"
    assert vendor["temporal_synchronization"] is False
    assert vendor["rgb_depth_capture_skew_ms"] is None


def test_missing_extrinsic_or_frame_refuses_start_without_io(rig):
    assert rig.plugin.dispatch("start", {})["state"] == "error"
    assert not rig.subscriptions
    rig.plugin.dispatch("config", {"camera_optical_from_cloud": IDENTITY})
    result = rig.plugin.dispatch("start", {})
    assert result["state"] == "error" and "cloud_frame_id" in result["message"]
    assert not rig.subscriptions


def test_config_is_atomic_persistent_and_rejects_invalid_values(rig):
    configure(rig)
    before = rig.path.read_bytes()
    for invalid in ({"rate_hz": True}, {"rate_hz": 10 ** 400}, {"max_depth_m": 100},
                    {"cloud_topic": "relative"}, {"unexpected": 1},
                    {"min_depth_m": 60}, {"camera_optical_from_cloud": [[1]]}):
        assert rig.plugin.dispatch("config", invalid)["state"] == "error"
        assert rig.path.read_bytes() == before
    loaded = module().FrontDepthPlugin({}, "robot", rig.nodes, config_path=rig.path)
    assert loaded.dispatch("info", {})["config"]["cloud_frame_id"] == "front_lidar"
    assert loaded.dispatch("info", {})["config"]["camera_optical_from_cloud"] == IDENTITY


def test_persist_failure_does_not_apply_new_config(rig, monkeypatch):
    configure(rig)
    rig.plugin.dispatch("start", {})
    worker = rig.plugin._worker
    old = rig.plugin.dispatch("info", {})["config"]
    monkeypatch.setattr(module().os, "replace", lambda *a: (_ for _ in ()).throw(OSError("disk full")))
    result = rig.plugin.dispatch("config", {"cloud_frame_id": "different"})
    assert result["state"] == "error" and "disk full" in result["message"]
    assert rig.plugin.dispatch("info", {})["config"] == old
    assert rig.plugin._worker is worker and worker.is_alive()
    assert not rig.destroyed


def test_canvas_instance_transport_does_not_become_a_depth_setting(rig):
    from common.vendor_runtime import DriverBundle
    bundle = DriverBundle([rig.plugin])
    result = bundle.dispatch("camera_front__depth", {
        "action": "config", "instance_id": "canvas-depth-1",
        "cloud_topic": "/raw/front", "cloud_frame_id": "front_lidar",
        "camera_optical_from_cloud": json.dumps(IDENTITY),
    })
    assert result["state"] == "configured", result
    assert "instance_id" not in result["config"]
    assert "instance_id" not in json.loads(rig.path.read_text())["config"]
    assert bundle.dispatch("camera_front__depth", {
        "action": "start", "instance_id": "canvas-depth-1"})["state"] == "running"


def test_start_is_lazy_and_idempotent_preserves_source_stamp_and_metric_payload(rig):
    configure(rig)
    rig.plugin.start()  # Bundle startup only advertises the card.
    assert not rig.subscriptions
    assert rig.plugin.dispatch("start", {})["state"] == "running"
    assert rig.plugin.dispatch("start", {})["state"] == "running"
    assert len(rig.subscriptions) == 1
    handle = rig.subscriptions[0]
    assert handle.topic == "/raw/front" and handle.qos is rig.nodes._sensor_qos
    handle.callback(cloud())
    worker = rig.plugin._worker
    assert len(worker.jobs) == 1
    worker.complete()
    output = rig.publications[-1]
    assert output.format == "16UC1; compressedDepth zlib"
    assert output.header.frame_id == "s10_pro_camera_front_optical"
    assert (output.header.stamp.sec, output.header.stamp.nanosec) == (10, 123)
    raw = zlib.decompress(output.data)
    assert len(raw) == 640 * 480 * 2
    values = struct.unpack("<307200H", raw)
    assert values.count(2000) == 1 and values.count(0) == 307199
    assert rig.plugin.dispatch("info", {})["stats"]["published"] == 1


@pytest.mark.parametrize("bad", [cloud(frame="map"), cloud(sec=0, nanosec=0),
                                  cloud(nanosec=1000000000), cloud(sec=True)])
def test_unexpected_frame_and_missing_or_malformed_stamp_are_rejected(rig, bad):
    configure(rig)
    rig.plugin.dispatch("start", {})
    rig.subscriptions[-1].callback(bad)
    assert not rig.plugin._worker.jobs
    assert rig.plugin.dispatch("info", {})["stats"]["dropped"] == 1


def test_stop_and_config_discard_in_flight_results_and_do_not_close_shared_nodes(rig):
    configure(rig)
    rig.plugin.dispatch("start", {})
    old_handle, worker = rig.subscriptions[-1], rig.plugin._worker
    old_handle.callback(cloud())
    assert rig.plugin.dispatch("stop", {})["state"] == "idle"
    worker.complete()
    assert not rig.publications and rig.destroyed == [old_handle]
    rig.plugin.dispatch("start", {})
    old_handle.callback(cloud(sec=11))  # A queued callback from the old session.
    assert not rig.plugin._worker.jobs
    rig.subscriptions[-1].callback(cloud(sec=12))
    worker = rig.plugin._worker
    assert rig.plugin.dispatch("config", {"rate_hz": 4})["state"] == "configured"
    worker.complete()
    assert not rig.publications
    assert rig.plugin.dispatch("info", {})["state"] == "idle"


def test_camera_geometry_change_drops_old_result_and_blocks_raw_view(rig):
    configure(rig)
    rig.plugin.dispatch("start", {})
    rig.subscriptions[-1].callback(cloud())
    worker = rig.plugin._worker
    rig.controller.configure("camera_front", {"undistort": False})
    worker.complete()
    assert not rig.publications
    assert rig.plugin.dispatch("info", {})["state"] == "error"
    rig.plugin.dispatch("stop", {})
    assert rig.plugin.dispatch("start", {})["state"] == "error"


def test_dead_worker_is_reported_and_start_can_recover(rig):
    configure(rig)
    rig.plugin.dispatch("start", {})
    old = rig.plugin._worker
    old.alive = False
    assert rig.plugin.dispatch("info", {})["state"] == "error"
    assert rig.plugin.dispatch("start", {})["state"] == "running"
    assert rig.plugin._worker is not old and rig.plugin._worker.is_alive()


def test_depth_publication_preserves_front_camera_error(rig):
    configure(rig)
    rig.plugin.dispatch("start", {})
    generation = rig.controller.generation("camera_front")
    rig.controller.record_error("camera_front", generation, "RTSP unavailable")
    rig.subscriptions[-1].callback(cloud())
    rig.plugin._worker.complete()
    assert len(rig.publications) == 1
    assert rig.controller.status("camera_front")["error"] == "RTSP unavailable"
    # The existing RGB publication path still acknowledges camera recovery.
    rig.controller.publish_if_current("camera_front", generation, lambda: None)
    assert rig.controller.status("camera_front")["error"] is None


def test_repeated_backward_and_stale_source_work_is_not_published(rig, monkeypatch):
    configure(rig)
    rig.plugin.dispatch("start", {})
    callback = rig.subscriptions[-1].callback
    callback(cloud())
    callback(cloud())
    callback(cloud(sec=9))
    assert len(rig.plugin._worker.jobs) == 1
    assert rig.plugin.dispatch("info", {})["stats"]["dropped"] == 2
    future = time.monotonic() + 2
    monkeypatch.setattr(module().time, "monotonic", lambda: future)
    rig.plugin._worker.complete()
    assert not rig.publications
    assert rig.plugin.dispatch("info", {})["last_error"] == "depth result is stale"


def test_failed_subscription_start_closes_owned_worker(rig, monkeypatch):
    configure(rig)
    workers = []

    def make_worker(*args):
        worker = FakeWorker(*args)
        workers.append(worker)
        return worker

    def fail_subscription(*args, **kwargs):
        raise RuntimeError("subscription failed")

    monkeypatch.setattr(module(), "DepthWorker", make_worker)
    rig.nodes.robot.create_subscription = fail_subscription
    result = rig.plugin.dispatch("start", {})
    assert result["state"] == "error" and "subscription failed" in result["message"]
    assert len(workers) == 1 and not workers[0].is_alive()
    assert rig.plugin.dispatch("info", {})["state"] == "idle"


def test_real_worker_reports_bad_layout_then_projects_and_closes():
    depth = module()
    received, failures = [], []
    ready, failed = threading.Event(), threading.Event()

    def on_result(context, packet, stats):
        received.append((context, packet, stats))
        ready.set()

    def on_error(context, reason):
        failures.append(reason)
        failed.set()

    message = cloud()
    source = {key: getattr(message, key) for key in
              ("width", "height", "point_step", "row_step", "is_bigendian", "data")}
    source["fields"] = [(f.name, f.offset, f.datatype, f.count) for f in message.fields]
    job = {"generation": 1, "camera_generation": 0, "stamp": (10, 123),
           "received_monotonic": time.monotonic(), "cloud": source,
           "K": [100, 0, 320, 0, 100, 240, 0, 0, 1], "transform": IDENTITY,
           "min_depth_m": .1, "max_depth_m": 50.}
    worker = depth.DepthWorker(on_result, on_error)
    try:
        worker.start()
        assert worker.submit({**job, "cloud": {**source, "data": b""}})
        assert failed.wait(5), "worker did not report malformed point layout"
        assert len(failures) == 1 and "truncated" in failures[0]
        assert worker.submit(job)
        assert ready.wait(5), "worker did not return depth"
        context, packet, stats = received[0]
        assert context["stamp"] == (10, 123) and "cloud" not in context
        raw = zlib.decompress(packet)
        assert len(raw) == 640 * 480 * 2
        assert struct.unpack_from("<H", raw, (240 * 640 + 320) * 2)[0] == 2000
        assert stats == {"source_points": 1, "sample_points": 1, "valid_pixels": 1}
    finally:
        worker.close()
    assert not worker.is_alive()


def _wait_for(predicate):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if predicate():
            return True
        threading.Event().wait(.01)
    return False


def _process_alive(worker):
    try:
        return worker._process.is_alive()
    except ValueError:  # Process.close() released the already-reaped child.
        return False


def _cleanup_faulted_worker(worker):
    # Keep a failing regression from leaking the intentionally damaged worker.
    if _process_alive(worker):
        worker._process.kill()
        worker._process.join(2)
    worker._closed.set()
    worker._results._writer.close()
    worker._collector.join(2)
    worker._jobs._reader.close()
    for channel in (worker._jobs, worker._results):
        channel.cancel_join_thread()
        channel.close()
        if channel._thread is not None:
            channel._thread.join(2)
        channel._writer.close()
        channel._reader.close()


def _partial_result_and_exit(jobs, results):
    # Model a child killed after the pickle frame header and before its body.
    os.write(results._writer.fileno(), struct.pack("!i", 4096) + b"partial")


def test_worker_close_reaps_suspended_child():
    worker = module().DepthWorker(lambda *a: None, lambda *a: None)
    worker.start()
    try:
        os.kill(worker._process.pid, signal.SIGSTOP)
        worker.close()
        assert not _process_alive(worker)
        assert not worker._collector.is_alive()
    finally:
        _cleanup_faulted_worker(worker)


def test_worker_close_releases_feeder_after_killed_large_transfer():
    worker = module().DepthWorker(lambda *a: None, lambda *a: None)
    worker.start()

    def child_is_receiving():
        acquired = worker._jobs._rlock.acquire(block=False)
        if acquired:
            worker._jobs._rlock.release()
        return not acquired

    try:
        assert _wait_for(child_is_receiving)
        os.kill(worker._process.pid, signal.SIGSTOP)
        assert worker.submit({"payload": b"x" * (32 * 1024 * 1024)})
        assert _wait_for(lambda: worker._jobs._reader.poll(0))
        worker._process.kill()
        worker._process.join(2)
        feeder = worker._jobs._thread
        worker.close()
        assert not feeder.is_alive()
        assert worker._jobs._reader.closed and worker._jobs._writer.closed
        assert not worker._collector.is_alive()
    finally:
        _cleanup_faulted_worker(worker)


def test_worker_close_releases_collector_after_partial_result(monkeypatch):
    monkeypatch.setattr(module(), "_project_worker", _partial_result_and_exit)
    worker = module().DepthWorker(lambda *a: None, lambda *a: None)
    worker.start()
    try:
        worker._process.join(5)
        assert worker._process.exitcode == 0
        worker.close()
        assert not worker._collector.is_alive()
        assert worker._results._reader.closed and worker._results._writer.closed
    finally:
        _cleanup_faulted_worker(worker)


def test_output_config_does_not_add_dependencies_or_drop_encoder_latency_fix():
    text = (DRIVER / "Dockerfile").read_text()
    assert "front_depth.py" in text and "depth_projection.py" in text
    requirements = (DRIVER / "requirements.txt").read_text()
    assert requirements.strip() == "PyYAML==6.0.2"
    from deep_robotics.s10_pro.media_workers import _rtsp_command
    cmd = _rtsp_command("rtsp://camera")
    assert cmd[cmd.index("-threads") + 1] == "1"
