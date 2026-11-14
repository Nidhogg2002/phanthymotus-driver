# Read-only S10 cloud relay

Continue the approved local development, without robot deployment, persistent
remote files, factory configuration changes, or new canvas settings.

Use the verified NOS host and existing map-management SSH credentials/known-host
pinning. A temporary Python process sources the existing Jazzy setup, subscribes
only to the calibrated PointCloud2 topic in domain 0, and exits when SSH stdin
closes. It must not publish into the factory domain or invoke services.

Keep direct ROS input when healthy. After the existing three-second input
timeout, start the relay once per card session, provided no frame/calibration
error is present. Select one input transport for that session; late direct-ROS
callbacks are ignored once SSH is selected. Stop/reconfigure closes SSH and
its remote subscriber, including failed connections and reconnect waits.

Use bounded binary framing: version magic, metadata length, data length, JSON
metadata and unchanged point bytes. Preserve stamp, frame, fields, layout and
endianness. Validate lengths before allocation and metadata before use. No CDR
cross-distro deserialization, coordinate conversion, or lossy point packing.

Use request/response credits: at most one frame is in flight; each request
selects the latest source sample. A response's local round-trip time plus
remote sample age is a conservative freshness bound independent of clock
synchronization. Drop responses exceeding one second, and reconnect when the
whole response stalls beyond three seconds. Do not claim exposure pairing.

Retain only the newest sample remotely while ROS spins independently of SSH
backpressure. Local reconnect is bounded/delayed and errors are generic and
credential-free. Reuse existing sshpass/key authentication, strict host checking,
and server-alive bounds; never disable SSH verification or install dependencies.

Alternatives: cross-host factory DDS changes violate the user constraint;
republishing raw clouds into domain 0 could feed factory subscribers and create
duplicates. Feed the selected relay directly into the card's existing validation
and projection path instead.
