# Single-Container Simulation (PX4 + Gazebo + ROS2 + MAVROS)

One Docker container runs **PX4 SITL**, **Gazebo Harmonic**, **MAVROS**, and optionally your ROS2 nodes (VIO, satellite matching, state fusion). There is no inter-container networking, so all communication is over `localhost` inside the container.

## Why use it

- **No communication issues** between PX4, Gazebo, and ROS2: everything is in one network namespace.
- **Simpler than multi-container**: one `docker compose up`, no `depends_on` or port mapping between services.
- **QGC**: MAVProxy listens inside the container on UDP 14550 and TCP 5760; Docker port mapping lets QGroundControl on the host connect to 127.0.0.1 (prefer TCP 5760, or UDP 14550).

## Prerequisites

1. Build the base PX4+Gazebo image (if not already):
   ```bash
   docker build -t proxigo/px4-gazebo:harmonic -f docker/px4_sitl/Dockerfile.gazebo .
   ```
2. From the repo root, build the all-in-one image:
   ```bash
   docker compose -f docker/simulation/docker-compose.all-in-one.yml build
   ```

## Run

From repo root:

```bash
docker compose -f docker/simulation/docker-compose.all-in-one.yml up
```

- **First run**: PX4 will build (5–10 minutes), then Gazebo and MAVROS start. Subsequent runs start much faster.
- **QGC**: Start the simulation first, then in QGroundControl add a link and connect:
  - **Recommended**: TCP — Host **127.0.0.1**, Port **5760**, **Connect**.
  - **Optional**: UDP — Connect to **127.0.0.1:14550**.

### With ROS2 nodes (VIO, satellite, fusion)

```bash
RUN_ROS2_NODES=1 docker compose -f docker/simulation/docker-compose.all-in-one.yml up
```

The entrypoint will build your workspace (if needed) and launch `vio_bridge`, `satellite_matching`, and `state_fusion` after MAVROS.

### Headless (no Gazebo GUI)

By default `HEADLESS=1` is set in the compose file. To run Gazebo with a GUI, set `HEADLESS=0` and ensure the host has an X server (e.g. VcXsrv on Windows with DISPLAY).

## What runs in the container

| Order | Component        | Purpose                          |
|-------|------------------|----------------------------------|
| 1     | PX4 SITL + Gazebo| `make px4_sitl gz_advanced_plane`|
| 2     | MAVProxy         | Listen on 14550 (UDP), 5760 (TCP) for QGC via port mapping |
| 3     | MAVROS           | ROS2 ↔ PX4 (`px4.launch` or `px4.launch.py`, fcu_url:=udp://:14540@localhost:14557) |
| 4     | (optional) VIO/satellite/fusion | If `RUN_ROS2_NODES=1` and `/ros2_ws/src` mounted |

## Ports

- **14550/udp** – QGC: MAVProxy listens in container; use QGC **Connect** to 127.0.0.1:14550.
- **5760/tcp** – QGC (recommended): MAVProxy listens in container; use QGC **Connect** to 127.0.0.1:5760.
- **18570/udp** – PX4 GCS (exposed for direct link if needed).

## QGC connection troubleshooting

- **Try TCP 5760 first**: In QGC go to **Application Settings → Comm Links → Add** → Type **TCP**, Host **127.0.0.1**, Port **5760**, enable **Connect**. Start the simulation, then connect that link (or enable Auto-Connect for it). This path does not depend on UDP or firewall.
- **If using UDP**: Add a UDP link and **Connect** to **127.0.0.1:14550** (do not use "Listen" on 14550). Start the simulation before connecting.
- **Verification**: Confirm MAVProxy is running with `docker exec sitl_all_in_one pgrep -a mavproxy`. Check that the host can reach TCP 5760 (e.g. `Test-NetConnection -ComputerName 127.0.0.1 -Port 5760` on Windows).

## Files

- `docker/px4_sitl/Dockerfile.all-in-one` – image with PX4, Gazebo, ROS2 Humble, MAVROS, MAVProxy.
- `docker/px4_sitl/entrypoint_all_in_one.sh` – starts PX4, then MAVProxy, then MAVROS, then optional nodes.
- `docker/simulation/docker-compose.all-in-one.yml` – single service, build, ports, volumes.
