# QGroundControl connection to all-in-one simulation

This describes how to connect QGroundControl (QGC) to the single-container PX4 SITL + Gazebo simulation.

## Recommended: TCP 5760

1. Start the simulation first:
   ```bash
   docker compose -f docker/simulation/docker-compose.all-in-one.yml up
   ```
2. In QGC: **Application Settings → Comm Links → Add**
   - Type: **TCP**
   - Host: **127.0.0.1**
   - Port: **5760**
   - Enable **Connect** (QGC connects to the simulation).
3. Enable **Auto Connect** for that link if desired, or click **Connect** after the simulation is up.

This path does not depend on `host.docker.internal`, UDP, or Windows firewall.

## Optional: UDP 14550

After the entrypoint change (MAVProxy uses `udpin:0.0.0.0:14550`), you can also use UDP:

- In QGC add a **UDP** link and **Connect** to **127.0.0.1:14550** (do not use "Listen" on 14550).
- Start the simulation before connecting.

## Troubleshooting: QGC still shows "Disconnected"

1. **Try TCP 5760 first** as above. It is the most reliable.
2. **If using UDP**, use **Connect** to **127.0.0.1:14550**, not "Listen" on 14550.
3. **Verify MAVProxy**: `docker exec sitl_all_in_one pgrep -a mavproxy`
4. **Verify port on host** (Windows): `Test-NetConnection -ComputerName 127.0.0.1 -Port 5760`
5. In QGC enable **LinkManagerLog** (Settings → Console → Set Logging) to see link activity.

See also [Single-Container Simulation](docs/SINGLE_CONTAINER_SIMULATION.md) for full setup and ports.
