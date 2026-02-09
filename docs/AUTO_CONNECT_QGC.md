# Automatic QGroundControl Connection

## Quick Start

Run this PowerShell script to automatically configure and launch QGC:

```powershell
.\scripts\auto_connect_qgc.ps1
```

This script will:
1. Check if QGC is installed
2. Verify PX4 SITL is running
3. Check if connection ports are listening
4. Launch QGroundControl with connection instructions

## Manual Connection Steps

If the auto-connect script doesn't work, follow these steps:

### Method 1: UDP Auto-Detect (Recommended)

1. **Open QGroundControl**
2. **QGC should auto-detect** the connection on UDP port 14550
3. If not, go to **Application Settings** (Q icon) → **Comm Links**
4. **Add new connection:**
   - Type: **UDP**
   - Listening Port: **14550**
   - Auto Connect: ✓
   - Click **OK** and **Connect**

### Method 2: TCP Manual Connection

1. **Application Settings** → **Comm Links** → **Add**
2. Configure:
   - Type: **TCP**
   - Server Address: **127.0.0.1**
   - Server Port: **5760**
   - Auto Connect: ✓
3. Click **OK** and **Connect**

## Troubleshooting

### "Disconnected" in QGC

1. **Check PX4 is running:**
   ```bash
   docker ps | grep px4_sitl
   ```

2. **Check MAVProxy is running:**
   ```bash
   docker exec px4_sitl pgrep -af mavproxy
   ```

3. **Test connection:**
   ```bash
   docker exec px4_sitl python3 /scripts/test_qgc_connection.py
   ```

4. **Check ports are listening:**
   ```bash
   netstat -an | grep -E "(14550|5760)"
   ```

### PX4 Not Running

If PX4 crashed during startup:
```bash
./scripts/start_simulation.sh --restart
```

Wait 2-3 minutes for PX4 to fully initialize, then try connecting QGC again.

### MAVProxy Not Running

Restart MAVProxy:
```bash
docker exec px4_sitl bash -c "pkill -f mavproxy; mavproxy.py --master=udp:127.0.0.1:14540 --out=udp:0.0.0.0:14550 --out=tcpin:0.0.0.0:5760 --daemon"
```

## Connection Verification

Once connected, you should see:
- ✅ Q icon in top-left shows "Connected" (not "Disconnected")
- ✅ Vehicle icon appears on the map
- ✅ Status messages in QGC showing vehicle data

If you see "Disconnected" or no vehicle icon:
1. Wait 10-15 seconds for EKF to initialize
2. Run: `docker exec px4_sitl python3 /scripts/set_vehicle_position.py`
3. Refresh QGC map view

## Automatic Connection on Startup

To make QGC auto-connect every time:

1. In QGC: **Application Settings** → **Comm Links**
2. Set your connection to **Auto Connect: ✓**
3. Save the connection
4. QGC will automatically connect when it starts

## Port Reference

- **UDP 14550**: QGC default auto-detect port
- **TCP 5760**: QGC manual connection port
- **UDP 14540**: PX4 offboard port (internal, forwarded by MAVProxy)
