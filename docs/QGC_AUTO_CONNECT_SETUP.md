# QGroundControl Auto-Connect Setup

Based on [QGC Container Documentation](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/getting_started/container.html), this guide shows how to automatically configure QGC to connect to PX4 SITL.

## Quick Setup

### Windows (PowerShell)

Run the auto-connect script:
```powershell
.\scripts\auto_connect_qgc.ps1
```

This will:
1. Check if QGC is installed
2. Verify PX4 SITL is running
3. **Automatically configure QGC settings** for auto-connect
4. Launch QGroundControl

### Linux/WSL

```bash
# Configure UDP connection (recommended)
./scripts/configure_qgc_autoconnect.sh UDP 14550

# Or configure TCP connection
./scripts/configure_qgc_autoconnect.sh TCP 5760 127.0.0.1
```

## Manual Configuration

### Windows

QGC settings are stored in:
```
%APPDATA%\QGroundControl.org\QGroundControl.ini
```

Edit this file and add:
```ini
[CommLinks]
PX4_SITL_UDP=UDP:14550
PX4_SITL_UDP_AutoConnect=true
PX4_SITL_UDP_HighLatency=false

PX4_SITL_TCP=TCP:127.0.0.1:5760
PX4_SITL_TCP_AutoConnect=true
PX4_SITL_TCP_HighLatency=false
```

### Linux/WSL

QGC settings are stored in:
```
~/.config/QGroundControl.org/QGroundControl.ini
```

Same format as Windows.

## Connection Types

### UDP (Recommended)
- **Port**: 14550
- **Auto-detect**: QGC automatically listens on this port
- **Best for**: Local connections, auto-discovery

### TCP (Backup)
- **Host**: 127.0.0.1
- **Port**: 5760
- **Best for**: Manual connections, when UDP doesn't work

## Verification

After configuration:

1. **Open QGroundControl**
2. **Check connection status**:
   - Top-left Q icon should show "Connected" (not "Disconnected")
   - Vehicle icon should appear on map
   - Status messages should show vehicle data

3. **If not connected**:
   - Go to **Application Settings** → **Comm Links**
   - Verify your connection is listed with **Auto Connect: ✓**
   - Click **Connect** manually if needed

## Troubleshooting

### QGC Still Shows "Disconnected"

1. **Check PX4 is running:**
   ```bash
   docker ps | grep px4_sitl
   ```

2. **Check MAVProxy is forwarding:**
   ```bash
   docker exec px4_sitl pgrep -af mavproxy
   ```

3. **Test connection:**
   ```bash
   docker exec px4_sitl python3 /scripts/test_qgc_connection.py
   ```

4. **Verify ports are listening:**
   ```bash
   netstat -an | grep -E "(14550|5760)"
   ```

### Settings Not Applied

1. **Close QGC completely** (not just minimized)
2. **Re-run configuration script**
3. **Reopen QGC**

### Multiple Connections

If you have multiple connections configured:
- QGC will try to connect to all auto-connect links
- Only one needs to succeed
- You can disable others in **Comm Links** settings

## Integration with Simulation Startup

The `start_simulation.sh` script now automatically:
1. Starts PX4 SITL
2. Starts MAVProxy bridge
3. Sets vehicle position
4. **Optionally configures QGC auto-connect** (if script is available)

To enable full auto-connect:
```bash
./scripts/start_simulation.sh --restart
# Then run:
.\scripts\auto_connect_qgc.ps1  # Windows
# Or:
./scripts/configure_qgc_autoconnect.sh UDP 14550  # Linux/WSL
```

## References

- [QGC Container Documentation](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/getting_started/container.html)
- [QGC Comm Links Documentation](https://docs.qgroundcontrol.com/master/en/SettingsView/CommLinks.html)
