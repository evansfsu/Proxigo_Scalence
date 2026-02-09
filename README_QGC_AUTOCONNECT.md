# QGroundControl Auto-Connect Setup

## ✅ Configuration Complete!

QGroundControl has been automatically configured to connect to PX4 SITL.

## What Was Configured

Two connections have been added to QGC with **Auto Connect** enabled:

1. **PX4 SITL UDP** - UDP port 14550 (auto-detect)
2. **PX4 SITL TCP** - TCP 127.0.0.1:5760 (manual backup)

Settings file location:
- **Windows**: `%APPDATA%\QGroundControl.org\LinkCollection.json`
- **Linux/WSL**: `~/.config/QGroundControl.org/LinkCollection.json`

## How to Use

### Option 1: Automatic (Recommended)

Just **open QGroundControl** - it will automatically connect to PX4 SITL!

The connections are configured with `autoConnect: true`, so QGC will:
1. Automatically detect and connect on UDP 14550
2. Or fall back to TCP 5760 if UDP doesn't work

### Option 2: Manual Verification

1. Open QGroundControl
2. Go to **Application Settings** (Q icon) → **Comm Links**
3. You should see:
   - **PX4 SITL UDP** - Auto Connect: ✓
   - **PX4 SITL TCP** - Auto Connect: ✓
4. Both should show "Connected" status

## Reconfiguration

If you need to reconfigure:

**Windows:**
```powershell
.\scripts\auto_connect_qgc.ps1
```

**Linux/WSL:**
```bash
./scripts/configure_qgc_autoconnect.sh UDP 14550
./scripts/configure_qgc_autoconnect.sh TCP 5760 127.0.0.1
```

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

4. **Restart QGC** - Close it completely and reopen

### Connection Not Listed in QGC

1. **Verify settings file exists:**
   - Windows: Check `%APPDATA%\QGroundControl.org\LinkCollection.json`
   - Linux: Check `~/.config/QGroundControl.org/LinkCollection.json`

2. **Re-run configuration:**
   ```powershell
   .\scripts\configure_qgc_autoconnect.ps1 -ConnectionType UDP -Port 14550
   ```

3. **Restart QGC**

## Integration with Simulation

The `start_simulation.sh` script now:
1. Starts PX4 SITL
2. Starts MAVProxy bridge
3. Sets vehicle position
4. **QGC auto-connect is pre-configured** (run once)

After running the configuration scripts once, QGC will automatically connect every time you open it!

## References

- [QGC Container Documentation](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/getting_started/container.html)
- [QGC Comm Links Settings](https://docs.qgroundcontrol.com/master/en/SettingsView/CommLinks.html)
