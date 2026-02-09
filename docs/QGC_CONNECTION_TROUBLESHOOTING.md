# QGroundControl Connection Troubleshooting

## Problem: QGC Shows "Disconnected" or Wrong Location

If QGroundControl shows:
- "Disconnected" even when connected to TCP
- Your computer's location (e.g., San Francisco) instead of Death Valley
- Vehicle icon not appearing

## Quick Fixes

### 1. Verify Connection is Working

Test if PX4 is sending heartbeats:
```bash
docker exec px4_sitl python3 /scripts/test_qgc_connection.py
```

If this fails, the connection isn't working. Check:
- MAVProxy is running: `docker exec px4_sitl pgrep -af mavproxy`
- Ports are listening: `netstat -tuln | grep -E '(14550|5760)'`

### 2. Set Vehicle Position (Again)

QGC might be showing your computer's location. Force the vehicle position:
```bash
docker exec px4_sitl python3 /scripts/set_vehicle_position.py
```

### 3. QGC Connection Settings

In QGroundControl:

1. **Remove old connections:**
   - Settings → Comm Links
   - Delete any existing connections to 127.0.0.1

2. **Add new connection:**
   - Settings → Comm Links → Add
   - Type: TCP
   - Server Address: `127.0.0.1`
   - Server Port: `5760`
   - Auto Connect: ✓
   - Save

3. **Or use Auto-Connect (UDP):**
   - QGC should auto-detect UDP on port 14550
   - If not, add UDP connection:
     - Type: UDP
     - Listening Port: `14550`
     - Auto Connect: ✓

### 4. Disable QGC Location Services

QGC might be using your computer's GPS location instead of the vehicle's:

1. **Settings → General:**
   - Uncheck "Use system location" if available
   - Or disable location services in Windows

2. **Settings → Map:**
   - Make sure "Follow Vehicle" is enabled
   - This centers the map on the vehicle, not your location

### 5. Force Vehicle Position Continuously

If QGC keeps showing the wrong location, run this in a separate terminal to continuously send GPS position:

```bash
docker exec px4_sitl python3 /scripts/force_vehicle_position.py
```

Keep this running while QGC is connected. It sends GPS_INPUT messages every 200ms to keep the vehicle position updated.

### 6. Check Vehicle Status in QGC

Once connected:
1. Look at the top-left corner - should show vehicle status (not "Disconnected")
2. In Plan view, right-click on the map at Death Valley
3. Select "Set Home Here" to force QGC to use Death Valley
4. The vehicle icon should appear at Death Valley

### 7. Verify PX4 is Sending Position

Check if PX4 is actually sending GPS data:
```bash
docker logs px4_sitl | grep -i "gps\|position\|ekf" | tail -20
```

## Common Issues

### "Disconnected" but Port is Open
- **Cause:** MAVProxy not forwarding messages correctly
- **Fix:** Restart MAVProxy:
  ```bash
  docker exec px4_sitl pkill mavproxy
  docker exec -d px4_sitl bash -c "mavproxy.py --master=udp:127.0.0.1:14540 --out=udp:0.0.0.0:14550 --out=tcpin:0.0.0.0:5760 --daemon"
  ```

### Shows San Francisco (Your Location)
- **Cause:** QGC using computer's GPS instead of vehicle position
- **Fix:** 
  1. Disable location services in Windows
  2. Run `set_vehicle_position.py` script
  3. In QGC, right-click map → "Set Home Here" at Death Valley

### Vehicle Icon Not Appearing
- **Cause:** Vehicle position not set or EKF not initialized
- **Fix:**
  1. Run `set_vehicle_position.py`
  2. Wait 10-15 seconds for EKF to initialize
  3. Refresh QGC map view

## Verification Checklist

After following the steps above:

- [ ] QGC shows "Connected" (not "Disconnected")
- [ ] Vehicle icon appears on map
- [ ] Vehicle is at Death Valley (36.2329, -116.8276), not San Francisco
- [ ] Home position is set to Death Valley
- [ ] Mission waypoints are visible and close together

If all checked, you're ready to fly!
