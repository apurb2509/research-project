# GNSS + IMU Precision Flight Monitor

A real-time desktop application for high-accuracy GNSS position logging and IMU sensor fusion, developed for ISRO weather radar calibration flight campaigns.

The application integrates a u-blox ZED-F9P GNSS receiver with an MPU-6050 inertial measurement unit, merges their data streams into timestamped epochs, applies configurable position filters, and exports the results to CSV.

---

## Hardware Requirements

| Component | Interface | Notes |
|-----------|-----------|-------|
| u-blox ZED-F9P | UART / USB | Supports RTK Fix, RTK Float, DGPS |
| MPU-6050 IMU | I2C (address 0x68) | Sampled at 100 Hz |

The IMU is optional. If the `smbus2` library is not installed or the sensor is absent, the application runs in GNSS-only mode.

---

## Software Dependencies

```
Python >= 3.10
PyQt5
pyserial
pynmea2
smbus2          # optional, required only for IMU
```

Install all dependencies:

```bash
pip install PyQt5 pyserial pynmea2 smbus2
```

---

## Running the Application

```bash
python U-center.py
```

On Windows:

```bash
py U-center.py
```

---

## Features

### GNSS Reception

- Connects to any serial port at configurable baud rates (9600 to 115200).
- Parses the following NMEA sentence types: GGA, RMC, GSA, GSV, GLL, VTG, ZDA.
- Detects fix quality: No Fix, GPS 3D, DGPS, RTK Fix, RTK Float.
- Tracks visible and used satellites with per-satellite C/N0 readings.
- Raw NMEA sentences are written to a timestamped log file under `~/.gnss_monitor_logs/`.

### UBX Protocol Configuration

- Sends UBX CFG-MSG commands to configure NMEA output rates per sentence type directly to the ZED-F9P over the active serial port.
- Configures measurement rate (CFG-RATE) based on the highest enabled sentence rate.
- All changes are applied in a single session without disconnecting.

### IMU Streaming (MPU-6050)

- Runs a dedicated QThread polling the sensor at 100 Hz via I2C.
- Reads raw accelerometer and gyroscope registers (14 bytes per cycle).
- Computes:
  - Linear acceleration in m/s^2 (X, Y, Z axes)
  - Angular rate in deg/s (X, Y, Z axes)
  - Roll and pitch angles via accelerometer trigonometry
  - Vibration index: deviation of total acceleration magnitude from 1 g

### Epoch Merger

- Each GGA fix anchors an epoch. GSA, GSV, RMC sentences received within a configurable timeout window are associated with that epoch.
- IMU state is snapshot-attached to each epoch at the moment the GGA fix arrives.
- Merged epoch rows include: UTC time, position, fix quality, HDOP, and all selected IMU metrics.

### Position Filter Pipelines

Up to four independent pipelines can be enabled simultaneously. Each pipeline independently processes every incoming fix and plots its output on the sky plot.

| Method | Description |
|--------|-------------|
| Raw | No filtering. Plots the direct GNSS fix. |
| EMA | Exponential Moving Average with configurable alpha. |
| SMA | Simple Moving Average over a configurable window. |
| Hampel | Sliding-window outlier identifier using median absolute deviation. Replaces outliers with the local median. |
| KF-ENU | 2-D Kalman Filter in East-North-Up coordinates. Tunable process noise and DGPS weighting factor. |

Each pipeline has an independently configurable colour and can be toggled on or off without losing its accumulated output.

### Sky Plot

- North-up scatter plot showing trajectory deviation from a computed reference centre.
- Reference centre is recomputed as the mean of all valid fixes within a configurable rolling time window.
- Concentric range rings with distance labels, compass rose, and a configurable selection circle.
- Per-pipeline point counts (inside / total) displayed in a legend.
- Optional ground truth marker (crosshair) for absolute accuracy assessment.
- Zoom control (20% to 500%) and selection radius control available from the toolbar.

### Signal Strength Display

- Bar chart of C/N0 (carrier-to-noise density) for all tracked satellites.
- GPS satellites (PRN 1-32) distinguished from other constellations by colour.
- Reference grid lines at 10, 20, 30, 40, 50 dBHz.

### Live Status Panel

Displays the following fields, refreshed every 200 ms:

- Latitude, Longitude (7 decimal places)
- Altitude (MSL)
- Fix mode and HDOP
- Satellite count: used / in view
- IMU roll, pitch, acceleration vector, vibration index

### Geodetic Computations

The following computations run internally on every fix:

- Haversine great-circle distance and bearing between two geodetic points
- Geodetic to ECEF conversion and back (WGS-84 ellipsoid)
- Tropospheric delay estimation using the Saastamoinen model

### Data Export

**Export Log (toolbar button)**

Exports the GPS + IMU snapshot log to CSV with a fixed, comprehensive schema:

```
timestamp_utc, gps_latitude_deg, gps_longitude_deg, gps_altitude_m,
gps_fix_mode, gps_hdop, gps_pdop, gps_vdop,
imu_ax_mps2, imu_ay_mps2, imu_az_mps2,
imu_gx_deg_s, imu_gy_deg_s, imu_gz_deg_s,
imu_roll_deg, imu_pitch_deg, imu_vibration_g, imu_ts_utc
```

**File > Save Merged CSV**

Exports the epoch-merged dataset. Columns are determined by whichever NMEA fields the user has marked for CSV inclusion via the Fields dialog. Suitable for post-processing with custom field selections.

**Streaming CSV (Start Stream / Stop Stream)**

Opens a file at session start and appends every completed epoch row in real time as data arrives. Intended for long-duration flights where data must be saved continuously rather than at session end.

### Theme

Light and dark themes are available. The toolbar button toggles between them on click, or the dropdown arrow provides explicit selection. Both themes apply to all widgets including the custom painted sky plot and signal bar chart.

---

## Application Layout

```
Left column
  Sky Plot (trajectory deviation, North-up)
  Position / Quality / IMU Live Status panel
  GPS + IMU Snapshot Log table (last 100 records)

Right column
  Filter Pipeline configuration (4 pipelines)
  UBX Message Frequency and Fields configuration
  Satellite Signal Strength chart
```

---

## Log Files

Raw NMEA sentences are automatically written to:

```
~/.gnss_monitor_logs/nmea_raw_<YYYYMMDD_HHMMSS>.log
```

Debug log:

```
~/.gnss_monitor_logs/gnss_monitor_debug.log
```

---

## Project Structure

```
research-project/
  U-center.py          Main application (single file)
  icons/
    checkmark.svg      Checkbox indicator (Feather Icons style, MIT)
    sun.svg            Light mode toolbar icon
    moon.svg           Dark mode toolbar icon
  .gitignore
  README.md
```

---

## Notes

- The application is designed for single-antenna GNSS (ZED-F9P in standalone or RTK rover mode). Multi-antenna heading is not supported.
- IMU integration is additive: GNSS fixes are the primary position source. The IMU data is logged and fused into epoch records but does not currently feed back into the position filter.
- All timestamps use UTC. The `datetime.timezone.utc` standard is applied consistently throughout to avoid naive-datetime ambiguity.
