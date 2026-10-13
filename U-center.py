#!/usr/bin/env python3
"""
GNSS + IMU Precision Flight Monitor (ISRO Weather Radar Calibration)
- Integrates u-blox ZED-F9P (UART/USB) and MPU-6050 (I2C @ 100 Hz).
- Deterministic monotonic timestamping with high-rate IMU logging.
- GGA-anchored epoch merger with attached dynamic IMU metrics (accel, gyro, roll, pitch, vibration).
- Real-time PyQt5 dashboard with skyplot, signal bars, kinematic displays, and streaming CSV export.
"""

import sys
import os
import math
import csv
import json
import time
import struct
import logging
from datetime import datetime, timedelta, timezone
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QComboBox, QPushButton, QAction, QStatusBar, QGroupBox, QGridLayout,
    QToolBar, QFileDialog, QTableWidget, QTableWidgetItem, QDialog,
    QSpinBox, QCheckBox, QMessageBox, QDoubleSpinBox, QColorDialog, QTabWidget,
    QToolButton, QMenu
)
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal, QPointF, QSize
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont, QIcon, QPixmap

import serial
import serial.tools.list_ports
import pynmea2

try:
    import smbus2
    HAS_SMBUS = True
except ImportError:
    HAS_SMBUS = False

# ------------- Config & Logging -------------
LOG_DIR = os.path.join(os.path.expanduser("~"), ".gnss_monitor_logs")
os.makedirs(LOG_DIR, exist_ok=True)
RAW_LOG_PATH = os.path.join(LOG_DIR, f"nmea_raw_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(os.path.join(LOG_DIR, "gnss_monitor_debug.log"))
    ]
)
logger = logging.getLogger("gnss_monitor")
MAX_CNO = 60.0

# ---------- Open-Source SVG Icons (Feather Icons style, MIT license) ----------
ICONS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "icons")
os.makedirs(ICONS_DIR, exist_ok=True)

_ICON_SVG = {
    # White checkmark on transparent – used for checked checkbox indicator
    'checkmark.svg': (
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 16 16">'
        '<polyline points="2,9 6,13 14,3" '
        'style="fill:none;stroke:#ffffff;stroke-width:2.5;'
        'stroke-linecap:round;stroke-linejoin:round"/>'
        '</svg>'
    ),
    # Amber sun – shown on toolbar when in dark mode ("switch to light")
    'sun.svg': (
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none"'
        ' stroke="#f59e0b" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round">'
        '<circle cx="12" cy="12" r="5"/>'
        '<line x1="12" y1="1" x2="12" y2="3"/>'
        '<line x1="12" y1="21" x2="12" y2="23"/>'
        '<line x1="4.22" y1="4.22" x2="5.64" y2="5.64"/>'
        '<line x1="18.36" y1="18.36" x2="19.78" y2="19.78"/>'
        '<line x1="1" y1="12" x2="3" y2="12"/>'
        '<line x1="21" y1="12" x2="23" y2="12"/>'
        '<line x1="4.22" y1="19.78" x2="5.64" y2="18.36"/>'
        '<line x1="18.36" y1="5.64" x2="19.78" y2="4.22"/>'
        '</svg>'
    ),
    # Indigo crescent moon – shown on toolbar when in light mode ("switch to dark")
    'moon.svg': (
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none"'
        ' stroke="#4f46e5" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round">'
        '<path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z"/>'
        '</svg>'
    ),
}
for _ifname, _isvg in _ICON_SVG.items():
    with open(os.path.join(ICONS_DIR, _ifname), 'w', encoding='utf-8') as _f:
        _f.write(_isvg)

# Forward-slash path for Qt stylesheet url()
_cm_url  = os.path.join(ICONS_DIR, 'checkmark.svg').replace('\\', '/')
_sun_url = os.path.join(ICONS_DIR, 'sun.svg').replace('\\', '/')
_moon_url = os.path.join(ICONS_DIR, 'moon.svg').replace('\\', '/')

# Per-theme checkbox-indicator overrides (appended to the main QSS in apply_theme)
CHECKBOX_LIGHT_CSS = f"""
QCheckBox::indicator {{
    width: 16px; height: 16px;
    border: 2px solid #64748b; border-radius: 4px;
    background: #ffffff;
}}
QCheckBox::indicator:hover {{
    border-color: #3b82f6; background: #eff6ff;
}}
QCheckBox::indicator:checked {{
    background: #2563eb; border-color: #2563eb;
    image: url({_cm_url});
}}
QCheckBox::indicator:checked:hover {{
    background: #1d4ed8; border-color: #1d4ed8;
    image: url({_cm_url});
}}
"""

CHECKBOX_DARK_CSS = f"""
QCheckBox::indicator {{
    width: 16px; height: 16px;
    border: 2px solid #64748b; border-radius: 4px;
    background: #1e293b;
}}
QCheckBox::indicator:hover {{
    border-color: #60a5fa; background: #1e3a5f;
}}
QCheckBox::indicator:checked {{
    background: #0ea5e9; border-color: #0ea5e9;
    image: url({_cm_url});
}}
QCheckBox::indicator:checked:hover {{
    background: #0284c7; border-color: #0284c7;
    image: url({_cm_url});
}}
"""

# -------- Theme Stylesheets --------
LIGHT_STYLE = """
QMainWindow { background: #f8fafc; }
QWidget { background: #f8fafc; color: #1e293b; font-family: 'Segoe UI', Arial, sans-serif; font-size: 10pt; }
QGroupBox { border: 1.5px solid #cbd5e1; border-radius: 10px; margin-top: 18px; padding: 10px 8px 8px 8px; font-weight: 700; color: #1d4ed8; background: #ffffff; }
QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 6px; background: #ffffff; }
QLabel { color: #334155; background: transparent; }
QPushButton { border: 1.5px solid #94a3b8; border-radius: 7px; background: #e2e8f0; color: #1e293b; padding: 5px 12px; font-weight: 600; min-height: 24px; }
QPushButton:hover { background: #bfdbfe; border-color: #3b82f6; color: #1d4ed8; }
QPushButton:pressed { background: #2563eb; color: #ffffff; border-color: #2563eb; }
QComboBox { border: 1.5px solid #94a3b8; border-radius: 7px; background: #ffffff; color: #1e293b; padding: 4px 8px; min-height: 24px; }
QComboBox QAbstractItemView { background: #ffffff; color: #1e293b; selection-background-color: #bfdbfe; border: 1px solid #cbd5e1; }
QSpinBox, QDoubleSpinBox { border: 1.5px solid #94a3b8; border-radius: 7px; background: #ffffff; color: #1e293b; padding: 4px 8px; min-height: 24px; }
QCheckBox { color: #334155; spacing: 6px; background: transparent; }
QCheckBox::indicator { width: 16px; height: 16px; border: 1.5px solid #94a3b8; border-radius: 4px; background: #ffffff; }
QCheckBox::indicator:checked { background: #2563eb; border-color: #2563eb; }
QTableWidget { background: #ffffff; color: #1e293b; border: 1px solid #e2e8f0; gridline-color: #e2e8f0; alternate-background-color: #f1f5f9; selection-background-color: #bfdbfe; selection-color: #1e293b; }
QHeaderView::section { background: #e2e8f0; color: #1e293b; padding: 6px 8px; border: none; border-right: 1px solid #cbd5e1; border-bottom: 1px solid #cbd5e1; font-weight: 700; }
QToolBar { background: #f1f5f9; border-bottom: 1.5px solid #cbd5e1; padding: 4px 8px; spacing: 4px; }
QToolBar QLabel { color: #475569; font-weight: 600; background: transparent; }
QStatusBar { background: #e2e8f0; color: #64748b; font-size: 9pt; border-top: 1px solid #cbd5e1; }
QMenuBar { background: #f1f5f9; color: #1e293b; }
QMenuBar::item:selected { background: #bfdbfe; }
QMenu { background: #ffffff; color: #1e293b; border: 1px solid #cbd5e1; }
QMenu::item:selected { background: #bfdbfe; }
QScrollBar:vertical { background: #f1f5f9; width: 8px; border-radius: 4px; }
QScrollBar::handle:vertical { background: #94a3b8; border-radius: 4px; min-height: 20px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QDialog { background: #f8fafc; color: #1e293b; }
"""

DARK_STYLE = """
QMainWindow { background: #0f172a; }
QWidget { background: #0f172a; color: #e2e8f0; font-family: 'Segoe UI', Arial, sans-serif; font-size: 10pt; }
QGroupBox { border: 1.5px solid #334155; border-radius: 10px; margin-top: 18px; padding: 10px 8px 8px 8px; font-weight: 700; color: #7dd3fc; background: #111827; }
QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 6px; background: #111827; }
QLabel { color: #cbd5e1; background: transparent; }
QPushButton { border: 1.5px solid #475569; border-radius: 7px; background: #1e293b; color: #f8fafc; padding: 5px 12px; font-weight: 600; min-height: 24px; }
QPushButton:hover { background: #334155; border-color: #60a5fa; }
QPushButton:pressed { background: #0ea5e9; color: #ffffff; border-color: #0ea5e9; }
QComboBox { border: 1.5px solid #475569; border-radius: 7px; background: #1e293b; color: #f8fafc; padding: 4px 8px; min-height: 24px; }
QComboBox QAbstractItemView { background: #1e293b; color: #f8fafc; selection-background-color: #0ea5e9; border: 1px solid #334155; }
QSpinBox, QDoubleSpinBox { border: 1.5px solid #475569; border-radius: 7px; background: #1e293b; color: #f8fafc; padding: 4px 8px; min-height: 24px; }
QCheckBox { color: #cbd5e1; spacing: 6px; background: transparent; }
QCheckBox::indicator { width: 16px; height: 16px; border: 1.5px solid #475569; border-radius: 4px; background: #1e293b; }
QCheckBox::indicator:checked { background: #0ea5e9; border-color: #0ea5e9; }
QTableWidget { background: #0b1220; color: #e2e8f0; border: 1px solid #334155; gridline-color: #1e293b; alternate-background-color: #111827; selection-background-color: #1e40af; selection-color: #f8fafc; }
QHeaderView::section { background: #1e293b; color: #f8fafc; padding: 6px 8px; border: none; border-right: 1px solid #334155; border-bottom: 1px solid #334155; font-weight: 700; }
QToolBar { background: #111827; border-bottom: 1.5px solid #334155; padding: 4px 8px; spacing: 4px; }
QToolBar QLabel { color: #94a3b8; font-weight: 600; background: transparent; }
QStatusBar { background: #0f172a; color: #94a3b8; font-size: 9pt; border-top: 1px solid #334155; }
QMenuBar { background: #111827; color: #e2e8f0; }
QMenuBar::item:selected { background: #1e293b; }
QMenu { background: #111827; color: #e2e8f0; border: 1px solid #334155; }
QMenu::item:selected { background: #1e293b; }
QScrollBar:vertical { background: #0f172a; width: 8px; border-radius: 4px; }
QScrollBar::handle:vertical { background: #475569; border-radius: 4px; min-height: 20px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QDialog { background: #0f172a; color: #e2e8f0; }
"""

def safe_float(x):
    if x is None:
        return None
    try:
        return float(x)
    except Exception:
        return None

# ---------- UBX Builder Helpers ----------
def ubx_checksum(payload_bytes):
    ck_a = 0
    ck_b = 0
    for b in payload_bytes:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])

def make_ubx(msg_class, msg_id, payload_bytes):
    hdr = b'\xB5\x62'
    length = len(payload_bytes).to_bytes(2, 'little')
    body = bytes([msg_class, msg_id]) + payload_bytes
    ck = ubx_checksum(body)
    return hdr + bytes([msg_class, msg_id]) + length + payload_bytes + ck

NMEA_MSG_IDS = {
    'GGA': 0x00, 'GLL': 0x01, 'GSA': 0x02, 'GSV': 0x03,
    'RMC': 0x04, 'VTG': 0x05, 'ZDA': 0x08
}

def build_cfg_msg_for_nmea(nmea_name, rate_on_uart1=0):
    if nmea_name not in NMEA_MSG_IDS:
        raise ValueError("Unknown NMEA message")
    msg_id = NMEA_MSG_IDS[nmea_name]
    payload = bytes([0xF0, msg_id, rate_on_uart1, 0, 0, 0, 0, 0])
    return make_ubx(0x06, 0x01, payload)

def build_cfg_rate_ms(meas_rate_ms=100, nav_rate=1, time_ref=1):
    payload = int(meas_rate_ms).to_bytes(2, 'little') + int(nav_rate).to_bytes(2, 'little') + int(time_ref).to_bytes(2, 'little')
    return make_ubx(0x06, 0x08, payload)

# ---------- Sentence Fields Templates ----------
SENTENCE_FIELDS = {
    'GGA': [
        ('utc_time', 'UTC Time'),
        ('latitude', 'Latitude'),
        ('longitude', 'Longitude'),
        ('gps_qual', 'Fix Quality'),
        ('num_sats', 'Sat Count'),
        ('horizontal_dil', 'HDOP'),
        ('altitude', 'Altitude (MSL)'),
        ('geo_sep', 'Geoid Separation'),
        ('alt_hae', 'Altitude (HAE)'),
        ('avg_cn0', 'Avg C/N0 (dBHz)'),
        ('raw_east', 'Raw East (m)'),
        ('raw_north', 'Raw North (m)'),
        ('tropo_delay', 'Tropo Delay (m)'),
        ('imu_ax', 'IMU Ax (m/s2)'),
        ('imu_ay', 'IMU Ay (m/s2)'),
        ('imu_az', 'IMU Az (m/s2)'),
        ('imu_gx', 'IMU Gx (deg/s)'),
        ('imu_gy', 'IMU Gy (deg/s)'),
        ('imu_gz', 'IMU Gz (deg/s)'),
        ('imu_roll', 'IMU Roll (deg)'),
        ('imu_pitch', 'IMU Pitch (deg)'),
        ('imu_vib', 'Vibration Index (g)'),
        ('raw', 'Raw NMEA')
    ],
    'RMC': [
        ('utc_time', 'UTC Time'),
        ('status', 'Status'),
        ('latitude', 'Latitude'),
        ('longitude', 'Longitude'),
        ('spd_over_grnd', 'Speed (knots)'),
        ('true_course', 'True Course'),
        ('datestamp', 'Date'),
        ('raw', 'Raw NMEA')
    ],
    'GSA': [
        ('mode', 'Mode'),
        ('mode2', 'Mode2'),
        ('pdop', 'PDOP'),
        ('hdop', 'HDOP'),
        ('vdop', 'VDOP'),
        ('satellites_used', 'Satellites Used (list)'),
        ('raw', 'Raw NMEA')
    ],
    'GSV': [
        ('num_messages', 'Num Messages'),
        ('msg_num', 'Msg No'),
        ('sat_info', 'Satellite Info (prn|elev|azim|snr)'),
        ('raw', 'Raw NMEA')
    ],
    'GLL': [
        ('utc_time', 'UTC Time'),
        ('latitude', 'Latitude'),
        ('longitude', 'Longitude'),
        ('status', 'Status'),
        ('raw', 'Raw NMEA')
    ],
    'VTG': [
        ('true_track', 'True Track'),
        ('spd_kmh', 'Speed (km/h)'),
        ('spd_knots', 'Speed (knots)'),
        ('raw', 'Raw NMEA')
    ],
    'ZDA': [
        ('utc_time', 'UTC Time'),
        ('datestamp', 'Date'),
        ('local_zone_hours', 'Local TZ Hours'),
        ('local_zone_minutes', 'Local TZ Minutes'),
        ('raw', 'Raw NMEA')
    ]
}

# ---------- Geodetic Helpers ----------
EARTH_R = 6371000.0

def haversine_distance_m(lat1, lon1, lat2, lon2):
    if None in (lat1, lon1, lat2, lon2):
        return None
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return EARTH_R * c

def bearing_deg(lat1, lon1, lat2, lon2):
    if None in (lat1, lon1, lat2, lon2):
        return None
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    x = math.sin(dlambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    theta = math.atan2(x, y)
    return (math.degrees(theta) + 360.0) % 360.0

def geodetic_to_ecef(lat_deg, lon_deg, alt_m):
    a = 6378137.0
    e2 = 6.69437999014e-3
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    N = a / math.sqrt(1 - e2 * math.sin(lat)**2)
    X = (N + alt_m) * math.cos(lat) * math.cos(lon)
    Y = (N + alt_m) * math.cos(lat) * math.sin(lon)
    Z = (N * (1 - e2) + alt_m) * math.sin(lat)
    return X, Y, Z

def ecef_to_geodetic(X, Y, Z):
    a = 6378137.0
    e2 = 6.69437999014e-3
    b = a * math.sqrt(1 - e2)
    eps = e2 / (1.0 - e2)
    p = math.sqrt(X*X + Y*Y)
    th = math.atan2(a * Z, b * p)
    lon = math.atan2(Y, X)
    lat = math.atan2(Z + eps * b * math.sin(th)**3, p - e2 * a * math.cos(th)**3)
    N = a / math.sqrt(1 - e2 * math.sin(lat)**2)
    alt = p / math.cos(lat) - N
    return math.degrees(lat), math.degrees(lon), alt

def tropo_delay_saas(lat_deg, alt_m, elev_deg, pressure=1013.25):
    try:
        if elev_deg is None:
            return None
        elev = float(elev_deg)
        if elev <= 0:
            return None
        elev_rad = math.radians(elev)
        return 0.0022768 * pressure / math.sin(elev_rad)
    except Exception:
        return None

# ---------- Kalman Filter (2D ENU) ----------
class Kalman2D:
    def __init__(self, q_pos=0.1, q_vel=0.1, r_pos=1.0):
        self.x = None
        self.P = None
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.r_pos = r_pos
        self.t0 = None

    def init(self, ex, ey, t):
        self.x = [ex, ey, 0.0, 0.0]
        self.P = [[100.0, 0, 0, 0], [0, 100.0, 0, 0], [0, 0, 25.0, 0], [0, 0, 0, 25.0]]
        self.t0 = t

    def predict(self, t):
        if self.x is None:
            self.t0 = t
            return
        dt = max(1e-6, t - (self.t0 or t))
        F = [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]]
        q_p, q_v = self.q_pos, self.q_vel
        Q = [[q_p*dt, 0, 0, 0], [0, q_p*dt, 0, 0], [0, 0, q_v*dt, 0], [0, 0, 0, q_v*dt]]
        
        temp = [[sum(F[i][k] * self.P[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
        self.P = [[sum(temp[i][k] * F[j][k] for k in range(4)) + Q[i][j] for j in range(4)] for i in range(4)]
        self.x = [sum(F[i][k] * self.x[k] for k in range(4)) for i in range(4)]
        self.t0 = t

    def update(self, ex_meas, ey_meas, Rpos):
        if self.x is None:
            self.init(ex_meas, ey_meas, time.time())
            return
        S00 = self.P[0][0] + Rpos
        S01 = self.P[0][1]
        S10 = self.P[1][0]
        S11 = self.P[1][1] + Rpos
        detS = S00 * S11 - S01 * S10
        if detS == 0:
            return
        invS00, invS01 = S11 / detS, -S01 / detS
        invS10, invS11 = -S10 / detS, S00 / detS
        
        K = [[self.P[i][0] * invS00 + self.P[i][1] * invS10,
              self.P[i][0] * invS01 + self.P[i][1] * invS11] for i in range(4)]
        
        y0 = ex_meas - self.x[0]
        y1 = ey_meas - self.x[1]
        for i in range(4):
            self.x[i] += K[i][0] * y0 + K[i][1] * y1
            
        KH = [[K[i][0] if j == 0 else (K[i][1] if j == 1 else 0.0) for j in range(4)] for i in range(4)]
        Iminus = [[(1.0 if i == j else 0.0) - KH[i][j] for j in range(4)] for i in range(4)]
        self.P = [[sum(Iminus[i][k] * self.P[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

    def state(self):
        if self.x is None:
            return None
        return self.x[0], self.x[1], self.x[2], self.x[3]

# ---------- Data Models ----------
class DataFix:
    def __init__(self, ts_iso, lat, lon, alt=None, is_dgps=False,
                 hdop=None, visible_sats=None, used_sats=None, avg_used_snr=None,
                 x=None, y=None, z=None, vdop=None, alt_hae=None):
        self.ts = ts_iso
        self.lat = safe_float(lat)
        self.lon = safe_float(lon)
        self.alt = safe_float(alt)
        self.is_dgps = bool(is_dgps)
        self.hdop = safe_float(hdop)
        self.visible_sats = visible_sats or []
        self.used_sats = used_sats or []
        self.avg_used_snr = safe_float(avg_used_snr) if avg_used_snr is not None else None
        self.x = safe_float(x)
        self.y = safe_float(y)
        self.z = safe_float(z)
        self.vdop = safe_float(vdop) if vdop is not None else None
        self.alt_hae = safe_float(alt_hae) if alt_hae is not None else None
        self.raw_east = None
        self.raw_north = None
        self.tropo_delay = None
        self.imu_metrics = {}

class Satellite:
    def __init__(self, prn, snr=None, elev=None, azim=None):
        self.prn = int(prn)
        self.snr = safe_float(snr)
        self.elev = safe_float(elev)
        self.azim = safe_float(azim)
        self.used = False
        self.last_update = datetime.now(timezone.utc)

# ---------- Dedicated High-Rate IMU Thread ----------
class IMUReaderThread(QThread):
    sample_received = pyqtSignal(dict)
    error = pyqtSignal(str)

    def __init__(self, i2c_bus=1, address=0x68):
        super().__init__()
        self.bus_num = i2c_bus
        self.addr = address
        self.running = False
        self.bus = None
        self.accel_scale = 16384.0  # +/- 2g
        self.gravity = 9.80665
        self.gyro_scale = 131.0     # +/- 250 deg/s

    def run(self):
        if not HAS_SMBUS:
            self.error.emit("smbus2 is not installed. IMU disabled.")
            return

        try:
            self.bus = smbus2.SMBus(self.bus_num)
            # Wake up MPU-6050
            self.bus.write_byte_data(self.addr, 0x6B, 0x00)
            time.sleep(0.05)
            # Set sample rate divider (100 Hz output)
            self.bus.write_byte_data(self.addr, 0x19, 0x09)
            # DLPF config: ~44Hz low-pass filter
            self.bus.write_byte_data(self.addr, 0x1A, 0x03)
            self.bus.write_byte_data(self.addr, 0x1B, 0x00)
            self.bus.write_byte_data(self.addr, 0x1C, 0x00)
        except Exception as e:
            self.error.emit(f"IMU Init Error on address {hex(self.addr)}: {e}")
            return

        self.running = True
        logger.info("IMU Reader Thread running at 100 Hz...")

        while self.running:
            try:
                # Read 14 bytes starting at ACCEL_XOUT_H (0x3B)
                data = self.bus.read_i2c_block_data(self.addr, 0x3B, 14)
                raw = struct.unpack('>7h', bytes(data))

                ax = (raw[0] / self.accel_scale) * self.gravity
                ay = (raw[1] / self.accel_scale) * self.gravity
                az = (raw[2] / self.accel_scale) * self.gravity
                gx = raw[4] / self.gyro_scale
                gy = raw[5] / self.gyro_scale
                gz = raw[6] / self.gyro_scale

                # Roll and Pitch estimation via accelerometer trigonometry
                roll = math.degrees(math.atan2(ay, az)) if (ay != 0 or az != 0) else 0.0
                pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2))) if (ay**2 + az**2) > 0 else 0.0

                # Drone vibration magnitude index (deviation from 1g)
                total_accel_g = math.sqrt(ax**2 + ay**2 + az**2) / self.gravity
                vib_index = abs(total_accel_g - 1.0)

                sample = {
                    "ts": datetime.now(timezone.utc).isoformat(),
                    "monotonic": time.monotonic(),
                    "ax": ax, "ay": ay, "az": az,
                    "gx": gx, "gy": gy, "gz": gz,
                    "roll": roll, "pitch": pitch,
                    "vib": vib_index
                }
                self.sample_received.emit(sample)
                time.sleep(0.01)  # ~100 Hz polling loop
            except Exception as e:
                self.error.emit(f"IMU Bus Error: {e}")
                time.sleep(0.1)

        if self.bus:
            try:
                self.bus.close()
            except Exception:
                pass

    def stop(self):
        self.running = False

# ---------- Streaming CSV Writer ----------
class StreamingCSVWriter:
    def __init__(self, path, headers):
        self.path = path
        self.headers = list(headers)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.file = open(self.path, "a", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        try:
            self.file.seek(0, os.SEEK_END)
            if self.file.tell() == 0:
                self.writer.writerow(self.headers)
                self.file.flush()
                os.fsync(self.file.fileno())
        except Exception:
            pass

    def write_row(self, row_values):
        try:
            self.writer.writerow(row_values)
            self.file.flush()
            try:
                os.fsync(self.file.fileno())
            except Exception:
                pass
        except Exception:
            pass

    def close(self):
        try:
            self.file.close()
        except Exception:
            pass

# ---------- Serial Thread for GNSS ----------
class SerialReaderThread(QThread):
    data_received = pyqtSignal(str)
    error = pyqtSignal(str)
    disconnected = pyqtSignal()

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = False
        self.conn = None

    def run(self):
        try:
            self.conn = serial.Serial(self.port, self.baud, timeout=1)
        except Exception as e:
            self.error.emit(str(e))
            return

        self.running = True
        while self.running:
            try:
                if self.conn.in_waiting:
                    raw = self.conn.readline()
                    if raw:
                        try:
                            line = raw.decode('ascii', errors='ignore').strip()
                        except Exception:
                            line = raw.decode('latin1', errors='ignore').strip()
                        if line:
                            self.data_received.emit(line)
                else:
                    time.sleep(0.005)
            except Exception as e:
                self.error.emit(str(e))
                break

        if self.conn and self.conn.is_open:
            try:
                self.conn.close()
            except Exception:
                pass
        self.disconnected.emit()

    def stop(self):
        self.running = False

# ---------- Sky Plot Widget ----------
class SkyPlotWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(520, 420)
        self.center_lat = None
        self.center_lon = None
        self.points_by_pipeline = {}
        self.pipeline_colors = {}
        self.zoom_percent = 100
        self.selection_m = 2.0
        self.show_selection = True
        self.last_counts = {}
        self.ground_truth = None
        self.show_ground_truth = False
        self.ground_truth_color = QColor(255, 80, 80)
        self.ground_truth_marker_size = 8
        self.dark_mode = True

    def set_center(self, lat, lon):
        self.center_lat = lat
        self.center_lon = lon
        self.update()

    def set_pipeline_points(self, name, pts):
        self.points_by_pipeline[name] = pts or []
        self.update()

    def set_pipeline_color(self, name, qcolor):
        self.pipeline_colors[name] = qcolor

    def set_zoom_percent(self, p):
        self.zoom_percent = max(10, min(int(p), 500))
        self.update()

    def set_selection_m(self, m):
        self.selection_m = max(0.1, float(m))
        self.update()

    def set_ground_truth(self, lat, lon):
        try:
            self.ground_truth = (float(lat), float(lon))
        except Exception:
            self.ground_truth = None
        self.update()

    def clear_ground_truth(self):
        self.ground_truth = None
        self.update()

    def set_show_ground_truth(self, show: bool):
        self.show_ground_truth = bool(show)
        self.update()

    def set_dark_mode(self, dark: bool):
        self.dark_mode = dark
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        if self.dark_mode:
            bg = QColor(5, 5, 12);       grid_c = QColor(70, 75, 90)
            ring_lbl = QColor(110, 120, 140); comp_line = QColor(100, 110, 130)
            comp_lbl = QColor(210, 215, 225); sel_c = QColor(255, 200, 0)
            ctr_c = QColor(0, 255, 120);  leg_c = QColor(210, 215, 225)
        else:
            bg = QColor(242, 246, 250); grid_c = QColor(170, 183, 200)
            ring_lbl = QColor(80, 100, 130); comp_line = QColor(140, 155, 175)
            comp_lbl = QColor(25, 40, 75);  sel_c = QColor(220, 130, 0)
            ctr_c = QColor(16, 155, 70);  leg_c = QColor(25, 40, 75)
        painter.fillRect(self.rect(), bg)
        w, h = self.width(), self.height()
        size = min(w, h)
        cx, cy = w / 2, h / 2
        base_radius = size / 2 - 40
        viz_radius = min(base_radius * (self.zoom_percent / 100.0), base_radius)

        painter.setPen(QPen(grid_c, 1, Qt.DashLine))
        rings = 4
        eff_max = max(1.0, self.selection_m * 4.0)
        for i in range(1, rings + 1):
            r = viz_radius * (i / rings)
            painter.drawEllipse(QPointF(cx, cy), r, r)
            painter.setPen(QPen(ring_lbl, 1))
            painter.setFont(QFont("Segoe UI", 8))
            painter.drawText(int(cx + 6), int(cy - r + 12), f"{eff_max * (i / rings):.1f} m")
            painter.setPen(QPen(grid_c, 1, Qt.DashLine))

        painter.setPen(QPen(comp_line, 1))
        painter.drawLine(int(cx), int(cy - viz_radius), int(cx), int(cy + viz_radius))
        painter.drawLine(int(cx - viz_radius), int(cy), int(cx + viz_radius), int(cy))
        painter.setPen(QPen(comp_lbl, 1))
        painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
        painter.drawText(int(cx - 10), int(cy - viz_radius - 6), "N")
        painter.drawText(int(cx + viz_radius + 6), int(cy + 4), "E")
        painter.drawText(int(cx - 10), int(cy + viz_radius + 14), "S")
        painter.drawText(int(cx - viz_radius - 18), int(cy + 4), "W")

        sel_px = viz_radius * min(1.0, self.selection_m / eff_max)
        if self.show_selection:
            painter.setPen(QPen(sel_c, 2, Qt.DashLine))
            painter.setBrush(QBrush(Qt.NoBrush))
            painter.drawEllipse(QPointF(cx, cy), sel_px, sel_px)
            painter.setPen(QPen(sel_c, 1))
            painter.setFont(QFont("Segoe UI", 9))
            painter.drawText(int(cx + 8), int(cy - sel_px + 12), f"Sel ≤ {self.selection_m:.2f} m")

        painter.setPen(QPen(ctr_c, 2))
        painter.setBrush(QBrush(ctr_c))
        painter.drawEllipse(QPointF(cx, cy), 5, 5)

        self.last_counts.clear()
        for name, pts in self.points_by_pipeline.items():
            color = self.pipeline_colors.get(name, QColor(255, 255, 255))
            count_in = 0
            for fix in pts:
                if fix.lat is None or fix.lon is None or self.center_lat is None or self.center_lon is None:
                    continue
                d = haversine_distance_m(self.center_lat, self.center_lon, fix.lat, fix.lon)
                if d is None:
                    continue
                b = bearing_deg(self.center_lat, self.center_lon, fix.lat, fix.lon)
                r = viz_radius * min(1.0, d / eff_max)
                ang = math.radians(b)
                x = cx + r * math.sin(ang)
                y = cy - r * math.cos(ang)
                painter.setPen(QPen(color, 1))
                painter.setBrush(QBrush(color))
                painter.drawEllipse(QPointF(x, y), 4, 4)
                dx, dy = x - cx, y - cy
                if dx*dx + dy*dy <= sel_px*sel_px:
                    count_in += 1
            self.last_counts[name] = (count_in, len(pts))

        if self.show_ground_truth and self.ground_truth is not None and self.center_lat is not None and self.center_lon is not None:
            try:
                gt_lat, gt_lon = self.ground_truth
                d = haversine_distance_m(self.center_lat, self.center_lon, gt_lat, gt_lon)
                if d is not None:
                    b = bearing_deg(self.center_lat, self.center_lon, gt_lat, gt_lon)
                    r = viz_radius * min(1.0, d / eff_max)
                    ang = math.radians(b)
                    gx = cx + r * math.sin(ang)
                    gy = cy - r * math.cos(ang)
                    pen = QPen(self.ground_truth_color, 2)
                    painter.setPen(pen)
                    sz = self.ground_truth_marker_size
                    painter.drawLine(int(gx - sz), int(gy), int(gx + sz), int(gy))
                    painter.drawLine(int(gx), int(gy - sz), int(gx), int(gy + sz))
                    painter.setFont(QFont("Segoe UI", 9))
                    painter.drawText(int(gx + sz + 4), int(gy - 2), "Ground Truth")
            except Exception:
                pass

        leg_x, leg_y = 10, 10
        painter.setFont(QFont("Segoe UI", 9))
        for i, (name, pts) in enumerate(self.points_by_pipeline.items()):
            color = self.pipeline_colors.get(name, QColor(255, 255, 255))
            painter.setPen(QPen(color, 1))
            painter.setBrush(QBrush(color))
            painter.drawRect(leg_x, leg_y + i * 20, 12, 12)
            painter.setPen(QPen(leg_c, 1))
            cnt = self.last_counts.get(name, (0, len(pts)))
            painter.drawText(leg_x + 18, leg_y + i * 20 + 12, f"{name}  in:{cnt[0]} / tot:{cnt[1]}")

# ---------- Signal Strength Widget ----------
class SignalStrengthWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.satellites = []
        self.setMinimumHeight(130)
        self.dark_mode = True

    def update_satellites(self, sats):
        self.satellites = sorted(sats, key=lambda s: s.prn if hasattr(s, 'prn') else 9999)
        self.update()

    def set_dark_mode(self, dark: bool):
        self.dark_mode = dark
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        if self.dark_mode:
            bg = QColor(15, 20, 35);   grid_c = QColor(50, 58, 78)
            bar_gps = QColor(22, 200, 80); bar_other = QColor(100, 115, 145)
            txt = QColor(185, 195, 215); no_data = QColor(120, 135, 160)
        else:
            bg = QColor(248, 250, 252); grid_c = QColor(203, 213, 225)
            bar_gps = QColor(22, 163, 74); bar_other = QColor(148, 163, 184)
            txt = QColor(55, 65, 81);   no_data = QColor(100, 116, 139)
        painter.fillRect(self.rect(), bg)
        if not self.satellites:
            painter.setPen(QPen(no_data, 1))
            painter.setFont(QFont("Segoe UI", 11))
            painter.drawText(self.rect(), Qt.AlignCenter, "No satellite tracking data")
            return
        w, h = self.width(), self.height()
        num = len(self.satellites)
        bar_width = max(18, w // (num + 2))
        painter.setPen(QPen(grid_c, 1))
        for snr_ref in [10, 20, 30, 40, 50]:
            y = h - (snr_ref / MAX_CNO) * (h - 40) - 20
            painter.drawLine(0, int(y), w, int(y))
        for i, s in enumerate(self.satellites):
            x = i * bar_width + 5
            snr = max(0.0, min(s.snr if s.snr is not None else 0.0, MAX_CNO))
            bar_height = (snr / MAX_CNO) * (h - 40)
            prn = getattr(s, 'prn', '?')
            color = bar_gps if 1 <= prn <= 32 else bar_other
            painter.fillRect(int(x), int(h - bar_height - 20), int(bar_width - 6), int(bar_height), color)
            painter.setPen(QPen(txt, 1))
            painter.setFont(QFont("Segoe UI", 7))
            painter.drawText(int(x), int(h - 5), str(prn))
            painter.drawText(int(x), int(h - bar_height - 22), f"{snr:.0f}")

# ---------- Extraction Helpers ----------
def extract_fields_from_msg(msg, sentence_name):
    out = {}
    try:
        if sentence_name == 'GGA':
            out['utc_time'] = getattr(msg, 'timestamp', None)
            out['latitude'] = safe_float(getattr(msg, 'latitude', None))
            out['longitude'] = safe_float(getattr(msg, 'longitude', None))
            out['gps_qual'] = getattr(msg, 'gps_qual', None)
            out['num_sats'] = getattr(msg, 'num_sats', None)
            out['horizontal_dil'] = safe_float(getattr(msg, 'horizontal_dil', None))
            out['altitude'] = safe_float(getattr(msg, 'altitude', None))
            out['geo_sep'] = safe_float(getattr(msg, 'geo_sep', None))
            if out.get('altitude') is not None and out.get('geo_sep') is not None:
                out['alt_hae'] = out['altitude'] + out['geo_sep']
            else:
                out['alt_hae'] = None
            out['raw'] = str(msg)
        elif sentence_name == 'RMC':
            out['utc_time'] = getattr(msg, 'timestamp', None)
            out['status'] = getattr(msg, 'status', None)
            out['latitude'] = safe_float(getattr(msg, 'latitude', None))
            out['longitude'] = safe_float(getattr(msg, 'longitude', None))
            out['spd_over_grnd'] = safe_float(getattr(msg, 'spd_over_grnd', None))
            out['true_course'] = safe_float(getattr(msg, 'true_course', None))
            out['datestamp'] = getattr(msg, 'datestamp', None)
            out['raw'] = str(msg)
        elif sentence_name == 'GSA':
            out['mode'] = getattr(msg, 'mode', None)
            out['mode2'] = getattr(msg, 'mode_fix_type', None) if hasattr(msg, 'mode_fix_type') else None
            out['pdop'] = safe_float(getattr(msg, 'pdop', None))
            out['hdop'] = safe_float(getattr(msg, 'hdop', None))
            out['vdop'] = safe_float(getattr(msg, 'vdop', None))
            used = [str(getattr(msg, f"sv_id_{i:02d}", "")) for i in range(1, 13) if getattr(msg, f"sv_id_{i:02d}", None)]
            out['satellites_used'] = ",".join(used)
            out['raw'] = str(msg)
        elif sentence_name == 'GSV':
            sat_info = []
            for i in range(1, 5):
                prn = getattr(msg, f"sv_prn_num_{i}", None)
                elev = getattr(msg, f"elevation_deg_{i}", None)
                az = getattr(msg, f"azimuth_{i}", None)
                snr = getattr(msg, f"snr_{i}", None)
                if prn is not None:
                    sat_info.append(f"{prn}|{elev or ''}|{az or ''}|{snr or ''}")
            out['num_messages'] = getattr(msg, 'num_msgs', None)
            out['msg_num'] = getattr(msg, 'msg_num', None)
            out['sat_info'] = ";".join(sat_info)
            out['raw'] = str(msg)
        elif sentence_name == 'GLL':
            out['utc_time'] = getattr(msg, 'timestamp', None)
            out['latitude'] = safe_float(getattr(msg, 'latitude', None))
            out['longitude'] = safe_float(getattr(msg, 'longitude', None))
            out['status'] = getattr(msg, 'status', None) if hasattr(msg, 'status') else None
            out['raw'] = str(msg)
        elif sentence_name == 'VTG':
            out['true_track'] = safe_float(getattr(msg, 'true_track', None))
            spd_kmh = getattr(msg, 'spd_over_grnd_kmph', None)
            if spd_kmh is None:
                spd_kn = safe_float(getattr(msg, 'spd_over_grnd', None))
                spd_kmh = spd_kn * 1.852 if spd_kn is not None else None
            out['spd_kmh'] = safe_float(spd_kmh)
            out['spd_knots'] = safe_float(getattr(msg, 'spd_over_grnd', None))
            out['raw'] = str(msg)
        elif sentence_name == 'ZDA':
            out['utc_time'] = getattr(msg, 'timestamp', None)
            out['datestamp'] = getattr(msg, 'datestamp', None)
            out['local_zone_hours'] = getattr(msg, 'local_zone_hours', None)
            out['local_zone_minutes'] = getattr(msg, 'local_zone_minutes', None)
            out['raw'] = str(msg)
        else:
            out['raw'] = str(msg)
    except Exception:
        out['raw'] = str(msg)
    return out

# ---------- Pipeline Filter Routines ----------
def stage_EMA(state, fix, alpha=0.3):
    if fix.lat is None or fix.lon is None:
        return None
    if state.get('ema_lat') is None:
        state['ema_lat'], state['ema_lon'] = fix.lat, fix.lon
    else:
        state['ema_lat'] = alpha * fix.lat + (1 - alpha) * state['ema_lat']
        state['ema_lon'] = alpha * fix.lon + (1 - alpha) * state['ema_lon']
    df = DataFix(fix.ts, state['ema_lat'], state['ema_lon'], fix.alt, fix.is_dgps,
                 fix.hdop, fix.visible_sats, fix.used_sats, fix.avg_used_snr,
                 fix.x, fix.y, fix.z, vdop=fix.vdop, alt_hae=fix.alt_hae)
    df.raw_east, df.raw_north, df.tropo_delay = fix.raw_east, fix.raw_north, fix.tropo_delay
    df.imu_metrics = fix.imu_metrics
    return df

def stage_SMA(state, fix, window=5):
    buf = state.setdefault('sma_buf', deque(maxlen=window))
    if fix.lat is None or fix.lon is None:
        return None
    buf.append((fix.lat, fix.lon))
    avg_lat = sum(p[0] for p in buf) / len(buf)
    avg_lon = sum(p[1] for p in buf) / len(buf)
    df = DataFix(fix.ts, avg_lat, avg_lon, fix.alt, fix.is_dgps,
                 fix.hdop, fix.visible_sats, fix.used_sats, fix.avg_used_snr,
                 fix.x, fix.y, fix.z, vdop=fix.vdop, alt_hae=fix.alt_hae)
    df.raw_east, df.raw_north, df.tropo_delay = fix.raw_east, fix.raw_north, fix.tropo_delay
    df.imu_metrics = fix.imu_metrics
    return df

def stage_HAMPEL(state, fix, window=7, k=3.0):
    buf = state.setdefault('h_buf', deque(maxlen=window))
    if fix.lat is None or fix.lon is None:
        return None
    buf.append((fix.lat, fix.lon))
    lats = [p[0] for p in buf]
    lons = [p[1] for p in buf]
    med_lat = sorted(lats)[len(lats)//2]
    med_lon = sorted(lons)[len(lons)//2]
    mad_lat = 1.4826 * (sorted([abs(a - med_lat) for a in lats])[len(lats)//2])
    mad_lon = 1.4826 * (sorted([abs(a - med_lon) for a in lons])[len(lons)//2])
    out_lat = mad_lat > 1e-9 and abs(fix.lat - med_lat) > k * mad_lat
    out_lon = mad_lon > 1e-9 and abs(fix.lon - med_lon) > k * mad_lon
    df = DataFix(fix.ts, med_lat if out_lat else fix.lat, med_lon if out_lon else fix.lon,
                 fix.alt, fix.is_dgps, fix.hdop, fix.visible_sats, fix.used_sats, fix.avg_used_snr,
                 fix.x, fix.y, fix.z, vdop=fix.vdop, alt_hae=fix.alt_hae)
    df.raw_east, df.raw_north, df.tropo_delay = fix.raw_east, fix.raw_north, fix.tropo_delay
    df.imu_metrics = fix.imu_metrics
    return df

def stage_KF_ENU(state, fix, ref_lat, ref_lon, q_pos=0.1, q_vel=0.1, dgps_factor=4.0):
    if ref_lat is None or ref_lon is None or fix.lat is None or fix.lon is None:
        return fix
    north_m = haversine_distance_m(ref_lat, ref_lon, fix.lat, ref_lon)
    if fix.lat < ref_lat:
        north_m = -north_m
    east_m = haversine_distance_m(ref_lat, ref_lon, ref_lat, fix.lon)
    if fix.lon < ref_lon:
        east_m = -east_m
    now = time.time()
    kf = state.get('kf')
    if kf is None:
        kf = Kalman2D(q_pos=q_pos, q_vel=q_vel, r_pos=1.0)
        state['kf'] = kf
    if kf.x is None:
        kf.init(east_m, north_m, now)
    else:
        kf.predict(now)
    sigma = 1.0
    if fix.hdop is not None:
        sigma = max(0.5, float(fix.hdop))
    if fix.is_dgps:
        sigma /= max(1.0, dgps_factor)
    kf.update(east_m, north_m, sigma * sigma)
    ex, ey, ve, vn = kf.state()
    latf = ref_lat + (ey / EARTH_R) * (180.0 / math.pi)
    lonf = ref_lon + (ex / (EARTH_R * math.cos(math.radians(ref_lat)))) * (180.0 / math.pi)
    df = DataFix(fix.ts, latf, lonf, fix.alt, fix.is_dgps, fix.hdop, fix.visible_sats,
                 fix.used_sats, fix.avg_used_snr, fix.x, fix.y, fix.z, vdop=fix.vdop, alt_hae=fix.alt_hae)
    df.raw_east, df.raw_north, df.tropo_delay = fix.raw_east, fix.raw_north, fix.tropo_delay
    df.imu_metrics = fix.imu_metrics
    return df

# ---------- Main Application Window ----------
class GNSSMonitor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.dark_mode = False  # default: light/white theme
        self.satellites = {}
        self.satellites_in_use = set()
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.hdop = None
        self.vdop = None
        self.pdop = None
        self.fix_mode = "No Fix"
        self.num_satellites = 0
        
        self.serial_thread = None
        self.imu_thread = None
        self.latest_imu = None
        
        self.fix_buffer = []
        self._fix_times = deque(maxlen=12000)
        self.recent_nmea = deque(maxlen=10000)
        self.sensor_log = []
        self.sensor_csv_columns = [
            'timestamp_utc', 'gps_latitude_deg', 'gps_longitude_deg', 'gps_altitude_m',
            'gps_fix_mode', 'gps_hdop', 'gps_pdop', 'gps_vdop',
            'imu_ax_mps2', 'imu_ay_mps2', 'imu_az_mps2',
            'imu_gx_deg_s', 'imu_gy_deg_s', 'imu_gz_deg_s',
            'imu_roll_deg', 'imu_pitch_deg', 'imu_vibration_g', 'imu_ts_utc'
        ]
        
        try:
            self.raw_log_writer = open(RAW_LOG_PATH, "a", encoding="utf-8")
        except Exception:
            self.raw_log_writer = None

        self.center_lat = None
        self.center_lon = None
        self.center_smoothed_lat = None
        self.center_smoothed_lon = None
        self.n_min_dgps = 4
        self.window_s = 60
        self.outlier_m = 3.0
        self.w_dgps = 4.0
        self.w_nondgps = 1.0
        self.auto_fallback = True
        self.compute_center_enabled = True
        self.use_entire_duration = False

        self.pipelines = []
        for i in range(4):
            self.pipelines.append({
                "enabled": False,
                "name": f"Pipeline {i+1}",
                "method": "Raw",
                "state": {},
                "color": QColor(255, 255 - i*50, 100 + i*30),
                "params": {},
                "output": []
            })

        self.epoch_map = {}
        self.streaming_csv = None
        self.streaming_csv_cols = None
        self.epoch_timeout_s = 0.4
        self.latest_msg_map = {}

        self.available_sentences = [
            ("GGA", 10), ("RMC", 10), ("GSA", 1), ("GSV", 1),
            ("GLL", 0), ("VTG", 0), ("ZDA", 1)
        ]

        self.sentence_field_config = {}
        for name, _ in self.available_sentences:
            self.sentence_field_config[name] = {}
            for fld, _ in SENTENCE_FIELDS.get(name, []):
                self.sentence_field_config[name][fld] = {'table': False, 'csv': False}

        self.init_ui()
        self.start_imu_thread()

    def init_ui(self):
        self.setWindowTitle("GNSS + IMU Sensor Fusion Precision Logger")
        self.setGeometry(30, 30, 1620, 970)

        menubar = self.menuBar()
        file_menu = menubar.addMenu("File")
        save_action = QAction("Save Merged CSV (Epoch + IMU)", self)
        save_action.triggered.connect(self.save_pipelines_csv)
        file_menu.addAction(save_action)
        file_menu.addSeparator()
        exit_action = QAction("Exit", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        toolbar = QToolBar()
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        toolbar.addWidget(QLabel("  Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(200)
        toolbar.addWidget(self.port_combo)

        refresh_btn = QPushButton("Refresh Ports")
        refresh_btn.clicked.connect(self.refresh_ports)
        toolbar.addWidget(refresh_btn)

        toolbar.addWidget(QLabel("  Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")
        toolbar.addWidget(self.baud_combo)

        self.connect_btn = QPushButton("Connect GPS")
        self.connect_btn.clicked.connect(self.connect_port)
        toolbar.addWidget(self.connect_btn)
        toolbar.addSeparator()

        toolbar.addWidget(QLabel("  Radius (m):"))
        self.sel_spin = QDoubleSpinBox()
        self.sel_spin.setRange(0.1, 100.0)
        self.sel_spin.setValue(2.0)
        self.sel_spin.setFixedWidth(72)
        toolbar.addWidget(self.sel_spin)

        toolbar.addWidget(QLabel("  Zoom (%):"))
        self.zoom_spin = QSpinBox()
        self.zoom_spin.setRange(20, 500)
        self.zoom_spin.setValue(120)
        self.zoom_spin.setFixedWidth(66)
        toolbar.addWidget(self.zoom_spin)
        toolbar.addSeparator()

        btn_showtable = QPushButton("Show Table")
        btn_showtable.clicked.connect(self.show_data_table)
        toolbar.addWidget(btn_showtable)

        # "Export Log" = comprehensive GPS+IMU fixed-schema export (save_sensor_csv).
        # "Save Merged CSV" (save_pipelines_csv) is accessible via File menu.
        btn_sensor_csv = QPushButton("Export Log")
        btn_sensor_csv.setToolTip(
            "Export full GPS + IMU snapshot log (fixed schema: lat/lon/alt, fix mode,\n"
            "HDOP/PDOP/VDOP, ax/ay/az, gx/gy/gz, roll, pitch, vibration, IMU timestamp).\n\n"
            "For a user-configurable merged epoch CSV use  File → Save Merged CSV."
        )
        btn_sensor_csv.clicked.connect(self.save_sensor_csv)
        toolbar.addWidget(btn_sensor_csv)

        # ── Theme toggle (inline, right after Export Log) ─────────────────────
        self.theme_btn = QToolButton()
        self.theme_btn.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.theme_btn.setFixedWidth(128)
        self.theme_btn.setPopupMode(QToolButton.MenuButtonPopup)
        self.theme_btn.clicked.connect(self.toggle_theme)
        _theme_menu = QMenu(self.theme_btn)
        _light_act = QAction("☀  Light Mode", self)
        _light_act.triggered.connect(lambda: self._set_theme(False))
        _dark_act  = QAction("🌙  Dark Mode",  self)
        _dark_act.triggered.connect(lambda: self._set_theme(True))
        _theme_menu.addAction(_light_act)
        _theme_menu.addAction(_dark_act)
        self.theme_btn.setMenu(_theme_menu)
        toolbar.addWidget(self.theme_btn)

        toolbar.addSeparator()

        stream_start_btn = QPushButton("Start Stream")
        stream_start_btn.clicked.connect(lambda: self.start_streaming_csv())
        toolbar.addWidget(stream_start_btn)

        stream_stop_btn = QPushButton("Stop Stream")
        stream_stop_btn.clicked.connect(lambda: self.stop_streaming_csv())
        toolbar.addWidget(stream_stop_btn)

        btn_quick = QPushButton("Enable All")
        btn_quick.clicked.connect(self.quick_add_features)
        toolbar.addWidget(btn_quick)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(14, 10, 14, 10)
        main_layout.setSpacing(14)

        # ── Left Column ─────────────────────────────────────
        left_v = QVBoxLayout()
        left_v.setSpacing(10)

        sky_group = QGroupBox("Sky Plot  —  Trajectory Deviation (North-Up)")
        sky_layout = QVBoxLayout()
        sky_layout.setContentsMargins(6, 16, 6, 6)
        self.sky_plot = SkyPlotWidget()
        sky_layout.addWidget(self.sky_plot)
        sky_group.setLayout(sky_layout)
        left_v.addWidget(sky_group, stretch=6)

        info_group = QGroupBox("Position, Quality & IMU Live Status")
        info_layout = QGridLayout()
        info_layout.setContentsMargins(16, 22, 16, 14)
        info_layout.setHorizontalSpacing(24)
        info_layout.setVerticalSpacing(10)
        info_layout.setColumnStretch(0, 1)
        info_layout.setColumnStretch(1, 1)

        self.lat_label = QLabel("Latitude: —")
        self.lon_label = QLabel("Longitude: —")
        self.alt_label = QLabel("Altitude: —")
        self.fix_label = QLabel("Fix: No Fix")
        self.hdop_label = QLabel("HDOP: —")
        self.sat_label = QLabel("Sats: 0 used / 0 in view")
        self.fs_label = QLabel("Sampling: — Hz")
        self.imu_att_label = QLabel("Roll: —° | Pitch: —°")
        self.imu_acc_label = QLabel("Accel (m/s²): [ — , — , — ]")
        self.imu_vib_label = QLabel("Vib Magnitude: — g")

        for lbl in [self.lat_label, self.lon_label, self.alt_label, self.fix_label,
                    self.hdop_label, self.sat_label, self.fs_label,
                    self.imu_att_label, self.imu_acc_label, self.imu_vib_label]:
            lbl.setStyleSheet("font-size: 10pt; font-weight: 600; padding: 4px 8px;")

        info_layout.addWidget(self.lat_label,      0, 0)
        info_layout.addWidget(self.fix_label,      0, 1)
        info_layout.addWidget(self.lon_label,      1, 0)
        info_layout.addWidget(self.hdop_label,     1, 1)
        info_layout.addWidget(self.alt_label,      2, 0)
        info_layout.addWidget(self.sat_label,      2, 1)
        info_layout.addWidget(self.fs_label,       3, 0)
        info_layout.addWidget(self.imu_vib_label,  3, 1)
        info_layout.addWidget(self.imu_att_label,  4, 0)
        info_layout.addWidget(self.imu_acc_label,  4, 1)
        info_group.setLayout(info_layout)
        left_v.addWidget(info_group, stretch=2)

        sensor_group = QGroupBox("GPS + IMU Snapshot Log")
        sensor_layout = QVBoxLayout(sensor_group)
        sensor_layout.setContentsMargins(8, 18, 8, 8)
        self.sensor_table = QTableWidget(0, 9)
        self.sensor_table.setHorizontalHeaderLabels([
            'Timestamp', 'Latitude', 'Longitude', 'Altitude', 'Fix', 'HDOP',
            'Roll', 'Pitch', 'Vibration'
        ])
        self.sensor_table.setAlternatingRowColors(True)
        self.sensor_table.setSortingEnabled(True)
        self.sensor_table.horizontalHeader().setStretchLastSection(True)
        self.sensor_table.verticalHeader().setVisible(False)
        sensor_layout.addWidget(self.sensor_table)
        left_v.addWidget(sensor_group, stretch=2)

        main_layout.addLayout(left_v, 2)

        # ── Right Column ─────────────────────────────────────
        right_v = QVBoxLayout()
        right_v.setSpacing(10)

        pipelines_group = QGroupBox("Filters & Sensor Fusion Pipelines")
        pg_layout = QVBoxLayout()
        pg_layout.setContentsMargins(12, 20, 12, 12)
        pg_layout.setSpacing(10)
        self.pipeline_widgets = []
        methods = ["Raw", "EMA", "SMA", "Hampel", "KF_ENU"]

        for i in range(4):
            row = QWidget()
            hl = QHBoxLayout(row)
            hl.setContentsMargins(6, 4, 6, 4)
            hl.setSpacing(10)
            enable_cb = QCheckBox("On")
            name_lbl = QLabel(self.pipelines[i]['name'])
            name_lbl.setMinimumWidth(76)
            method_combo = QComboBox()
            method_combo.addItems(methods)
            method_combo.setMinimumWidth(90)
            color_btn = QPushButton("Color")
            color_btn.setFixedWidth(60)
            params_btn = QPushButton("Params")
            params_btn.setFixedWidth(64)

            hl.addWidget(enable_cb)
            hl.addWidget(name_lbl)
            hl.addWidget(method_combo)
            hl.addWidget(color_btn)
            hl.addWidget(params_btn)
            pg_layout.addWidget(row)

            self.pipeline_widgets.append({
                "enable": enable_cb,
                "label": name_lbl,
                "method": method_combo,
                "color_btn": color_btn,
                "params_btn": params_btn
            })

            def mk_color(idx):
                return lambda: self._pick_color(idx)
            color_btn.clicked.connect(mk_color(i))

            def mk_change(idx, combo):
                return lambda: self._change_method(idx, combo.currentText())
            method_combo.currentIndexChanged.connect(mk_change(i, method_combo))

            def mk_enable(idx, cb):
                return lambda st: self._toggle_pipeline(idx, st == Qt.Checked)
            enable_cb.stateChanged.connect(mk_enable(i, enable_cb))

        pipelines_group.setLayout(pg_layout)
        right_v.addWidget(pipelines_group, stretch=2)

        sentences_group = QGroupBox("UBX Message Frequency & Fields")
        s_layout = QGridLayout()
        s_layout.setContentsMargins(12, 20, 12, 12)
        s_layout.setHorizontalSpacing(10)
        s_layout.setVerticalSpacing(8)
        self.sentence_widgets = []
        for i, (name, default_rate) in enumerate(self.available_sentences):
            name_btn = QPushButton(name)
            name_btn.clicked.connect(self._make_fields_click_cb(name))
            cb = QCheckBox()
            cb.setChecked(default_rate > 0)
            sp = QSpinBox()
            sp.setRange(0, 20)
            sp.setValue(default_rate)
            fld_btn = QPushButton("Fields...")
            fld_btn.clicked.connect(self._make_fields_click_cb(name))
            s_layout.addWidget(name_btn, i, 0)
            s_layout.addWidget(cb, i, 1)
            s_layout.addWidget(sp, i, 2)
            s_layout.addWidget(fld_btn, i, 3)
            self.sentence_widgets.append({'name': name, 'enable': cb, 'rate': sp})

        apply_btn = QPushButton("Apply Rates to ZED-F9P")
        apply_btn.clicked.connect(self.apply_sentence_config)
        s_layout.addWidget(apply_btn, len(self.available_sentences), 0, 1, 4)
        sentences_group.setLayout(s_layout)
        right_v.addWidget(sentences_group, stretch=3)

        sig_group = QGroupBox("Satellite Signal Strength")
        sig_layout = QVBoxLayout(sig_group)
        sig_layout.setContentsMargins(8, 16, 8, 8)
        self.signal_widget = SignalStrengthWidget()
        sig_layout.addWidget(self.signal_widget)
        right_v.addWidget(sig_group, stretch=2)

        main_layout.addLayout(right_v, 1)

        self.zoom_spin.valueChanged.connect(self.sky_plot.set_zoom_percent)
        self.sel_spin.valueChanged.connect(self.sky_plot.set_selection_m)
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.ui_tick)
        self.ui_timer.start(200)
        self.refresh_ports()
        self.apply_theme()

    def apply_theme(self):
        """Apply the current theme (light or dark) to the entire UI."""
        qss = DARK_STYLE if self.dark_mode else LIGHT_STYLE
        # Append per-theme checkbox indicator override so the tick mark is
        # clearly visible regardless of OS / platform checkbox rendering.
        qss += CHECKBOX_DARK_CSS if self.dark_mode else CHECKBOX_LIGHT_CSS
        self.setStyleSheet(qss)
        if hasattr(self, 'sky_plot'):
            self.sky_plot.set_dark_mode(self.dark_mode)
        if hasattr(self, 'signal_widget'):
            self.signal_widget.set_dark_mode(self.dark_mode)
        if hasattr(self, 'theme_btn'):
            # Sun icon = currently dark → switch to light; Moon = currently light → switch to dark
            icon_file = _sun_url if self.dark_mode else _moon_url
            self.theme_btn.setIcon(QIcon(icon_file))
            self.theme_btn.setIconSize(QSize(20, 20))
            self.theme_btn.setText(" Light" if self.dark_mode else " Dark")

    def toggle_theme(self):
        """Toggle between light and dark themes (toolbar button click)."""
        self.dark_mode = not self.dark_mode
        self.apply_theme()

    def _set_theme(self, dark: bool):
        """Explicitly set light or dark theme (dropdown menu action)."""
        self.dark_mode = dark
        self.apply_theme()

    def start_imu_thread(self):
        if HAS_SMBUS:
            self.imu_thread = IMUReaderThread(i2c_bus=1, address=0x68)
            self.imu_thread.sample_received.connect(self._on_imu_sample)
            self.imu_thread.error.connect(lambda err: self.statusBar().showMessage(f"IMU: {err}"))
            self.imu_thread.start()
        else:
            self.statusBar().showMessage("smbus2 not found. IMU streaming inactive.")

    def _on_imu_sample(self, sample):
        self.latest_imu = sample
        self._sync_sensor_log()

    def _sync_sensor_log(self):
        if not self.sensor_log:
            return
        last = self.sensor_log[-1]
        if self.latest_imu:
            imu = self.latest_imu
            last['imu_ts_utc'] = imu.get('ts', datetime.now(timezone.utc).isoformat())
            last['imu_ax_mps2'] = imu.get('ax')
            last['imu_ay_mps2'] = imu.get('ay')
            last['imu_az_mps2'] = imu.get('az')
            last['imu_gx_deg_s'] = imu.get('gx')
            last['imu_gy_deg_s'] = imu.get('gy')
            last['imu_gz_deg_s'] = imu.get('gz')
            last['imu_roll_deg'] = imu.get('roll')
            last['imu_pitch_deg'] = imu.get('pitch')
            last['imu_vibration_g'] = imu.get('vib')
        self._refresh_sensor_table()

    def _append_sensor_record(self, fix):
        row = {
            'timestamp_utc': fix.ts if fix and fix.ts else datetime.now(timezone.utc).isoformat(),
            'gps_latitude_deg': fix.lat if fix and fix.lat is not None else '',
            'gps_longitude_deg': fix.lon if fix and fix.lon is not None else '',
            'gps_altitude_m': fix.alt if fix and fix.alt is not None else '',
            'gps_fix_mode': self.fix_mode,
            'gps_hdop': self.hdop,
            'gps_pdop': self.pdop,
            'gps_vdop': self.vdop,
            'imu_ax_mps2': '',
            'imu_ay_mps2': '',
            'imu_az_mps2': '',
            'imu_gx_deg_s': '',
            'imu_gy_deg_s': '',
            'imu_gz_deg_s': '',
            'imu_roll_deg': '',
            'imu_pitch_deg': '',
            'imu_vibration_g': '',
            'imu_ts_utc': ''
        }
        if self.latest_imu:
            imu = self.latest_imu
            row['imu_ts_utc'] = imu.get('ts', datetime.now(timezone.utc).isoformat())
            row['imu_ax_mps2'] = imu.get('ax')
            row['imu_ay_mps2'] = imu.get('ay')
            row['imu_az_mps2'] = imu.get('az')
            row['imu_gx_deg_s'] = imu.get('gx')
            row['imu_gy_deg_s'] = imu.get('gy')
            row['imu_gz_deg_s'] = imu.get('gz')
            row['imu_roll_deg'] = imu.get('roll')
            row['imu_pitch_deg'] = imu.get('pitch')
            row['imu_vibration_g'] = imu.get('vib')
        self.sensor_log.append(row)
        if len(self.sensor_log) > 5000:
            self.sensor_log.pop(0)
        self._refresh_sensor_table()

    def _refresh_sensor_table(self):
        rows = list(self.sensor_log)[-100:]
        self.sensor_table.setRowCount(len(rows))
        for r, row in enumerate(rows):
            values = [
                row.get('timestamp_utc', ''),
                row.get('gps_latitude_deg', ''),
                row.get('gps_longitude_deg', ''),
                row.get('gps_altitude_m', ''),
                row.get('gps_fix_mode', ''),
                row.get('gps_hdop', ''),
                row.get('imu_roll_deg', ''),
                row.get('imu_pitch_deg', ''),
                row.get('imu_vibration_g', '')
            ]
            for c, value in enumerate(values):
                item = QTableWidgetItem('' if value in (None, '') else str(value))
                if isinstance(value, float):
                    item.setText(f'{value:.3f}')
                self.sensor_table.setItem(r, c, item)

    def _pick_color(self, idx):
        c = QColorDialog.getColor(self.pipelines[idx]['color'], self)
        if c.isValid():
            self.pipelines[idx]['color'] = c
            self.sky_plot.set_pipeline_color(self.pipelines[idx]['name'], c)

    def _change_method(self, idx, method):
        self.pipelines[idx]['method'] = method

    def _toggle_pipeline(self, idx, is_enabled):
        self.pipelines[idx]['enabled'] = is_enabled

    def _make_fields_click_cb(self, sentence_name):
        return lambda: self.open_fields_dialog(sentence_name)

    def open_fields_dialog(self, sentence_name):
        fields = SENTENCE_FIELDS.get(sentence_name, [])
        dlg = QDialog(self)
        dlg.setWindowTitle(f"Fields: {sentence_name}")
        lay = QVBoxLayout(dlg)
        grid = QGridLayout()
        grid.addWidget(QLabel("Field"), 0, 0)
        grid.addWidget(QLabel("Include in Table"), 0, 1)
        grid.addWidget(QLabel("Include in CSV"), 0, 2)
        check_widgets = []
        for i, (fld, label) in enumerate(fields):
            grid.addWidget(QLabel(label), i+1, 0)
            cb_table = QCheckBox()
            cb_table.setChecked(self.sentence_field_config[sentence_name].get(fld, {}).get('table', False))
            cb_csv = QCheckBox()
            cb_csv.setChecked(self.sentence_field_config[sentence_name].get(fld, {}).get('csv', False))
            grid.addWidget(cb_table, i+1, 1)
            grid.addWidget(cb_csv, i+1, 2)
            check_widgets.append((fld, cb_table, cb_csv))
        lay.addLayout(grid)
        btn_h = QHBoxLayout()
        ok = QPushButton("OK")
        cancel = QPushButton("Cancel")
        ok.clicked.connect(dlg.accept)
        cancel.clicked.connect(dlg.reject)
        btn_h.addWidget(ok)
        btn_h.addWidget(cancel)
        lay.addLayout(btn_h)
        if dlg.exec_() == QDialog.Accepted:
            for fld, ctab, ccsv in check_widgets:
                self.sentence_field_config[sentence_name][fld]['table'] = bool(ctab.isChecked())
                self.sentence_field_config[sentence_name][fld]['csv'] = bool(ccsv.isChecked())

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        if ports:
            for p in ports:
                self.port_combo.addItem(f"{p.device} - {p.description}", p.device)
        else:
            self.port_combo.addItem("No ports found", None)

    def connect_port(self):
        if self.serial_thread and self.serial_thread.isRunning():
            self.disconnect_port()
            return
        port = self.port_combo.currentData()
        if not port:
            self.statusBar().showMessage("Select a valid serial port")
            return
        baud = int(self.baud_combo.currentText())
        try:
            self.serial_thread = SerialReaderThread(port, baud)
            self.serial_thread.data_received.connect(self.process_nmea)
            self.serial_thread.error.connect(lambda e: self.statusBar().showMessage(f"Serial Error: {e}"))
            self.serial_thread.disconnected.connect(self._on_serial_disconnected)
            self.serial_thread.start()
            self.connect_btn.setText("Disconnect")
            self.statusBar().showMessage(f"Connected: {port} @ {baud}")
        except Exception as e:
            self.statusBar().showMessage(f"Conn failed: {e}")

    def disconnect_port(self):
        if self.serial_thread and self.serial_thread.isRunning():
            self.connect_btn.setEnabled(False)
            self.connect_btn.setText("Closing...")
            self.serial_thread.stop()

    def _on_serial_disconnected(self):
        self.connect_btn.setText("Connect GPS")
        self.connect_btn.setEnabled(True)
        self.statusBar().showMessage("GNSS disconnected")

    def apply_sentence_config(self):
        port = self.port_combo.currentData()
        if not port:
            return
        baud = int(self.baud_combo.currentText())
        req = [(w['name'], int(w['rate'].value()) if w['enable'].isChecked() else 0) for w in self.sentence_widgets]
        max_rate = max([r for _, r in req] + [1])
        meas_ms = max(20, int(round(1000.0 / max_rate)))
        pkts = [build_cfg_rate_ms(meas_rate_ms=meas_ms, nav_rate=1, time_ref=1)]
        for name, r in req:
            pkts.append(build_cfg_msg_for_nmea(name, rate_on_uart1=1 if r > 0 else 0))
        try:
            with serial.Serial(port, baud, timeout=1) as s:
                for p in pkts:
                    s.write(p)
                    s.flush()
                    time.sleep(0.05)
            self.statusBar().showMessage("ZED-F9P rates successfully applied")
        except Exception as e:
            QMessageBox.critical(self, "Config Error", str(e))

    def quick_add_features(self):
        for i, p in enumerate(self.pipelines):
            p['enabled'] = True
            self.pipeline_widgets[i]['enable'].setChecked(True)
        for w in self.sentence_widgets:
            w['enable'].setChecked(True)
            w['rate'].setValue(10)
        for sent, fields in SENTENCE_FIELDS.items():
            for fld, _ in fields:
                self.sentence_field_config[sent][fld]['table'] = True
                self.sentence_field_config[sent][fld]['csv'] = True
        self.statusBar().showMessage("All pipelines, IMU fields, and CSV streams fully active")

    def process_nmea(self, sentence):
        ts_iso = datetime.now(timezone.utc).isoformat()
        if self.raw_log_writer:
            try:
                self.raw_log_writer.write(f"{ts_iso} {sentence}\n")
                self.raw_log_writer.flush()
            except Exception:
                pass

        if not sentence.startswith('$'):
            return

        try:
            msg = pynmea2.parse(sentence)
        except Exception:
            return

        tname = msg.sentence_type if hasattr(msg, 'sentence_type') else type(msg).__name__
        self.recent_nmea.append((ts_iso, tname, sentence, msg, {}))
        new_fix = False

        if isinstance(msg, pynmea2.types.talker.GGA):
            raw_qual = getattr(msg, "gps_qual", 0)
            try:
                gq = int(raw_qual)
            except Exception:
                gq = 0
            is_dgps = (gq == 2) or (gq in (4, 5))
            lat, lon, alt = safe_float(msg.latitude), safe_float(msg.longitude), safe_float(msg.altitude)
            hdop = safe_float(getattr(msg, "horizontal_dil", None))

            if lat is not None and lon is not None:
                df = DataFix(ts_iso, lat, lon, alt, is_dgps, hdop)
                
                # Snapshot dynamic IMU state directly onto the GGA point
                if self.latest_imu:
                    df.imu_metrics = dict(self.latest_imu)
                
                self.fix_buffer.append(df)
                self._fix_times.append(datetime.now(timezone.utc))
                self._append_sensor_record(df)
                new_fix = True

            self.hdop = hdop
            self.fix_mode = {0: "No Fix", 1: "GPS 3D", 2: "DGPS", 4: "RTK Fix", 5: "RTK Float"}.get(gq, f"Fix {gq}")
            self.latitude, self.longitude, self.altitude = lat, lon, alt

            # Epoch Merger Entry
            self.epoch_map[ts_iso] = {
                "ts": ts_iso,
                "gga": msg,
                "gsa": None,
                "gsv_parts": {},
                "rmc": None,
                "imu": dict(self.latest_imu) if self.latest_imu else {},
                "created_at": time.time(),
                "written": False
            }

        elif isinstance(msg, pynmea2.types.talker.GSA):
            self.hdop = safe_float(getattr(msg, "hdop", None))
            self.vdop = safe_float(getattr(msg, "vdop", None))
            self.pdop = safe_float(getattr(msg, "pdop", None))
            self.satellites_in_use.clear()
            for i in range(1, 13):
                sv = getattr(msg, f"sv_id_{i:02d}", None)
                if sv:
                    try:
                        pr = int(sv)
                        self.satellites_in_use.add(pr)
                        self.satellites.setdefault(pr, Satellite(pr))
                    except Exception:
                        pass

        elif isinstance(msg, pynmea2.types.talker.GSV):
            for i in range(1, 5):
                prn = getattr(msg, f"sv_prn_num_{i}", None)
                snr = getattr(msg, f"snr_{i}", None)
                if prn:
                    try:
                        prn_i = int(prn)
                        sat = self.satellites.setdefault(prn_i, Satellite(prn_i))
                        if snr:
                            sat.snr = safe_float(snr)
                    except Exception:
                        continue

        if new_fix:
            self._recompute_center()
            self._run_pipelines(self.fix_buffer[-1])

    def _recompute_center(self):
        buf = [f for f in self.fix_buffer if f.lat is not None and f.lon is not None]
        if not buf:
            return
        cutoff = datetime.now(timezone.utc) - timedelta(seconds=self.window_s)
        windowed = [f for f in buf if datetime.fromisoformat(f.ts) >= cutoff] or buf
        self.center_lat = sum(f.lat for f in windowed) / len(windowed)
        self.center_lon = sum(f.lon for f in windowed) / len(windowed)

    def _run_pipelines(self, fix):
        for i, p in enumerate(self.pipelines):
            if not p['enabled']:
                continue
            m = p['method']
            st = p.setdefault('state', {})
            out = None
            if m == "Raw":
                out = fix
            elif m == "EMA":
                out = stage_EMA(st, fix, p['params'].get('alpha', 0.3))
            elif m == "SMA":
                out = stage_SMA(st, fix, p['params'].get('window', 5))
            elif m == "Hampel":
                out = stage_HAMPEL(st, fix, p['params'].get('window', 7), p['params'].get('k', 3.0))
            elif m == "KF_ENU":
                out = stage_KF_ENU(st, fix, self.center_lat, self.center_lon)
            if out:
                buf = p.setdefault('output', [])
                buf.append(out)
                if len(buf) > 12000:
                    buf.pop(0)

    def ui_tick(self):
        if self.center_lat and self.center_lon:
            self.sky_plot.set_center(self.center_lat, self.center_lon)
        for i, p in enumerate(self.pipelines):
            self.sky_plot.set_pipeline_points(p['name'], p['output'])
            self.sky_plot.set_pipeline_color(p['name'], p['color'])

        # Update displays
        if self.latitude and self.longitude:
            self.lat_label.setText(f"Lat: {self.latitude:.7f}°")
            self.lon_label.setText(f"Lon: {self.longitude:.7f}°")
            self.alt_label.setText(f"Alt: {self.altitude:.2f} m" if self.altitude else "Alt: --")
        self.fix_label.setText(f"Fix: {self.fix_mode}")
        self.hdop_label.setText(f"HDOP: {self.hdop:.2f}" if self.hdop else "HDOP: --")
        self.sat_label.setText(f"Sats: {len(self.satellites_in_use)} used / {len(self.satellites)} in view")
        
        # Display dynamic IMU state
        if self.latest_imu:
            im = self.latest_imu
            self.imu_att_label.setText(f"Roll: {im['roll']:.1f}° | Pitch: {im['pitch']:.1f}°")
            self.imu_acc_label.setText(f"Accel: [{im['ax']:.1f}, {im['ay']:.1f}, {im['az']:.1f}]")
            self.imu_vib_label.setText(f"Vib Index: {im['vib']:.2f} g")

        self.signal_widget.update_satellites(list(self.satellites.values()))
        self._check_epoch_timeouts()

    def _check_epoch_timeouts(self):
        now = time.time()
        for epoch_id, epoch in list(self.epoch_map.items()):
            if epoch.get('written'):
                del self.epoch_map[epoch_id]
                continue
            if now - epoch.get('created_at', now) >= self.epoch_timeout_s:
                self._write_epoch_row(epoch_id)

    def _build_stream_csv_columns(self):
        cols = []
        for name, _ in self.available_sentences:
            sel = [(fld, lbl) for fld, lbl in SENTENCE_FIELDS.get(name, []) if self.sentence_field_config[name].get(fld, {}).get('csv')]
            for fld, lbl in sel:
                cols.append((name, fld, lbl))
        return cols

    def start_streaming_csv(self):
        if self.streaming_csv:
            return
        cols = self._build_stream_csv_columns()
        if not cols:
            QMessageBox.warning(self, "CSV", "No fields selected. Click 'Quick Features' or configure fields.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Start Flight CSV", f"flight_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}.csv", "CSV (*.csv)")
        if not path:
            return
        self.streaming_csv = StreamingCSVWriter(path, [h for (_, _, h) in cols])
        self.streaming_csv_cols = cols
        self.statusBar().showMessage(f"Streaming CSV live to: {path}")

    def stop_streaming_csv(self):
        if self.streaming_csv:
            self.streaming_csv.close()
            self.streaming_csv = None
            self.statusBar().showMessage("Streaming CSV stopped.")

    def _write_epoch_row(self, epoch_id):
        if not self.streaming_csv or not self.streaming_csv_cols:
            return
        epoch = self.epoch_map.get(epoch_id)
        if not epoch:
            return
        df_fix = next((f for f in reversed(self.fix_buffer) if f.ts == epoch.get('ts')), None)
        im = epoch.get('imu', {})
        row = []
        for sent, fld, _ in self.streaming_csv_cols:
            val = ""
            if sent == 'GGA':
                if fld == 'utc_time': val = epoch.get('ts', '')
                elif fld == 'latitude' and df_fix: val = f"{df_fix.lat:.8f}" if df_fix.lat else ""
                elif fld == 'longitude' and df_fix: val = f"{df_fix.lon:.8f}" if df_fix.lon else ""
                elif fld == 'altitude' and df_fix: val = f"{df_fix.alt:.2f}" if df_fix.alt else ""
                elif fld == 'gps_qual': val = self.fix_mode
                elif fld == 'horizontal_dil' and df_fix: val = f"{df_fix.hdop:.2f}" if df_fix.hdop else ""
                elif fld.startswith('imu_'):
                    metric_key = fld.replace('imu_', '')
                    val = f"{im.get(metric_key, ''):.3f}" if metric_key in im else ""
            row.append(val)
        self.streaming_csv.write_row(row)
        epoch['written'] = True
        try:
            del self.epoch_map[epoch_id]
        except Exception:
            pass

    def save_pipelines_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save Flight Snapshot", f"flight_snapshot_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}.csv", "CSV (*.csv)")
        if not path:
            return
        cols = self._build_stream_csv_columns()
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow([h for (_, _, h) in cols])
                for fix in self.fix_buffer:
                    im = fix.imu_metrics
                    row = []
                    for sent, fld, _ in cols:
                        val = ""
                        if sent == 'GGA':
                            if fld == 'utc_time': val = fix.ts
                            elif fld == 'latitude': val = f"{fix.lat:.8f}" if fix.lat else ""
                            elif fld == 'longitude': val = f"{fix.lon:.8f}" if fix.lon else ""
                            elif fld == 'altitude': val = f"{fix.alt:.2f}" if fix.alt else ""
                            elif fld == 'horizontal_dil': val = f"{fix.hdop:.2f}" if fix.hdop else ""
                            elif fld.startswith('imu_'):
                                metric = fld.replace('imu_', '')
                                val = f"{im.get(metric, ''):.3f}" if metric in im else ""
                        row.append(val)
                    writer.writerow(row)
            QMessageBox.information(self, "Export", f"Merged dataset saved successfully to {path}")
        except Exception as e:
            QMessageBox.critical(self, "Export Error", str(e))

    def save_sensor_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save GPS + IMU CSV", f"gps_imu_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}.csv", "CSV (*.csv)")
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(self.sensor_csv_columns)
                for row in self.sensor_log:
                    writer.writerow([row.get(col, '') for col in self.sensor_csv_columns])
            QMessageBox.information(self, "GPS + IMU Export", f"GPS and IMU dataset saved successfully to {path}")
        except Exception as e:
            QMessageBox.critical(self, "Export Error", str(e))

    def show_data_table(self):
        dlg = QDialog(self)
        dlg.setWindowTitle("Live Flight Data Matrix")
        dlg.resize(1100, 500)
        lay = QVBoxLayout(dlg)
        tab = QTabWidget(dlg)
        p_table = QTableWidget()
        cols = ["UTC Time", "Lat", "Lon", "Alt", "HDOP", "Roll", "Pitch", "Vibration"]
        p_table.setColumnCount(len(cols))
        p_table.setHorizontalHeaderLabels(cols)
        p_table.setRowCount(min(len(self.fix_buffer), 100))
        for r, fix in enumerate(list(self.fix_buffer)[-100:]):
            im = fix.imu_metrics
            p_table.setItem(r, 0, QTableWidgetItem(str(fix.ts)))
            p_table.setItem(r, 1, QTableWidgetItem(f"{fix.lat:.7f}" if fix.lat else ""))
            p_table.setItem(r, 2, QTableWidgetItem(f"{fix.lon:.7f}" if fix.lon else ""))
            p_table.setItem(r, 3, QTableWidgetItem(f"{fix.alt:.2f}" if fix.alt else ""))
            p_table.setItem(r, 4, QTableWidgetItem(f"{fix.hdop:.2f}" if fix.hdop else ""))
            p_table.setItem(r, 5, QTableWidgetItem(f"{im.get('roll', 0.0):.1f}°"))
            p_table.setItem(r, 6, QTableWidgetItem(f"{im.get('pitch', 0.0):.1f}°"))
            p_table.setItem(r, 7, QTableWidgetItem(f"{im.get('vib', 0.0):.2f} g"))
        lay.addWidget(p_table)
        dlg.exec_()

    def closeEvent(self, event):
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()
        if self.imu_thread and self.imu_thread.isRunning():
            self.imu_thread.stop()
        if self.streaming_csv:
            self.streaming_csv.close()
        if self.raw_log_writer:
            self.raw_log_writer.close()
        event.accept()

def main():
    app = QApplication(sys.argv)
    w = GNSSMonitor()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()