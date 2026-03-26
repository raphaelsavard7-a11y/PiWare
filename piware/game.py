#!/usr/bin/env python3
"""
PiWare — Retro arcade microgame collection for ESP32 + Raspberry Pi.

Hardware: 2 buttons, 2 potentiometers, 3 LEDs on ESP32 (Lilygo T-SIM7670E)
Display:  720x1280 portrait touchscreen on Raspberry Pi 5
Touch:    evdev (/dev/input)
Render:   SDL2 kmsdrm (headless, no X11)

Communication modes (selected from in-game menu):
  - Serial USB  (reuses testscreen_serial ESP32 sketch)
  - MQTT WiFi   (reuses testscreen_mqtt_wifi ESP32 sketch)
  - MQTT LTE    (reuses testscreen_mqtt ESP32 sketch)

Usage (from SSH):
  sudo python3 game.py [--port /dev/ttyACM0]
"""

import os
import sys
import abc
import json
import math
import time
import random
import argparse
import threading

# Configure SDL for framebuffer before importing pygame
os.environ["SDL_VIDEODRIVER"] = "kmsdrm"
os.environ.setdefault("SDL_FBDEV", "/dev/fb0")

import pygame
from evdev import InputDevice, ecodes, list_devices

# ═══════════════════════════════════════════════════════════
#  CONSTANTS
# ═══════════════════════════════════════════════════════════
SCREEN_W, SCREEN_H = 720, 1280
PIXEL_SCALE = 3
VIRT_W, VIRT_H = SCREEN_W // PIXEL_SCALE, SCREEN_H // PIXEL_SCALE  # 240x426
FPS = 30
HIGHSCORE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "highscores.json")
CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "calibration.json")

# ── Wii-Inspired Palette ─────────────────────────────────
BG          = (230, 235, 240)
WHITE       = (255, 255, 255)
BLACK       = (0, 0, 0)
GREY        = (160, 165, 170)
DARK_GREY   = (90, 95, 100)
RED         = (220, 60, 60)
GREEN       = (80, 190, 80)
BLUE        = (50, 120, 255)
YELLOW      = (240, 200, 40)
ORANGE      = (240, 150, 40)
PURPLE      = (140, 80, 200)
CYAN        = (0, 170, 210)
PINK        = (230, 90, 160)
NEON_GREEN  = (80, 200, 80)
NEON_BLUE   = (70, 130, 230)
MENU_BG     = (235, 240, 245)

# ── Wii Theme Colors ────────────────────────────────────
WII_BLUE       = (0, 154, 199)
WII_LIGHT_BLUE = (140, 210, 240)
WII_PANEL      = (245, 247, 250)
WII_BORDER     = (200, 205, 215)
WII_TEXT       = (60, 65, 75)
WII_TEXT_LIGHT = (130, 135, 145)
WII_SUCCESS    = (80, 190, 80)
WII_DANGER     = (220, 60, 60)
WII_GOLD       = (220, 180, 40)
WII_SHADOW     = (0, 0, 0, 30)


# ═══════════════════════════════════════════════════════════
#  GRADIENT SURFACE CACHE (scanline fix)
# ═══════════════════════════════════════════════════════════
_gradient_cache = {}


def make_gradient_surface(w, h, top, bottom, key=None):
    """Pre-render a vertical gradient to a Surface. Cached by key or colors."""
    cache_key = key or (w, h, top, bottom)
    if cache_key in _gradient_cache:
        return _gradient_cache[cache_key]
    surf = pygame.Surface((w, h))
    # Draw in 2-pixel-tall bands with subtle dithering to prevent banding
    for y in range(0, h, 2):
        t = y / max(1, h - 1)
        r = int(top[0] + (bottom[0] - top[0]) * t)
        g = int(top[1] + (bottom[1] - top[1]) * t)
        b = int(top[2] + (bottom[2] - top[2]) * t)
        # Subtle dither: alternate rows get +1/-1
        d = 1 if (y // 2) % 2 == 0 else -1
        c1 = (max(0, min(255, r + d)), max(0, min(255, g + d)),
              max(0, min(255, b + d)))
        c2 = (max(0, min(255, r - d)), max(0, min(255, g - d)),
              max(0, min(255, b - d)))
        pygame.draw.rect(surf, c1, (0, y, w, 1))
        if y + 1 < h:
            pygame.draw.rect(surf, c2, (0, y + 1, w, 1))
    _gradient_cache[cache_key] = surf
    return surf


def clear_gradient_cache():
    """Call when the display mode changes."""
    _gradient_cache.clear()


# ═══════════════════════════════════════════════════════════
#  HARDWARE STATE
# ═══════════════════════════════════════════════════════════
class HardwareState:
    """Thread-safe container for ESP32 sensor readings."""
    def __init__(self):
        self.btn1 = False
        self.btn2 = False
        self.pot1 = 0
        self.pot2 = 0
        self.led_r = False
        self.led_g = False
        self.led_b = False
        self.acX = 0.0
        self.acY = 0.0
        self.acZ = 0.0
        self.gyX = 0.0
        self.gyY = 0.0
        self.gyZ = 0.0
        self.connected = False
        self.btn1_presses = 0
        self.btn2_presses = 0
        self._prev_btn1 = False
        self._prev_btn2 = False
        self.lock = threading.Lock()
        self._imu_ac_scale = None
        self._imu_gy_scale = None
        # Calibration offsets (loaded from file or computed)
        self.cal_acX = 0.0
        self.cal_acY = 0.0
        self.cal_acZ = 0.0
        self.cal_gyX = 0.0
        self.cal_gyY = 0.0
        self.cal_gyZ = 0.0

    def _detect_edges(self):
        if self.btn1 != self._prev_btn1:
            self.btn1_presses += 1
        if self.btn2 != self._prev_btn2:
            self.btn2_presses += 1
        self._prev_btn1 = self.btn1
        self._prev_btn2 = self.btn2

    def _detect_imu_scale(self):
        max_ac = max(abs(self.acX), abs(self.acY), abs(self.acZ))
        max_gy = max(abs(self.gyX), abs(self.gyY), abs(self.gyZ))
        if max_ac > 1000:
            self._imu_ac_scale = 1.0 / 16384.0
        elif max_ac > 5:
            self._imu_ac_scale = 1.0 / 9.81
        else:
            self._imu_ac_scale = 1.0
        if max_gy > 1000:
            self._imu_gy_scale = 1.0 / 131.0
        elif max_gy > 10:
            self._imu_gy_scale = 1.0
        else:
            self._imu_gy_scale = 57.3

    def snapshot(self):
        with self.lock:
            if self._imu_ac_scale is None:
                max_ac = max(abs(self.acX), abs(self.acY), abs(self.acZ))
                if max_ac > 0.01:
                    self._detect_imu_scale()
            ac_s = self._imu_ac_scale or 1.0
            gy_s = self._imu_gy_scale or 1.0
            return {
                "btn1": self.btn1, "btn2": self.btn2,
                "pot1": self.pot1, "pot2": self.pot2,
                "led_r": self.led_r, "led_g": self.led_g, "led_b": self.led_b,
                "acX": self.acX * ac_s - self.cal_acX,
                "acY": self.acY * ac_s - self.cal_acY,
                "acZ": self.acZ * ac_s - self.cal_acZ,
                "gyX": self.gyX * gy_s - self.cal_gyX,
                "gyY": self.gyY * gy_s - self.cal_gyY,
                "gyZ": self.gyZ * gy_s - self.cal_gyZ,
                "connected": self.connected,
                "btn1_presses": self.btn1_presses,
                "btn2_presses": self.btn2_presses,
            }

    def snapshot_raw(self):
        """Raw snapshot WITHOUT calibration offsets, for calibration routine."""
        with self.lock:
            if self._imu_ac_scale is None:
                max_ac = max(abs(self.acX), abs(self.acY), abs(self.acZ))
                if max_ac > 0.01:
                    self._detect_imu_scale()
            ac_s = self._imu_ac_scale or 1.0
            gy_s = self._imu_gy_scale or 1.0
            return {
                "acX": self.acX * ac_s, "acY": self.acY * ac_s,
                "acZ": self.acZ * ac_s,
                "gyX": self.gyX * gy_s, "gyY": self.gyY * gy_s,
                "gyZ": self.gyZ * gy_s,
                "connected": self.connected,
            }

    def reset_press_count(self, btn):
        with self.lock:
            if btn == 1:
                self.btn1_presses = 0
            else:
                self.btn2_presses = 0


# ═══════════════════════════════════════════════════════════
#  CALIBRATION
# ══════════���════════════════════════════════════════════════
def load_calibration():
    try:
        with open(CALIBRATION_FILE, "r") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def save_calibration(cal):
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(cal, f, indent=2)


def apply_calibration(state, cal):
    """Apply calibration dict to HardwareState offsets."""
    if cal:
        state.cal_acX = cal.get("acX", 0.0)
        state.cal_acY = cal.get("acY", 0.0)
        state.cal_acZ = cal.get("acZ", 0.0)
        state.cal_gyX = cal.get("gyX", 0.0)
        state.cal_gyY = cal.get("gyY", 0.0)
        state.cal_gyZ = cal.get("gyZ", 0.0)


# ═══════════════════════════════════════════════════════════
#  COMMUNICATION BACKENDS
# ═══════════════════════════════════════════════════════════
class CommBackend(abc.ABC):
    def __init__(self, state):
        self.state = state

    @abc.abstractmethod
    def start(self): ...

    @abc.abstractmethod
    def set_led(self, color, on): ...

    @abc.abstractmethod
    def stop(self): ...


class SerialBackend(CommBackend):
    def __init__(self, state, port="/dev/ttyACM0"):
        super().__init__(state)
        self.port = port
        self.ser = None

    def start(self):
        import serial as _serial
        self._serial_mod = _serial
        t = threading.Thread(target=self._read_loop, daemon=True)
        t.start()

    def _read_loop(self):
        while True:
            try:
                self.ser = self._serial_mod.Serial(self.port, 115200, timeout=1)
                while True:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line.startswith("S,"):
                        parts = line[2:].split(",")
                        if len(parts) >= 7:
                            with self.state.lock:
                                self.state.btn1  = parts[0] == "1"
                                self.state.btn2  = parts[1] == "1"
                                self.state.pot1  = int(parts[2])
                                self.state.pot2  = int(parts[3])
                                self.state.led_r = parts[4] == "1"
                                self.state.led_g = parts[5] == "1"
                                self.state.led_b = parts[6] == "1"
                                if len(parts) >= 13:
                                    self.state.acX = float(parts[7])
                                    self.state.acY = float(parts[8])
                                    self.state.acZ = float(parts[9])
                                    self.state.gyX = float(parts[10])
                                    self.state.gyY = float(parts[11])
                                    self.state.gyZ = float(parts[12])
                                self.state.connected = True
                                self.state._detect_edges()
            except Exception:
                with self.state.lock:
                    self.state.connected = False
                self.ser = None
                time.sleep(2)

    def set_led(self, color, on):
        if not self.ser:
            return
        key = {"r": "led_r", "g": "led_g", "b": "led_b"}[color]
        current = getattr(self.state, key)
        if current != on:
            cmd = color.upper()
            try:
                self.ser.write(cmd.encode())
            except Exception:
                pass

    def stop(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass


class MQTTBackend(CommBackend):
    def __init__(self, state, mode="wifi"):
        super().__init__(state)
        self.mqtt = None
        self.mode = mode

    def _load_mqtt_config(self):
        import importlib.util
        script_dir = os.path.dirname(os.path.abspath(__file__))
        if self.mode == "wifi":
            preferred = ["../testscreen_mqtt_wifi", "../testscreen_mqtt", "."]
        else:
            preferred = ["../testscreen_mqtt", "../testscreen_mqtt_wifi", "."]
        for subdir in preferred:
            cfg_path = os.path.join(script_dir, subdir, "mqtt_config.py")
            if os.path.exists(cfg_path):
                spec = importlib.util.spec_from_file_location("mqtt_config", cfg_path)
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                return mod.MQTT_CONFIG
        raise FileNotFoundError(
            "mqtt_config.py not found.  Copy it from testscreen_mqtt/.")

    def start(self):
        import paho.mqtt.client as paho_mqtt
        import ssl
        cfg = self._load_mqtt_config()
        self.device_id = cfg["device_id"]
        self.status_topic = f"{self.device_id}/testscreen/status"
        self.led_topics = {
            "r": f"{self.device_id}/testscreen/led/red",
            "g": f"{self.device_id}/testscreen/led/green",
            "b": f"{self.device_id}/testscreen/led/blue",
        }
        client_id = f"pi-piware-{int(time.time())}"
        port = cfg.get("port", 1883)
        if port == 443:
            self.mqtt = paho_mqtt.Client(client_id=client_id, transport="websockets")
            self.mqtt.tls_set(cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLS)
            self.mqtt.ws_set_options(path="/mqtt")
        else:
            self.mqtt = paho_mqtt.Client(client_id=client_id)
        self.mqtt.username_pw_set(cfg["username"], cfg["password"])

        def on_connect(client, _ud, _flags, rc):
            if rc == 0:
                client.subscribe(self.status_topic)

        def on_message(_client, _ud, msg):
            try:
                data = json.loads(msg.payload.decode())
                with self.state.lock:
                    self.state.btn1  = bool(data.get("btn1", 0))
                    self.state.btn2  = bool(data.get("btn2", 0))
                    self.state.pot1  = int(data.get("pot1", 0))
                    self.state.pot2  = int(data.get("pot2", 0))
                    self.state.led_r = bool(data.get("ledR", 0))
                    self.state.led_g = bool(data.get("ledG", 0))
                    self.state.led_b = bool(data.get("ledB", 0))
                    self.state.acX   = float(data.get("acX", 0))
                    self.state.acY   = float(data.get("acY", 0))
                    self.state.acZ   = float(data.get("acZ", 0))
                    self.state.gyX   = float(data.get("gyX", 0))
                    self.state.gyY   = float(data.get("gyY", 0))
                    self.state.gyZ   = float(data.get("gyZ", 0))
                    self.state.connected = True
                    self.state._detect_edges()
            except Exception:
                pass

        self.mqtt.on_connect = on_connect
        self.mqtt.on_message = on_message
        self.mqtt.connect(cfg["broker"], cfg["port"], 60)
        self.mqtt.loop_start()

    def set_led(self, color, on):
        if self.mqtt:
            topic = self.led_topics.get(color)
            if topic:
                self.mqtt.publish(topic, "ON" if on else "OFF")

    def stop(self):
        if self.mqtt:
            self.mqtt.loop_stop()
            self.mqtt.disconnect()


# ═══════════════════════════════════════════════════════════
#  TOUCH INPUT
# ═══════════════════════════════════════════════════════════
def find_touch_device():
    for path in list_devices():
        dev = InputDevice(path)
        if "touch" in dev.name.lower() or "ft5406" in dev.name.lower():
            return dev
    return None


def touch_reader(dev, queue):
    x, y = 0, 0
    for ev in dev.read_loop():
        if ev.type == ecodes.EV_ABS:
            if ev.code in (ecodes.ABS_MT_POSITION_X, ecodes.ABS_X):
                x = ev.value
            elif ev.code in (ecodes.ABS_MT_POSITION_Y, ecodes.ABS_Y):
                y = ev.value
        elif (ev.type == ecodes.EV_KEY
              and ev.code == ecodes.BTN_TOUCH
              and ev.value == 1):
            queue.append((x, y))


# ═══════════════════════════════════════════════════════════
#  HELPERS
# ═══════════════════════════════════════════════════════════
def draw_rounded_rect(surface, colour, rect, radius=8):
    pygame.draw.rect(surface, colour, rect, border_radius=radius)


def draw_wii_panel(surface, rect, border_color=None, shadow=True):
    x, y, w, h = rect
    if shadow:
        shadow_rect = (x + 2, y + 2, w, h)
        pygame.draw.rect(surface, (180, 185, 195), shadow_rect, border_radius=10)
    pygame.draw.rect(surface, WII_PANEL, rect, border_radius=10)
    if border_color:
        pygame.draw.rect(surface, border_color, rect, width=2, border_radius=10)


def draw_wii_button(surface, rect, text, font, color=WII_BLUE,
                     text_color=WHITE, selected=False):
    x, y, w, h = rect
    pygame.draw.rect(surface, (180, 185, 195), (x + 1, y + 2, w, h),
                     border_radius=8)
    bg = color if selected else WII_PANEL
    tc = WHITE if selected else color
    pygame.draw.rect(surface, bg, rect, border_radius=8)
    pygame.draw.rect(surface, color, rect, width=2, border_radius=8)
    txt = font.render(text, True, tc)
    surface.blit(txt, (x + w // 2 - txt.get_width() // 2,
                       y + h // 2 - txt.get_height() // 2))


def draw_pixel_border(surface, rect, color, thickness=1):
    pygame.draw.rect(surface, color, rect, width=thickness, border_radius=4)


def draw_glow_text(surface, text, font, color, pos, glow_color=None,
                   glow_radius=1):
    if glow_color is None:
        glow_color = tuple(min(255, c // 2 + 30) for c in color)
    if glow_radius >= 1:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                glow = font.render(text, True, glow_color)
                glow.set_alpha(60)
                surface.blit(glow, (pos[0] + dx, pos[1] + dy))
    txt = font.render(text, True, color)
    surface.blit(txt, pos)


def draw_clean_text(surface, text, font, color, pos, center=False):
    txt = font.render(text, True, color)
    if center:
        surface.blit(txt, (pos[0] - txt.get_width() // 2,
                           pos[1] - txt.get_height() // 2))
    else:
        surface.blit(txt, pos)


def draw_wii_bg(surface, top=(242, 247, 252), bottom=(218, 228, 238)):
    """Blit a cached gradient surface — no scanline artifacts."""
    grad = make_gradient_surface(VIRT_W, VIRT_H, top, bottom)
    surface.blit(grad, (0, 0))


def draw_wii_header(surface, text, font, color=WII_BLUE):
    grad = make_gradient_surface(VIRT_W, 44,
                                 (248, 250, 253), (236, 238, 245),
                                 key=("header_grad",))
    surface.blit(grad, (0, 0))
    pygame.draw.line(surface, WII_BORDER, (0, 44), (VIRT_W, 44), 1)
    txt = font.render(text, True, color)
    surface.blit(txt, (VIRT_W // 2 - txt.get_width() // 2,
                       22 - txt.get_height() // 2))


def draw_wii_progress_bar(surface, x, y, w, h, pct, color=WII_BLUE,
                           bg=(210, 215, 225)):
    pct = max(0.0, min(1.0, pct))
    rad = h // 2
    pygame.draw.rect(surface, bg, (x, y, w, h), border_radius=rad)
    fill_w = int(w * pct)
    if fill_w > 2:
        pygame.draw.rect(surface, color, (x, y, fill_w, h), border_radius=rad)
        shine_h = max(1, h // 3)
        shine_c = (min(255, color[0] + 55), min(255, color[1] + 55),
                   min(255, color[2] + 55))
        pygame.draw.rect(surface, shine_c, (x + 1, y + 1, fill_w - 2, shine_h),
                         border_radius=rad)
    pygame.draw.rect(surface, (180, 185, 195), (x, y, w, h), width=1,
                     border_radius=rad)


def draw_wii_badge(surface, cx, cy, text, font, active=False, color=WII_BLUE):
    tw, th = font.size(text)
    bw, bh = tw + 14, th + 6
    bx, by = cx - bw // 2, cy - bh // 2
    if active:
        pygame.draw.rect(surface, color, (bx, by, bw, bh),
                         border_radius=bh // 2)
        txt = font.render(text, True, WHITE)
    else:
        pygame.draw.rect(surface, WII_PANEL, (bx, by, bw, bh),
                         border_radius=bh // 2)
        pygame.draw.rect(surface, WII_BORDER, (bx, by, bw, bh), width=1,
                         border_radius=bh // 2)
        txt = font.render(text, True, WII_TEXT_LIGHT)
    surface.blit(txt, (cx - txt.get_width() // 2, cy - txt.get_height() // 2))


def draw_wii_circle_meter(surface, cx, cy, radius, pct, color=WII_BLUE,
                           bg=None, width=4):
    if bg is None:
        bg = (210, 215, 225)
    pygame.draw.circle(surface, bg, (cx, cy), radius, width)
    if pct > 0:
        end_angle = int(pct * 360)
        for a_deg in range(0, end_angle, 2):
            a = math.radians(a_deg - 90)
            px = cx + int(math.cos(a) * radius)
            py = cy + int(math.sin(a) * radius)
            pygame.draw.circle(surface, color, (px, py), width // 2)


def draw_mii(surface, cx, cy, size=16, happy=True, skin=(255, 222, 186)):
    pygame.draw.circle(surface, skin, (cx, cy), size)
    pygame.draw.circle(surface, (min(255, skin[0] + 15), min(255, skin[1] + 15),
                                  min(255, skin[2] + 15)),
                       (cx - size // 3, cy - size // 3), size // 3)
    ed = size // 3
    ey = cy - size // 5
    pygame.draw.circle(surface, (40, 42, 48), (cx - ed, ey), max(1, size // 8))
    pygame.draw.circle(surface, (40, 42, 48), (cx + ed, ey), max(1, size // 8))
    my = cy + size // 4
    if happy:
        pygame.draw.arc(surface, (40, 42, 48),
                        (cx - size // 4, my - 2, size // 2, size // 3),
                        3.14, 6.28, 1)
    else:
        pygame.draw.arc(surface, (40, 42, 48),
                        (cx - size // 4, my, size // 2, size // 3),
                        0, 3.14, 1)


def draw_wii_result_panel(surface, text, font, won, t):
    cx, cy = VIRT_W // 2, VIRT_H // 2
    panel_w = min(int(t * 600), 180)
    panel_h = min(int(t * 400), 100)
    if panel_w > 20 and panel_h > 10:
        draw_wii_panel(surface, (cx - panel_w // 2, cy - panel_h // 2,
                                  panel_w, panel_h),
                       border_color=WII_SUCCESS if won else WII_DANGER)
    scale = min(t / 0.2, 1.0)
    if won:
        if scale > 0.1:
            pts = [(cx - int(10 * scale), cy),
                   (cx - int(2 * scale), cy + int(10 * scale)),
                   (cx + int(14 * scale), cy - int(10 * scale))]
            pygame.draw.lines(surface, WII_SUCCESS, False, pts, 3)
        for i in range(3):
            r = int((t * 100 + i * 20) % 80)
            if 0 < r < 80:
                alpha_pct = max(0, 1.0 - r / 80)
                c = (int(130 * alpha_pct), int(220 * alpha_pct),
                     int(130 * alpha_pct))
                pygame.draw.circle(surface, c, (cx, cy), r, 2)
        txt = font.render(text, True, WII_SUCCESS)
    else:
        if scale > 0.1:
            xs = int(12 * scale)
            pygame.draw.line(surface, WII_DANGER,
                             (cx - xs, cy - xs), (cx + xs, cy + xs), 3)
            pygame.draw.line(surface, WII_DANGER,
                             (cx + xs, cy - xs), (cx - xs, cy + xs), 3)
        txt = font.render(text, True, WII_DANGER)
    if t > 0.15:
        surface.blit(txt, (cx - txt.get_width() // 2, cy - 40))


# ── High scores ──────────────────────────────────────────
def load_highscores():
    try:
        with open(HIGHSCORE_FILE, "r") as f:
            data = json.load(f)
            return data.get("scores", [])[:5]
    except (FileNotFoundError, json.JSONDecodeError):
        return []


def save_highscores(scores):
    scores = sorted(scores, reverse=True)[:5]
    with open(HIGHSCORE_FILE, "w") as f:
        json.dump({"scores": scores}, f)


def is_highscore(score):
    scores = load_highscores()
    return len(scores) < 5 or score > min(scores)


# ═══════════════════════════════════════════════════════════
#  MICROGAME BASE
# ═══════════════════════════════════════════════════════════
class MicroGame(abc.ABC):
    name = ""
    hint = ""
    instruction = ""
    base_duration = 4.0
    game_type = "action"
    wave_based = False
    bg_color = BG
    RESULT_DURATION = 0.7

    def reset(self, snap=None, speed_mult=1.0):
        pass

    @abc.abstractmethod
    def update(self, snap, dt):
        ...

    @abc.abstractmethod
    def draw(self, screen, snap, time_left, time_total, fonts):
        ...

    def animate_result(self, screen, won, t, fonts):
        draw_wii_bg(screen)
        cx, cy = VIRT_W // 2, VIRT_H // 2
        draw_wii_result_panel(screen,
                              "NICE!" if won else "MISS!",
                              fonts["huge"], won, t)


# ═══════════════════════════════════════════════════════════
#  NEW MICROGAMES
# ═══════════════════════════════════════════════════════════

# ── 1. SKEE-BALL ──────────────────────────────────────────
class SkeeBallGame(MicroGame):
    name = "SKEE-BALL!"
    hint = "POT + BTN"
    instruction = "Aim & throw!"
    base_duration = 4.5
    game_type = "action"

    def reset(self, snap=None, speed_mult=1.0):
        self.pot = random.choice([1, 2])
        self.launched = False
        self.ball_x = 0.0
        self.ball_y = 0.0
        self.ball_vx = 0.0
        self.ball_vy = 0.0
        self.aim_angle = 0.5
        self.t_since_launch = 0.0
        # Single target — hit it to win, miss to lose
        self.target_cx = VIRT_W // 2
        self.target_cy = 80
        self.target_r = max(8, int(14 * speed_mult))
        self.hint = f"POT {self.pot}"

    def update(self, snap, dt):
        if not self.launched:
            self.aim_angle = snap[f"pot{self.pot}"] / 4095.0
            if snap["btn1"] or snap["btn2"]:
                self.launched = True
                cx = VIRT_W // 2
                self.ball_x = float(cx)
                self.ball_y = float(VIRT_H - 60)
                spread = (self.aim_angle - 0.5) * 60
                self.ball_vx = spread
                self.ball_vy = -280.0
            return None

        self.t_since_launch += dt
        self.ball_x += self.ball_vx * dt
        self.ball_y += self.ball_vy * dt
        self.ball_vy += 80 * dt  # slight gravity arc

        # Check target hit
        dist = math.sqrt((self.ball_x - self.target_cx) ** 2 +
                         (self.ball_y - self.target_cy) ** 2)
        if dist < self.target_r + 4:
            return True

        if self.ball_y < 40 or self.t_since_launch > 2.5:
            return False
        return None

    def draw(self, screen, snap, tl, tt, fonts):
        # Ramp background
        draw_wii_bg(screen, top=(200, 180, 160), bottom=(240, 230, 220))
        draw_wii_header(screen, "SKEE-BALL!", fonts["big"], color=ORANGE)
        cx = VIRT_W // 2

        # Lane
        pygame.draw.rect(screen, (170, 150, 130),
                         (cx - 40, 70, 80, VIRT_H - 120), border_radius=6)
        pygame.draw.rect(screen, (150, 130, 110),
                         (cx - 40, 70, 80, VIRT_H - 120), width=2,
                         border_radius=6)
        # Lane lines
        for ly in range(90, VIRT_H - 60, 20):
            pygame.draw.line(screen, (160, 140, 120),
                             (cx - 30, ly), (cx + 30, ly), 1)

        # Target (bullseye)
        tx, ty, tr = self.target_cx, self.target_cy, self.target_r
        pygame.draw.circle(screen, WII_GOLD, (tx, ty), tr)
        pygame.draw.circle(screen, WHITE, (tx, ty), max(2, tr // 2))
        pygame.draw.circle(screen, WII_BORDER, (tx, ty), tr, 2)

        # Aim indicator
        if not self.launched:
            aim_x = cx + int((self.aim_angle - 0.5) * 60)
            pygame.draw.polygon(screen, WII_DANGER, [
                (aim_x, VIRT_H - 75), (aim_x - 4, VIRT_H - 65),
                (aim_x + 4, VIRT_H - 65)])
            # Ball waiting at bottom
            pygame.draw.circle(screen, WHITE, (cx, VIRT_H - 55), 6)
            pygame.draw.circle(screen, WII_BORDER, (cx, VIRT_H - 55), 6, 1)
        else:
            # Flying ball
            bx, by = int(self.ball_x), int(self.ball_y)
            if 0 < bx < VIRT_W and 0 < by < VIRT_H:
                pygame.draw.circle(screen, (200, 195, 210), (bx + 1, by + 1), 5)
                pygame.draw.circle(screen, WHITE, (bx, by), 5)
                pygame.draw.circle(screen, WII_BORDER, (bx, by), 5, 1)

        draw_wii_badge(screen, cx, VIRT_H - 30, f"POT {self.pot}",
                       fonts["small"])

    def animate_result(self, screen, won, t, fonts):
        draw_wii_bg(screen, top=(200, 180, 160), bottom=(240, 230, 220))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            for i in range(4):
                r = int(t * 120 + i * 15) % 80
                if 0 < r < 80:
                    a = max(0, 1.0 - r / 80)
                    c = (int(WII_GOLD[0] * a), int(WII_GOLD[1] * a),
                         int(WII_GOLD[2] * a))
                    pygame.draw.circle(screen, c, (cx, cy), r, 2)
            txt = fonts["huge"].render("NICE!", True, WII_SUCCESS)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))
        else:
            txt = fonts["huge"].render("GUTTER!", True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))


# ── 2. CRANE GAME ────────────────────────────────────────
class CraneGame(MicroGame):
    name = "CRANE!"
    hint = "POT1+B1"
    instruction = "Grab a prize!"
    base_duration = 6.0
    game_type = "action"

    def reset(self, snap=None, speed_mult=1.0):
        self.claw_x = float(VIRT_W // 2)
        self.claw_target_x = float(VIRT_W // 2)
        self.dropping = False
        self.claw_y = 65.0
        self.drop_speed = 400.0
        self.rising = False
        self.grabbed = False
        self.prize_grabbed_idx = -1
        # Random prizes
        self.prizes = []
        colors = [RED, PINK, CYAN, YELLOW, NEON_GREEN, PURPLE, ORANGE]
        for i in range(6):
            self.prizes.append({
                "x": random.randint(30, VIRT_W - 30),
                "y": random.randint(VIRT_H - 100, VIRT_H - 65),
                "color": colors[i % len(colors)],
                "size": random.randint(6, 10),
                "shape": random.choice(["circle", "rect", "star"]),
            })
        self.sway_phase = 0.0
        self.claw_open = True

    def update(self, snap, dt):
        self.sway_phase += dt * 3
        if not self.dropping and not self.rising:
            self.claw_target_x = 20 + (snap["pot1"] / 4095.0) * (VIRT_W - 40)
            self.claw_x += (self.claw_target_x - self.claw_x) * 0.12
            if snap["btn1"]:
                self.dropping = True
                self.claw_open = True
            return None

        if self.dropping:
            self.claw_y += self.drop_speed * dt
            # Check prize grab
            for i, p in enumerate(self.prizes):
                if (abs(self.claw_x - p["x"]) < 12 and
                        abs(self.claw_y - p["y"]) < 10):
                    self.grabbed = True
                    self.prize_grabbed_idx = i
                    self.claw_open = False
                    self.rising = True
                    self.dropping = False
                    return None
            if self.claw_y >= VIRT_H - 55:
                self.rising = True
                self.dropping = False
            return None

        if self.rising:
            self.claw_y -= self.drop_speed * dt
            if self.claw_y <= 65:
                self.claw_y = 65
                return self.grabbed
            return None

        return None

    def draw(self, screen, snap, tl, tt, fonts):
        # Machine bg
        draw_wii_bg(screen, top=(60, 50, 80), bottom=(40, 35, 60))
        draw_wii_header(screen, "CRANE!", fonts["big"], color=WII_GOLD)
        cx = int(self.claw_x)
        cy = int(self.claw_y)

        # Glass box
        pygame.draw.rect(screen, (80, 75, 100),
                         (10, 50, VIRT_W - 20, VIRT_H - 80), width=2,
                         border_radius=4)

        # Rail at top
        pygame.draw.rect(screen, GREY, (10, 55, VIRT_W - 20, 4))

        # Cable
        pygame.draw.line(screen, GREY, (cx, 57), (cx, cy - 6), 1)

        # Claw
        sway = int(math.sin(self.sway_phase) * (3 if self.dropping else 1))
        claw_cx = cx + sway
        pygame.draw.rect(screen, (200, 200, 210),
                         (claw_cx - 4, cy - 6, 8, 6), border_radius=2)
        if self.claw_open:
            pygame.draw.line(screen, (200, 200, 210),
                             (claw_cx - 4, cy), (claw_cx - 8, cy + 6), 2)
            pygame.draw.line(screen, (200, 200, 210),
                             (claw_cx + 4, cy), (claw_cx + 8, cy + 6), 2)
        else:
            pygame.draw.line(screen, (200, 200, 210),
                             (claw_cx - 4, cy), (claw_cx - 2, cy + 6), 2)
            pygame.draw.line(screen, (200, 200, 210),
                             (claw_cx + 4, cy), (claw_cx + 2, cy + 6), 2)

        # Prizes
        for i, p in enumerate(self.prizes):
            if i == self.prize_grabbed_idx and self.grabbed:
                # Prize follows claw
                px, py = claw_cx, cy + 8
            else:
                px, py = p["x"], p["y"]
            if p["shape"] == "circle":
                pygame.draw.circle(screen, p["color"], (px, py), p["size"])
                pygame.draw.circle(screen, WHITE, (px - 2, py - 2),
                                   p["size"] // 3)
            elif p["shape"] == "rect":
                pygame.draw.rect(screen, p["color"],
                                 (px - p["size"], py - p["size"],
                                  p["size"] * 2, p["size"] * 2),
                                 border_radius=2)
            else:
                # Star shape
                for a_i in range(5):
                    a = a_i * math.pi * 2 / 5 - math.pi / 2
                    sx = px + int(math.cos(a) * p["size"])
                    sy = py + int(math.sin(a) * p["size"])
                    pygame.draw.line(screen, p["color"], (px, py), (sx, sy), 2)

        # Floor
        pygame.draw.rect(screen, (50, 45, 65),
                         (10, VIRT_H - 50, VIRT_W - 20, 20), border_radius=3)

        draw_wii_badge(screen, VIRT_W // 2, VIRT_H - 20, "POT1 + B1",
                       fonts["small"])

    def animate_result(self, screen, won, t, fonts):
        draw_wii_bg(screen, top=(60, 50, 80), bottom=(40, 35, 60))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            p = self.prizes[self.prize_grabbed_idx]
            # Prize bounces up
            bounce_y = cy - int(abs(math.sin(t * 8)) * 20)
            pygame.draw.circle(screen, p["color"], (cx, bounce_y), 12)
            pygame.draw.circle(screen, WHITE, (cx - 3, bounce_y - 3), 4)
            for i in range(8):
                a = t * 5 + i * math.pi / 4
                d = t * 50
                sx = cx + int(math.cos(a) * d)
                sy = bounce_y + int(math.sin(a) * d)
                if 0 < sx < VIRT_W and 20 < sy < VIRT_H:
                    pygame.draw.rect(screen, WII_GOLD, (sx, sy, 2, 2))
            txt = fonts["huge"].render("GOT IT!", True, WII_SUCCESS)
            screen.blit(txt, (cx - txt.get_width() // 2, cy + 30))
        else:
            txt = fonts["huge"].render("EMPTY!", True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))
            # Empty claw swings sadly
            sway = int(math.sin(t * 6) * 8)
            pygame.draw.rect(screen, GREY, (cx + sway - 4, cy - 30, 8, 6),
                             border_radius=2)
            pygame.draw.line(screen, GREY, (cx + sway - 8, cy - 24),
                             (cx + sway - 4, cy - 18), 2)
            pygame.draw.line(screen, GREY, (cx + sway + 8, cy - 24),
                             (cx + sway + 4, cy - 18), 2)


# ── 3. PINBALL ────────────────────────────────────────────
class PinballGame(MicroGame):
    name = "PINBALL!"
    hint = "B1+B2 FLIP"
    instruction = "Hit the bumpers!"
    base_duration = 8.0
    game_type = "action"
    wave_based = True

    def reset(self, snap=None, speed_mult=1.0):
        self.ball_x = float(VIRT_W // 2)
        self.ball_y = 80.0
        self.ball_vx = random.uniform(-30, 30)
        self.ball_vy = 60.0
        self.score = 0
        self.target_score = 3
        self.left_flip = 0.0
        self.right_flip = 0.0
        self.bumpers = [
            {"x": VIRT_W // 2, "y": 120, "r": 10, "hit": 0},
            {"x": VIRT_W // 3, "y": 160, "r": 8, "hit": 0},
            {"x": 2 * VIRT_W // 3, "y": 160, "r": 8, "hit": 0},
            {"x": VIRT_W // 2, "y": 210, "r": 10, "hit": 0},
        ]
        self.dead = False

    def update(self, snap, dt):
        # Flippers
        self.left_flip = 1.0 if snap["btn1"] else max(0, self.left_flip - 5 * dt)
        self.right_flip = 1.0 if snap["btn2"] else max(0, self.right_flip - 5 * dt)

        # Tilt nudge from IMU
        self.ball_vx += snap["acX"] * 30 * dt

        # Physics
        self.ball_vy += 200 * dt  # gravity
        self.ball_x += self.ball_vx * dt
        self.ball_y += self.ball_vy * dt

        # Wall bounces
        if self.ball_x < 15:
            self.ball_x = 15
            self.ball_vx = abs(self.ball_vx) * 0.8
        if self.ball_x > VIRT_W - 15:
            self.ball_x = VIRT_W - 15
            self.ball_vx = -abs(self.ball_vx) * 0.8
        if self.ball_y < 55:
            self.ball_y = 55
            self.ball_vy = abs(self.ball_vy) * 0.6

        # Bumper collisions
        for b in self.bumpers:
            dx = self.ball_x - b["x"]
            dy = self.ball_y - b["y"]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < b["r"] + 4 and b["hit"] <= 0:
                # Bounce away
                if dist > 0:
                    nx, ny = dx / dist, dy / dist
                    self.ball_vx = nx * 120
                    self.ball_vy = ny * 120
                b["hit"] = 8
                self.score += 1
                if self.score >= self.target_score:
                    return True

        for b in self.bumpers:
            if b["hit"] > 0:
                b["hit"] -= 1

        # Flipper zones
        flip_y = VIRT_H - 75
        # Left flipper — wider zone reaching to wall
        if (self.left_flip > 0.5 and
                abs(self.ball_y - flip_y) < 10 and
                15 < self.ball_x < VIRT_W // 2 - 5):
            self.ball_vy = -180
            self.ball_vx = random.uniform(-30, 50)
        # Right flipper — wider zone reaching to wall
        if (self.right_flip > 0.5 and
                abs(self.ball_y - flip_y) < 10 and
                VIRT_W // 2 + 5 < self.ball_x < VIRT_W - 15):
            self.ball_vy = -180
            self.ball_vx = random.uniform(-50, 30)

        # Side gutters — funnel ball toward flippers
        gutter_top = VIRT_H - 130
        gutter_bot = flip_y + 5
        if self.ball_y > gutter_top and self.ball_y < gutter_bot:
            if self.ball_x < 30:
                self.ball_x = 30
                self.ball_vx = abs(self.ball_vx) * 0.6
            if self.ball_x > VIRT_W - 30:
                self.ball_x = VIRT_W - 30
                self.ball_vx = -abs(self.ball_vx) * 0.6
        # Bottom walls below gutters — keep ball on the flippers
        if self.ball_y >= gutter_bot:
            if self.ball_x < 15:
                self.ball_x = 15
                self.ball_vx = abs(self.ball_vx) * 0.8
            if self.ball_x > VIRT_W - 15:
                self.ball_x = VIRT_W - 15
                self.ball_vx = -abs(self.ball_vx) * 0.8

        # Ball lost
        if self.ball_y > VIRT_H - 30:
            return False

        return None

    def draw(self, screen, snap, tl, tt, fonts):
        # Dark table
        screen.fill((20, 30, 50))
        draw_wii_header(screen, "PINBALL!", fonts["big"], color=WII_GOLD)

        cx = VIRT_W // 2
        flip_y = VIRT_H - 75

        # Table borders
        pygame.draw.rect(screen, (60, 70, 100),
                         (10, 50, VIRT_W - 20, VIRT_H - 70), width=2,
                         border_radius=8)

        # Bumpers
        for b in self.bumpers:
            bc = WII_GOLD if b["hit"] > 0 else WII_BLUE
            pygame.draw.circle(screen, bc, (b["x"], b["y"]), b["r"])
            pygame.draw.circle(screen, WHITE, (b["x"], b["y"]), b["r"], 1)
            if b["hit"] > 0:
                pygame.draw.circle(screen, WHITE, (b["x"], b["y"]),
                                   b["r"] + 3, 1)

        # Flippers — V shape at rest, swing up when pressed
        # Both use same angle formula: 0.4 rads at rest, -0.2 when active
        l_angle = 0.4 - self.left_flip * 0.6
        r_angle = 0.4 - self.right_flip * 0.6
        # Left
        lx1, ly1 = cx - 35, flip_y
        lx2 = lx1 + int(math.cos(l_angle) * 30)
        ly2 = ly1 + int(math.sin(l_angle) * 30)
        pygame.draw.line(screen, WII_LIGHT_BLUE, (lx1, ly1), (lx2, ly2), 4)
        # Right
        rx1, ry1 = cx + 35, flip_y
        rx2 = rx1 + int(math.cos(math.pi - r_angle) * 30)
        ry2 = ry1 + int(math.sin(math.pi - r_angle) * 30)
        pygame.draw.line(screen, WII_LIGHT_BLUE, (rx1, ry1), (rx2, ry2), 4)

        # Side gutter walls
        gutter_top = VIRT_H - 130
        gutter_bot = flip_y + 5
        pygame.draw.line(screen, (60, 70, 100), (30, gutter_top),
                         (15, gutter_bot), 2)
        pygame.draw.line(screen, (60, 70, 100), (VIRT_W - 30, gutter_top),
                         (VIRT_W - 15, gutter_bot), 2)
        # Bottom side walls (extend to ball-lost zone)
        pygame.draw.line(screen, (60, 70, 100), (15, gutter_bot),
                         (15, VIRT_H - 30), 2)
        pygame.draw.line(screen, (60, 70, 100), (VIRT_W - 15, gutter_bot),
                         (VIRT_W - 15, VIRT_H - 30), 2)

        # Gutter
        pygame.draw.line(screen, WII_DANGER, (cx - 25, VIRT_H - 45),
                         (cx + 25, VIRT_H - 45), 1)

        # Ball
        bx, by = int(self.ball_x), int(self.ball_y)
        pygame.draw.circle(screen, (220, 220, 230), (bx, by), 4)
        pygame.draw.circle(screen, WHITE, (bx - 1, by - 1), 1)

        # Score
        draw_wii_badge(screen, cx, VIRT_H - 25,
                       f"{self.score}/{self.target_score}", fonts["med"],
                       active=(self.score >= self.target_score),
                       color=WII_SUCCESS)

    def animate_result(self, screen, won, t, fonts):
        screen.fill((20, 30, 50))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            # Multiball explosion
            for i in range(12):
                a = t * 4 + i * math.pi / 6
                d = t * 80
                sx = cx + int(math.cos(a) * d)
                sy = cy + int(math.sin(a) * d)
                if 0 < sx < VIRT_W and 20 < sy < VIRT_H:
                    pygame.draw.circle(screen, WHITE, (sx, sy), 3)
            txt = fonts["huge"].render("JACKPOT!", True, WII_GOLD)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))
        else:
            txt = fonts["huge"].render("DRAIN!", True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))


# ── 4. FRY EGG ───────────────────────────────────────────
class FryEggGame(MicroGame):
    name = "FRY IT!"
    hint = "TILT+BTN"
    instruction = "Flip at the right time!"
    base_duration = 5.0
    game_type = "action"

    def reset(self, snap=None, speed_mult=1.0):
        self.egg_x = float(VIRT_W // 2)
        self.egg_y = float(VIRT_H // 2)
        self.cook_time = 0.0
        self.flipped = False
        self.flip_time = 0.0
        self.flip_window_lo = 1.5
        self.flip_window_hi = 3.0
        self.steam = []
        self.fell_off = False

    def update(self, snap, dt):
        # Tilt moves egg
        self.egg_x += snap["acX"] * 40 * dt
        self.egg_y += snap["acY"] * 40 * dt
        self.egg_x = max(40, min(VIRT_W - 40, self.egg_x))
        self.egg_y = max(80, min(VIRT_H - 80, self.egg_y))

        # Egg fell off the pan (elliptical check matching pan shape)
        pan_cx, pan_cy = VIRT_W // 2, VIRT_H // 2
        pan_rx, pan_ry = 50, 36  # half-widths of inner pan ellipse
        dx = (self.egg_x - pan_cx) / pan_rx
        dy = (self.egg_y - pan_cy) / pan_ry
        if dx * dx + dy * dy > 1:
            self.fell_off = True
            return False

        self.cook_time += dt

        if not self.flipped:
            if snap["btn1"] or snap["btn2"]:
                self.flipped = True
                self.flip_time = self.cook_time
                if self.cook_time < self.flip_window_lo:
                    return False  # too early — raw
                elif self.cook_time > self.flip_window_hi:
                    return False  # too late — burnt
                else:
                    return True  # perfect
        return None

    def draw(self, screen, snap, tl, tt, fonts):
        # Stovetop gradient
        draw_wii_bg(screen, top=(60, 60, 65), bottom=(40, 40, 45))
        draw_wii_header(screen, "FRY IT!", fonts["big"], color=ORANGE)
        cx, cy = VIRT_W // 2, VIRT_H // 2
        ex, ey = int(self.egg_x), int(self.egg_y)

        # Pan
        pygame.draw.ellipse(screen, (50, 50, 55),
                            (cx - 55, cy - 40, 110, 80))
        pygame.draw.ellipse(screen, (70, 70, 78),
                            (cx - 50, cy - 36, 100, 72))
        # Handle
        pygame.draw.rect(screen, (90, 75, 50),
                         (cx + 48, cy - 4, 40, 8), border_radius=3)

        # Egg
        cook_pct = self.cook_time / self.flip_window_hi
        # White
        pygame.draw.ellipse(screen, WHITE,
                            (ex - 14, ey - 10, 28, 20))
        # Yolk color changes with cook time
        if cook_pct < 0.4:
            yc = (255, 210, 50)  # raw yellow
        elif cook_pct < 0.75:
            yc = (240, 180, 30)  # golden
        else:
            yc = (120, 80, 20)   # burnt
        pygame.draw.circle(screen, yc, (ex, ey), 6)
        pygame.draw.circle(screen, (min(255, yc[0] + 30), min(255, yc[1] + 30),
                                     min(255, yc[2] + 20)),
                           (ex - 2, ey - 2), 2)

        # Cook indicator
        if cook_pct < 0.4:
            status, sc = "RAW", WII_LIGHT_BLUE
        elif cook_pct < 0.75:
            status, sc = "GOLDEN!", WII_SUCCESS
        else:
            status, sc = "BURNING!", WII_DANGER
        draw_clean_text(screen, status, fonts["med"], sc,
                        (cx, VIRT_H - 60), center=True)

        # Flip zone indicator bar
        bar_w = 140
        bar_x = (VIRT_W - bar_w) // 2
        bar_y = VIRT_H - 45
        pygame.draw.rect(screen, (60, 60, 65), (bar_x, bar_y, bar_w, 6),
                         border_radius=3)
        # Green zone
        z_lo = int(self.flip_window_lo / 5.0 * bar_w)
        z_hi = int(self.flip_window_hi / 5.0 * bar_w)
        pygame.draw.rect(screen, WII_SUCCESS,
                         (bar_x + z_lo, bar_y, z_hi - z_lo, 6),
                         border_radius=3)
        # Current time marker
        marker_x = bar_x + int(min(1, self.cook_time / 5.0) * bar_w)
        pygame.draw.polygon(screen, WHITE, [
            (marker_x, bar_y - 1), (marker_x - 3, bar_y - 5),
            (marker_x + 3, bar_y - 5)])

        # Steam
        if self.cook_time > 0.5:
            if random.random() < 0.3:
                self.steam.append([ex + random.randint(-8, 8),
                                   ey - 10, random.randint(8, 15)])
        new_s = []
        for s in self.steam:
            s[1] -= 1
            s[2] -= 1
            if s[2] > 0:
                pygame.draw.circle(screen, (180, 180, 190),
                                   (int(s[0]), int(s[1])), 1)
                new_s.append(s)
        self.steam = new_s[-20:]

        draw_wii_badge(screen, cx, VIRT_H - 25, "BTN = FLIP",
                       fonts["small"])

    def animate_result(self, screen, won, t, fonts):
        draw_wii_bg(screen, top=(60, 60, 65), bottom=(40, 40, 45))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            # Egg flips with sparkles
            flip_y = cy - int(abs(math.sin(t * 6)) * 30)
            pygame.draw.ellipse(screen, WHITE, (cx - 14, flip_y - 10, 28, 20))
            pygame.draw.circle(screen, (240, 180, 30), (cx, flip_y), 6)
            txt = fonts["huge"].render("TASTY!", True, WII_SUCCESS)
            screen.blit(txt, (cx - txt.get_width() // 2, cy + 40))
        else:
            if self.fell_off:
                msg = "DROPPED!"
            elif self.cook_time < self.flip_window_lo:
                msg = "TOO RAW!"
            else:
                msg = "BURNT!"
            txt = fonts["huge"].render(msg, True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))


# ── 6. SLINGSHOT ──────────────────────────────────────────
class SlingshotGame(MicroGame):
    name = "SLINGSHOT!"
    hint = "POT1+POT2+B1"
    instruction = "Hit the target!"
    base_duration = 5.0
    game_type = "action"

    def reset(self, snap=None, speed_mult=1.0):
        self.target_x = random.randint(VIRT_W // 2, VIRT_W - 30)
        self.target_y = random.randint(80, VIRT_H // 2)
        self.target_r = max(10, int(18 * speed_mult))
        self.fired = False
        self.proj_x = 0.0
        self.proj_y = 0.0
        self.proj_vx = 0.0
        self.proj_vy = 0.0
        self.aim = 0.5
        self.power = 0.5
        self.t_fire = 0.0
        self.hit = False

    def update(self, snap, dt):
        if not self.fired:
            self.aim = snap["pot1"] / 4095.0
            self.power = snap["pot2"] / 4095.0
            if snap["btn1"]:
                self.fired = True
                angle = -0.2 - self.aim * 1.2  # upward arc
                force = 400 + self.power * 400
                self.proj_x = 30.0
                self.proj_y = float(VIRT_H - 80)
                self.proj_vx = math.cos(angle) * force
                self.proj_vy = math.sin(angle) * force
            return None

        self.t_fire += dt
        self.proj_vy += 180 * dt  # gravity
        self.proj_x += self.proj_vx * dt
        self.proj_y += self.proj_vy * dt

        dist = math.sqrt((self.proj_x - self.target_x) ** 2 +
                          (self.proj_y - self.target_y) ** 2)
        if dist < self.target_r:
            self.hit = True
            return True
        if self.proj_y > VIRT_H or self.proj_x > VIRT_W + 20 or self.t_fire > 3:
            return False
        return None

    def draw(self, screen, snap, tl, tt, fonts):
        draw_wii_bg(screen, top=(180, 220, 255), bottom=(120, 180, 220))
        draw_wii_header(screen, "SLINGSHOT!", fonts["big"])
        cx = VIRT_W // 2

        # Ground
        pygame.draw.rect(screen, (100, 160, 80),
                         (0, VIRT_H - 50, VIRT_W, 50))
        pygame.draw.rect(screen, (80, 140, 60),
                         (0, VIRT_H - 50, VIRT_W, 3))

        # Target (bullseye)
        tx, ty = self.target_x, self.target_y
        pygame.draw.circle(screen, WHITE, (tx, ty), self.target_r)
        pygame.draw.circle(screen, WII_DANGER, (tx, ty), self.target_r, 2)
        pygame.draw.circle(screen, WII_DANGER, (tx, ty),
                           self.target_r // 2)
        pygame.draw.circle(screen, WHITE, (tx, ty), 2)

        # Slingshot (Y-stick)
        sy_base = VIRT_H - 60
        pygame.draw.rect(screen, (100, 70, 30), (25, sy_base - 30, 4, 30),
                         border_radius=1)
        pygame.draw.line(screen, (100, 70, 30), (27, sy_base - 30),
                         (20, sy_base - 40), 2)
        pygame.draw.line(screen, (100, 70, 30), (27, sy_base - 30),
                         (34, sy_base - 40), 2)

        if not self.fired:
            # Elastic band pulled back
            pull = int(self.power * 20)
            pygame.draw.line(screen, (180, 60, 40),
                             (20, sy_base - 40), (27 - pull, sy_base - 30), 2)
            pygame.draw.line(screen, (180, 60, 40),
                             (34, sy_base - 40), (27 - pull, sy_base - 30), 2)
            # Aiming arc (dotted)
            angle = -0.2 - self.aim * 1.2
            force = 400 + self.power * 400
            for i in range(1, 15):
                ft = i * 0.06
                ax = 30 + math.cos(angle) * force * ft
                ay = (VIRT_H - 80) + math.sin(angle) * force * ft + 90 * ft * ft
                if 0 < ax < VIRT_W and 0 < ay < VIRT_H:
                    pygame.draw.rect(screen, BLACK,
                                     (int(ax), int(ay), 3, 3))
                    pygame.draw.rect(screen, WII_DANGER,
                                     (int(ax), int(ay), 2, 2))
            # Rock in sling
            pygame.draw.circle(screen, (120, 110, 100),
                               (27 - pull, sy_base - 30), 4)
        else:
            px, py = int(self.proj_x), int(self.proj_y)
            if 0 < px < VIRT_W and 0 < py < VIRT_H:
                pygame.draw.circle(screen, (120, 110, 100), (px, py), 4)
                pygame.draw.circle(screen, (90, 80, 70), (px, py), 4, 1)

        draw_wii_badge(screen, cx, VIRT_H - 20, "POT1=AIM POT2=POW",
                       fonts["small"])

    def animate_result(self, screen, won, t, fonts):
        draw_wii_bg(screen, top=(180, 220, 255), bottom=(120, 180, 220))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            # Target shatters
            for i in range(15):
                a = i * math.pi * 2 / 15 + t
                d = t * 60
                sx = self.target_x + int(math.cos(a) * d)
                sy = self.target_y + int(math.sin(a) * d)
                if 0 < sx < VIRT_W and 20 < sy < VIRT_H:
                    c = random.choice([RED, WHITE, WII_GOLD])
                    pygame.draw.rect(screen, c, (sx, sy, 3, 2))
            txt = fonts["huge"].render("BULLSEYE!", True, WII_SUCCESS)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))
        else:
            txt = fonts["huge"].render("MISS!", True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))


# ── 7. DJ SCRATCH ─────────────────────────────────────────
class DJScratchGame(MicroGame):
    name = "SCRATCH!"
    hint = "POT1"
    instruction = "Follow the pattern!"
    base_duration = 5.0
    game_type = "action"

    def reset(self, snap=None, speed_mult=1.0):
        self.pot = 1
        self.pattern = []
        count = random.randint(3, 5)
        for _ in range(count):
            self.pattern.append(random.choice(["L", "R"]))
        self.current_step = 0
        self.last_pot = snap["pot1"] if snap else 2048
        self.moving_left = False
        self.moving_right = False
        self.grace = 0.4
        self.spin_angle = 0.0
        self.particles = []
        self.hint = "POT 1"

    def update(self, snap, dt):
        if self.grace > 0:
            self.grace -= dt
            return None
        pot = snap["pot1"]
        delta = pot - self.last_pot
        self.last_pot = pot

        self.spin_angle += delta * 0.05

        if self.current_step >= len(self.pattern):
            return True

        target = self.pattern[self.current_step]
        if target == "L" and delta < -40:
            self.current_step += 1
            self.particles.append([VIRT_W // 2, VIRT_H // 2,
                                   random.randint(5, 10), WII_BLUE])
        elif target == "R" and delta > 40:
            self.current_step += 1
            self.particles.append([VIRT_W // 2, VIRT_H // 2,
                                   random.randint(5, 10), PURPLE])

        if self.current_step >= len(self.pattern):
            return True
        return None

    def draw(self, screen, snap, tl, tt, fonts):
        screen.fill((25, 20, 35))
        draw_wii_header(screen, "SCRATCH!", fonts["big"], color=PURPLE)
        cx, cy = VIRT_W // 2, VIRT_H // 2 - 10

        # Turntable
        pygame.draw.circle(screen, (40, 35, 50), (cx, cy), 55)
        pygame.draw.circle(screen, (30, 25, 40), (cx, cy), 50)
        # Grooves
        for r in range(15, 50, 5):
            pygame.draw.circle(screen, (35, 30, 45), (cx, cy), r, 1)
        # Label
        pygame.draw.circle(screen, PURPLE, (cx, cy), 12)
        pygame.draw.circle(screen, (60, 40, 80), (cx, cy), 12, 1)
        # Spinning marker
        a = math.radians(self.spin_angle)
        mx = cx + int(math.cos(a) * 8)
        my = cy + int(math.sin(a) * 8)
        pygame.draw.circle(screen, WHITE, (mx, my), 2)

        # Tone arm
        arm_a = math.radians(-30 + math.sin(time.time() * 2) * 3)
        ax = cx + 50
        ay = cy - 30
        pygame.draw.line(screen, (160, 160, 170), (ax, ay),
                         (ax + int(math.cos(arm_a) * 20),
                          ay + int(math.sin(arm_a) * 20)), 2)

        # Pattern display
        pat_y = cy + 75
        total_w = len(self.pattern) * 28
        sx0 = cx - total_w // 2
        for i, p in enumerate(self.pattern):
            sx = sx0 + i * 28 + 14
            if i < self.current_step:
                pygame.draw.circle(screen, WII_SUCCESS, (sx, pat_y), 9)
                pygame.draw.line(screen, WHITE, (sx - 3, pat_y),
                                 (sx - 1, pat_y + 3), 2)
                pygame.draw.line(screen, WHITE, (sx - 1, pat_y + 3),
                                 (sx + 4, pat_y - 3), 2)
            elif i == self.current_step:
                pulse = abs(math.sin(time.time() * 4))
                gc = WII_GOLD
                pygame.draw.circle(screen, gc, (sx, pat_y),
                                   int(10 + pulse * 2), 2)
                arrow = "←" if p == "L" else "→"
                at = fonts["med"].render(arrow, True, gc)
                screen.blit(at, (sx - at.get_width() // 2,
                                 pat_y - at.get_height() // 2))
            else:
                pygame.draw.circle(screen, (60, 55, 70), (sx, pat_y), 9, 1)
                arrow = "←" if p == "L" else "→"
                at = fonts["small"].render(arrow, True, (80, 75, 90))
                screen.blit(at, (sx - at.get_width() // 2,
                                 pat_y - at.get_height() // 2))

        # Particles
        new_p = []
        for p in self.particles:
            p[0] += random.uniform(-2, 2)
            p[1] -= random.uniform(0.5, 1.5)
            p[2] -= 1
            if p[2] > 0:
                pygame.draw.rect(screen, p[3], (int(p[0]), int(p[1]), 2, 2))
                new_p.append(p)
        self.particles = new_p

        draw_wii_badge(screen, cx, VIRT_H - 25, "POT 1", fonts["small"])

    def animate_result(self, screen, won, t, fonts):
        screen.fill((25, 20, 35))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            # Record spins fast, notes fly out
            for i in range(int(t * 20)):
                a = i * 0.5 + t * 8
                d = t * 60
                sx = cx + int(math.cos(a) * d)
                sy = cy + int(math.sin(a) * d)
                if 0 < sx < VIRT_W and 20 < sy < VIRT_H:
                    pygame.draw.rect(screen,
                                     random.choice([PURPLE, WII_BLUE, PINK]),
                                     (sx, sy, 2, 2))
            txt = fonts["huge"].render("FIRE MIX!", True, PURPLE)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))
        else:
            txt = fonts["huge"].render("OFF BEAT!", True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))


# ── 8. DON'T SNEEZE ──────────────────────────────────────
class SneezeGame(MicroGame):
    name = "DON'T SNEEZE!"
    hint = "TILT"
    instruction = "Hold still!"
    base_duration = 3.5
    game_type = "survive"

    def reset(self, snap=None, speed_mult=1.0):
        self.grace = 0.5
        self.threshold = max(15, int(30 * speed_mult))
        self.feather_y = -10.0
        self.feather_x = float(VIRT_W // 2)
        self.nose_twitch = 0.0

    def update(self, snap, dt):
        if self.grace > 0:
            self.grace -= dt
            return None
        gx = abs(snap["gyX"])
        gy = abs(snap["gyY"])
        gz = abs(snap["gyZ"])
        movement = gx + gy + gz
        if movement > self.threshold:
            return False  # sneezed!
        # Feather approaches
        self.feather_y += dt * 30
        self.feather_x = VIRT_W // 2 + math.sin(time.time() * 3) * 20
        self.nose_twitch = min(1.0, self.feather_y / (VIRT_H * 0.5))
        return None

    def draw(self, screen, snap, tl, tt, fonts):
        draw_wii_bg(screen, top=(255, 248, 240), bottom=(248, 235, 225))
        draw_wii_header(screen, "DON'T SNEEZE!", fonts["big"],
                        color=WII_DANGER)
        cx, cy = VIRT_W // 2, VIRT_H // 2

        # Big face
        pygame.draw.circle(screen, (255, 222, 186), (cx, cy + 20), 50)
        # Eyes (squinting from irritation)
        squint = int(self.nose_twitch * 3)
        pygame.draw.ellipse(screen, WHITE,
                            (cx - 20, cy + 5 + squint, 12, 8 - squint))
        pygame.draw.ellipse(screen, WHITE,
                            (cx + 8, cy + 5 + squint, 12, 8 - squint))
        pygame.draw.circle(screen, (40, 42, 48), (cx - 14, cy + 9), 3)
        pygame.draw.circle(screen, (40, 42, 48), (cx + 14, cy + 9), 3)
        # Nose (twitching)
        twitch = int(math.sin(time.time() * 12) * self.nose_twitch * 3)
        pygame.draw.ellipse(screen, (240, 195, 160),
                            (cx - 6 + twitch, cy + 20, 12, 8))
        # Nostrils
        pygame.draw.circle(screen, (200, 160, 130),
                           (cx - 2 + twitch, cy + 24), 2)
        pygame.draw.circle(screen, (200, 160, 130),
                           (cx + 3 + twitch, cy + 24), 2)
        # Mouth (clenched)
        pygame.draw.line(screen, (200, 140, 120),
                         (cx - 8, cy + 38), (cx + 8, cy + 38), 1)

        # Feather
        fx, fy = int(self.feather_x), int(self.feather_y)
        if fy > 0:
            sway = math.sin(time.time() * 5) * 5
            # Feather spine
            for i in range(10):
                py = fy + i * 2
                px = fx + int(sway * (1 - i / 10))
                c = (180, 200, 220) if i % 2 == 0 else (200, 220, 240)
                pygame.draw.line(screen, c, (px - 3, py), (px + 3, py), 1)
            pygame.draw.line(screen, (160, 180, 200),
                             (fx, fy), (fx + int(sway), fy + 20), 1)

        # Danger indicator
        pct = self.nose_twitch
        if pct > 0.7:
            draw_clean_text(screen, "AH... AH...", fonts["big"],
                            WII_DANGER, (cx, cy - 35), center=True)

    def animate_result(self, screen, won, t, fonts):
        draw_wii_bg(screen, top=(255, 248, 240), bottom=(248, 235, 225))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            # Relieved face
            pygame.draw.circle(screen, (255, 222, 186), (cx, cy), 40)
            pygame.draw.circle(screen, (40, 42, 48), (cx - 12, cy - 8), 3)
            pygame.draw.circle(screen, (40, 42, 48), (cx + 12, cy - 8), 3)
            pygame.draw.arc(screen, (40, 42, 48),
                            (cx - 8, cy + 8, 16, 8), 3.14, 6.28, 2)
            draw_clean_text(screen, "PHEW!", fonts["huge"], WII_SUCCESS,
                            (cx, cy + 50), center=True)
        else:
            # ACHOO! explosion
            txt = fonts["giant"].render("ACHOO!", True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 20))
            for i in range(20):
                a = i * math.pi * 2 / 20
                d = t * 80
                sx = cx + int(math.cos(a) * d)
                sy = cy + int(math.sin(a) * d)
                if 0 < sx < VIRT_W and 20 < sy < VIRT_H:
                    pygame.draw.circle(screen, WII_LIGHT_BLUE,
                                       (sx, sy), random.randint(1, 3))


# ── 9. SWORD DUEL ────────────────────────────────────────
class SwordFightGame(MicroGame):
    name = "DUEL!"
    hint = "B1=ATK B2=BLK"
    instruction = "Attack & block!"
    base_duration = 12.0
    game_type = "action"
    wave_based = True

    def reset(self, snap=None, speed_mult=1.0):
        self.phase = "wait"  # wait, telegraph, window, result
        self.phase_timer = 0.0
        self.telegraph_time = random.uniform(1.5, 3.0)
        self.window_time = max(0.3, 0.8 * speed_mult)
        self.enemy_attack = False
        self.player_blocked = False
        self.player_attacked = False
        self.hits_needed = 2
        self.hits = 0
        self.sparks = []
        self.enemy_hp = 3
        self._b1_prev = snap["btn1_presses"] if snap else 0
        self._b2_prev = snap["btn2_presses"] if snap else 0

    def update(self, snap, dt):
        b1_new = snap["btn1_presses"] != self._b1_prev
        b2_new = snap["btn2_presses"] != self._b2_prev
        self._b1_prev = snap["btn1_presses"]
        self._b2_prev = snap["btn2_presses"]

        self.phase_timer += dt

        if self.phase == "wait":
            # Wait phase - enemy is idle, player can attack
            if self.phase_timer >= self.telegraph_time:
                self.phase = "telegraph"
                self.phase_timer = 0
                self.enemy_attack = True
            elif b1_new:
                self.hits += 1
                self.sparks = [[VIRT_W // 2 + 20, VIRT_H // 2,
                                random.randint(5, 10),
                                random.choice([WII_GOLD, WHITE])]
                               for _ in range(8)]
                if self.hits >= self.hits_needed:
                    return True
                self.phase = "wait"
                self.phase_timer = 0
                self.telegraph_time = random.uniform(0.8, 2.0)
                self.enemy_attack = False
            return None

        if self.phase == "telegraph":
            # Enemy telegraphing — block now!
            if self.phase_timer >= 0.5:
                self.phase = "window"
                self.phase_timer = 0
            if b2_new:
                self.player_blocked = True
                self.sparks = [[VIRT_W // 2, VIRT_H // 2,
                                random.randint(5, 10),
                                random.choice([WII_BLUE, WII_LIGHT_BLUE])]
                               for _ in range(6)]
                self.phase = "wait"
                self.phase_timer = 0
                self.enemy_attack = False
                self.telegraph_time = random.uniform(0.8, 1.5)
            return None

        if self.phase == "window":
            if b2_new:
                self.player_blocked = True
                self.phase = "wait"
                self.phase_timer = 0
                self.enemy_attack = False
                self.telegraph_time = random.uniform(0.8, 1.5)
                return None
            if self.phase_timer >= self.window_time:
                return False  # didn't block in time
            return None

        return None

    def draw(self, screen, snap, tl, tt, fonts):
        # Castle interior
        draw_wii_bg(screen, top=(60, 55, 70), bottom=(40, 38, 50))
        draw_wii_header(screen, "DUEL!", fonts["big"], color=WII_DANGER)
        cx, cy = VIRT_W // 2, VIRT_H // 2

        # Floor
        pygame.draw.rect(screen, (80, 70, 55),
                         (0, cy + 50, VIRT_W, VIRT_H - cy - 50))

        # Player knight (left)
        px, py = cx - 45, cy + 20
        pygame.draw.rect(screen, (160, 165, 180), (px - 5, py - 15, 10, 20),
                         border_radius=2)
        pygame.draw.circle(screen, (180, 185, 200), (px, py - 20), 7)
        # Sword
        if snap["btn1"]:
            pygame.draw.line(screen, (220, 220, 230), (px + 5, py - 10),
                             (px + 25, py - 25), 2)
        else:
            pygame.draw.line(screen, (200, 200, 210), (px + 5, py - 10),
                             (px + 20, py - 5), 2)
        # Shield
        if snap["btn2"]:
            pygame.draw.rect(screen, WII_BLUE, (px - 10, py - 12, 8, 14),
                             border_radius=2)

        # Enemy knight (right)
        ex, ey = cx + 45, cy + 20
        pygame.draw.rect(screen, (120, 50, 50), (ex - 5, ey - 15, 10, 20),
                         border_radius=2)
        pygame.draw.circle(screen, (140, 60, 60), (ex, ey - 20), 7)
        # Enemy sword
        if self.enemy_attack and self.phase in ("telegraph", "window"):
            # Attacking pose — sword raised
            flash = int(time.time() * 8) % 2
            sc = WII_DANGER if flash else ORANGE
            pygame.draw.line(screen, sc, (ex - 5, ey - 10),
                             (ex - 20, ey - 30), 3)
            draw_clean_text(screen, "!", fonts["big"], WII_DANGER,
                            (ex, ey - 35), center=True)
        else:
            pygame.draw.line(screen, (180, 80, 80), (ex - 5, ey - 10),
                             (ex - 20, ey - 5), 2)

        # Sparks
        new_s = []
        for s in self.sparks:
            s[0] += random.uniform(-2, 2)
            s[1] += random.uniform(-2, 2)
            s[2] -= 1
            if s[2] > 0:
                pygame.draw.rect(screen, s[3], (int(s[0]), int(s[1]), 2, 2))
                new_s.append(s)
        self.sparks = new_s

        # Hit counter
        draw_wii_badge(screen, cx, VIRT_H - 30,
                       f"HITS {self.hits}/{self.hits_needed}",
                       fonts["small"], active=(self.hits > 0))

    def animate_result(self, screen, won, t, fonts):
        draw_wii_bg(screen, top=(60, 55, 70), bottom=(40, 38, 50))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            txt = fonts["huge"].render("VICTORY!", True, WII_GOLD)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))
            for i in range(8):
                a = t * 4 + i * math.pi / 4
                d = t * 50
                sx = cx + int(math.cos(a) * d)
                sy = cy + int(math.sin(a) * d)
                if 0 < sx < VIRT_W and 20 < sy < VIRT_H:
                    pygame.draw.rect(screen, WII_GOLD, (sx, sy, 3, 3))
        else:
            txt = fonts["huge"].render("SLAIN!", True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))


# ── 10. BUG SQUASH ───────────────────────────────────────
class BugSquashGame(MicroGame):
    name = "SQUASH!"
    hint = "POT1+POT2+B1"
    instruction = "Squash the bugs!"
    base_duration = 6.0
    game_type = "action"
    wave_based = True

    def reset(self, snap=None, speed_mult=1.0):
        self.hand_x = float(VIRT_W // 2)
        self.hand_y = float(VIRT_H // 2)
        self.bugs = []
        self.target_squash = 5
        self.squashed = 0
        self.spawn_timer = 0.0
        self.spawned = 0
        self.total_bugs = 8
        self.splats = []
        self.ladybug_hit = False

    def _spawn_bug(self):
        is_ladybug = (self.spawned == 4)  # one ladybug mixed in
        self.bugs.append({
            "x": float(random.randint(20, VIRT_W - 20)),
            "y": float(random.randint(70, VIRT_H - 60)),
            "vx": random.uniform(-20, 20),
            "vy": random.uniform(-20, 20),
            "ladybug": is_ladybug,
            "alive": True,
        })
        self.spawned += 1

    def update(self, snap, dt):
        self.hand_x = 15 + (snap["pot1"] / 4095.0) * (VIRT_W - 30)
        self.hand_y = 60 + (snap["pot2"] / 4095.0) * (VIRT_H - 120)

        # Spawn
        self.spawn_timer += dt
        if self.spawn_timer > 0.5 and self.spawned < self.total_bugs:
            self.spawn_timer = 0
            self._spawn_bug()

        # Move bugs
        for b in self.bugs:
            if not b["alive"]:
                continue
            b["x"] += b["vx"] * dt
            b["y"] += b["vy"] * dt
            if b["x"] < 15 or b["x"] > VIRT_W - 15:
                b["vx"] *= -1
            if b["y"] < 60 or b["y"] > VIRT_H - 50:
                b["vy"] *= -1
            # Random direction changes
            if random.random() < 0.02:
                b["vx"] = random.uniform(-25, 25)
                b["vy"] = random.uniform(-25, 25)

        # Squash check
        if snap["btn1"]:
            for b in self.bugs:
                if not b["alive"]:
                    continue
                if (abs(self.hand_x - b["x"]) < 12 and
                        abs(self.hand_y - b["y"]) < 12):
                    b["alive"] = False
                    if b["ladybug"]:
                        self.ladybug_hit = True
                        return False  # hit the friendly!
                    self.squashed += 1
                    self.splats.append((int(b["x"]), int(b["y"]), 10))
                    if self.squashed >= self.target_squash:
                        return True

        # Update splats
        self.splats = [(x, y, t - 1) for x, y, t in self.splats if t > 0]

        if (self.spawned >= self.total_bugs and
                all(not b["alive"] for b in self.bugs)):
            return self.squashed >= self.target_squash

        return None

    def draw(self, screen, snap, tl, tt, fonts):
        # Kitchen counter
        draw_wii_bg(screen, top=(240, 235, 225), bottom=(220, 212, 200))
        draw_wii_header(screen, "SQUASH!", fonts["big"])
        cx = VIRT_W // 2

        # Counter texture
        for y in range(50, VIRT_H - 40, 8):
            pygame.draw.line(screen, (215, 208, 195),
                             (0, y), (VIRT_W, y), 1)

        # Splats
        for sx, sy, st in self.splats:
            c = (80, 120, 40) if st > 5 else (100, 140, 60)
            pygame.draw.circle(screen, c, (sx, sy), 4)
            for i in range(3):
                dx = random.Random(hash((sx, sy, i))).randint(-5, 5)
                dy = random.Random(hash((sx, sy, i + 7))).randint(-5, 5)
                pygame.draw.rect(screen, c, (sx + dx, sy + dy, 1, 1))

        # Bugs
        for b in self.bugs:
            if not b["alive"]:
                continue
            bx, by = int(b["x"]), int(b["y"])
            if b["ladybug"]:
                # Red with spots
                pygame.draw.ellipse(screen, RED, (bx - 4, by - 3, 8, 6))
                pygame.draw.circle(screen, BLACK, (bx - 1, by - 1), 1)
                pygame.draw.circle(screen, BLACK, (bx + 2, by + 1), 1)
                pygame.draw.line(screen, BLACK, (bx, by - 3), (bx, by + 3), 1)
            else:
                # Dark bug
                pygame.draw.ellipse(screen, (40, 35, 25), (bx - 3, by - 2, 6, 4))
                # Legs
                for dx in [-3, 0, 3]:
                    pygame.draw.line(screen, (60, 50, 35),
                                     (bx + dx, by), (bx + dx - 1, by + 2), 1)
                    pygame.draw.line(screen, (60, 50, 35),
                                     (bx + dx, by), (bx + dx + 1, by - 2), 1)

        # Hand shadow/cursor
        hx, hy = int(self.hand_x), int(self.hand_y)
        # Shadow
        pygame.draw.ellipse(screen, (180, 175, 165),
                            (hx - 10, hy - 8, 20, 16))
        # Hand
        pygame.draw.ellipse(screen, (255, 222, 186),
                            (hx - 8, hy - 6, 16, 12))
        # Fingers
        for i in range(-3, 4, 2):
            pygame.draw.rect(screen, (255, 222, 186),
                             (hx + i - 1, hy - 8, 2, 4), border_radius=1)

        # Warning for ladybug
        for b in self.bugs:
            if b["alive"] and b["ladybug"]:
                lx, ly = int(b["x"]), int(b["y"])
                pygame.draw.circle(screen, WII_SUCCESS, (lx, ly), 8, 1)

        draw_wii_badge(screen, cx, VIRT_H - 25,
                       f"{self.squashed}/{self.target_squash}",
                       fonts["med"], active=(self.squashed >= self.target_squash))

    def animate_result(self, screen, won, t, fonts):
        draw_wii_bg(screen, top=(240, 235, 225), bottom=(220, 212, 200))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            txt = fonts["huge"].render("CLEAN!", True, WII_SUCCESS)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))
        else:
            msg = "LADYBUG!" if self.ladybug_hit else "TOO SLOW!"
            txt = fonts["huge"].render(msg, True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))


# ── 11. TIGHTROPE ────────────────────────────────────────
class TightropeGame(MicroGame):
    name = "TIGHTROPE!"
    hint = "TILT"
    instruction = "Keep your balance!"
    base_duration = 4.5
    game_type = "survive"

    def reset(self, snap=None, speed_mult=1.0):
        self.grace = 0.5
        self.walker_x = 30.0
        self.walk_speed = max(25, 40 * (1.0 / max(0.4, speed_mult)))
        self.tilt = 0.0
        self.max_tilt = 0.8
        self.wobble = 0.0

    def update(self, snap, dt):
        if self.grace > 0:
            self.grace -= dt
            return None
        # Tilt from accelerometer
        self.tilt += snap["acX"] * 2 * dt
        self.tilt *= 0.95  # damping
        self.wobble = abs(self.tilt)

        if self.wobble > self.max_tilt:
            return False

        self.walker_x += self.walk_speed * dt
        if self.walker_x >= VIRT_W - 30:
            return True  # survived = made it across (survive games invert)
        return None

    def draw(self, screen, snap, tl, tt, fonts):
        # Sky
        draw_wii_bg(screen, top=(140, 190, 255), bottom=(200, 220, 255))
        draw_wii_header(screen, "TIGHTROPE!", fonts["big"])
        cx, cy = VIRT_W // 2, VIRT_H // 2

        # City below
        for i in range(8):
            bw = random.Random(i * 7).randint(15, 30)
            bh = random.Random(i * 13).randint(30, 80)
            bx = random.Random(i * 19).randint(0, VIRT_W - bw)
            bc = random.Random(i * 23).randint(100, 160)
            pygame.draw.rect(screen, (bc, bc - 10, bc - 20),
                             (bx, VIRT_H - 40 - bh, bw, bh + 40))
            # Windows
            for wy in range(VIRT_H - 40 - bh + 5, VIRT_H - 40, 8):
                for wx in range(bx + 3, bx + bw - 3, 6):
                    wc = (255, 240, 150) if random.Random(wx + wy).random() > 0.5 else (bc + 10, bc, bc - 10)
                    pygame.draw.rect(screen, wc, (wx, wy, 3, 4))

        # Rope
        rope_y = cy + 40
        pygame.draw.line(screen, (140, 120, 90), (10, rope_y),
                         (VIRT_W - 10, rope_y), 2)
        # Poles
        pygame.draw.rect(screen, (120, 100, 70), (8, rope_y - 30, 4, 60))
        pygame.draw.rect(screen, (120, 100, 70),
                         (VIRT_W - 12, rope_y - 30, 4, 60))

        # Walker Mii
        wx = int(self.walker_x)
        tilt_angle = self.tilt * 30  # visual degrees
        wy = rope_y - 12

        # Body tilts
        pygame.draw.circle(screen, (255, 222, 186), (wx, wy - 6), 5)
        pygame.draw.circle(screen, (40, 42, 48), (wx - 1, wy - 7), 1)
        pygame.draw.circle(screen, (40, 42, 48), (wx + 2, wy - 7), 1)
        pygame.draw.line(screen, WII_BLUE, (wx, wy), (wx, wy + 8), 2)
        # Balance pole
        pole_dx = int(tilt_angle * 0.8)
        pygame.draw.line(screen, (180, 160, 120),
                         (wx - 18 + pole_dx, wy + 2),
                         (wx + 18 + pole_dx, wy + 2), 2)
        # Legs
        step = int(time.time() * 6) % 2
        pygame.draw.line(screen, WII_TEXT, (wx, wy + 8),
                         (wx - 3 + step * 6, wy + 14), 1)
        pygame.draw.line(screen, WII_TEXT, (wx, wy + 8),
                         (wx + 3 - step * 6, wy + 14), 1)

        # Tilt meter
        meter_w = 60
        meter_x = cx - meter_w // 2
        meter_y = 52
        pygame.draw.rect(screen, (210, 215, 225),
                         (meter_x, meter_y, meter_w, 6), border_radius=3)
        # Danger zones
        danger_w = int(meter_w * 0.15)
        pygame.draw.rect(screen, (240, 200, 200),
                         (meter_x, meter_y, danger_w, 6), border_radius=3)
        pygame.draw.rect(screen, (240, 200, 200),
                         (meter_x + meter_w - danger_w, meter_y, danger_w, 6),
                         border_radius=3)
        # Indicator
        ind_x = meter_x + int((self.tilt / self.max_tilt * 0.5 + 0.5) * meter_w)
        ind_x = max(meter_x, min(meter_x + meter_w, ind_x))
        ic = WII_DANGER if self.wobble > self.max_tilt * 0.7 else WII_SUCCESS
        pygame.draw.circle(screen, ic, (ind_x, meter_y + 3), 4)

        # Progress
        pct = self.walker_x / (VIRT_W - 30)
        draw_wii_progress_bar(screen, 20, VIRT_H - 30, VIRT_W - 40, 6, pct,
                              color=WII_SUCCESS)

    def animate_result(self, screen, won, t, fonts):
        draw_wii_bg(screen, top=(140, 190, 255), bottom=(200, 220, 255))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            # Mii celebrates at end
            draw_mii(screen, VIRT_W - 30, cy + 25, size=12, happy=True)
            txt = fonts["huge"].render("MADE IT!", True, WII_SUCCESS)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 20))
        else:
            # Falling
            fall_y = cy + 40 + int(t * 200)
            draw_mii(screen, cx, fall_y, size=10, happy=False)
            txt = fonts["huge"].render("FELL!", True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 20))


# ── 12. SAFE CRACK ────────────────────────────────────────
class SafeCrackGame(MicroGame):
    name = "CRACK IT!"
    hint = "POT1"
    instruction = "Find the combo!"
    base_duration = 7.0
    game_type = "action"

    def reset(self, snap=None, speed_mult=1.0):
        self.combo = [random.randint(10, 90),
                      random.randint(10, 90),
                      random.randint(10, 90)]
        self.directions = ["CW", "CCW", "CW"]
        self.current_step = 0
        self.tolerance = max(5, int(10 * speed_mult))
        self.dial_value = 50.0
        self.grace = 0.3
        self.hold_timer = 0.0
        self.hold_needed = 0.3
        self.click_flash = 0

    def update(self, snap, dt):
        if self.grace > 0:
            self.grace -= dt
            return None
        self.dial_value = (snap["pot1"] / 4095.0) * 100
        if self.current_step >= len(self.combo):
            return True
        target = self.combo[self.current_step]
        if abs(self.dial_value - target) < self.tolerance:
            self.hold_timer += dt
            if self.hold_timer >= self.hold_needed:
                self.current_step += 1
                self.hold_timer = 0
                self.click_flash = 8
                if self.current_step >= len(self.combo):
                    return True
        else:
            self.hold_timer = max(0, self.hold_timer - dt * 2)
        if self.click_flash > 0:
            self.click_flash -= 1
        return None

    def draw(self, screen, snap, tl, tt, fonts):
        # Vault bg
        draw_wii_bg(screen, top=(50, 55, 65), bottom=(35, 38, 48))
        draw_wii_header(screen, "CRACK IT!", fonts["big"], color=WII_GOLD)
        cx, cy = VIRT_W // 2, VIRT_H // 2

        # Vault door
        pygame.draw.circle(screen, (80, 85, 95), (cx, cy), 60)
        pygame.draw.circle(screen, (70, 75, 85), (cx, cy), 55)
        pygame.draw.circle(screen, (60, 65, 75), (cx, cy), 55, 2)

        # Dial
        dial_r = 35
        pygame.draw.circle(screen, (100, 105, 115), (cx, cy), dial_r)
        pygame.draw.circle(screen, (120, 125, 135), (cx, cy), dial_r, 2)

        # Number ticks
        for i in range(0, 100, 10):
            a = math.radians(i * 3.6 - 90)
            tx = cx + int(math.cos(a) * (dial_r - 4))
            ty = cy + int(math.sin(a) * (dial_r - 4))
            ox = cx + int(math.cos(a) * (dial_r + 2))
            oy = cy + int(math.sin(a) * (dial_r + 2))
            pygame.draw.line(screen, (160, 165, 175), (tx, ty), (ox, oy), 1)

        # Dial needle
        needle_a = math.radians(self.dial_value * 3.6 - 90)
        nx = cx + int(math.cos(needle_a) * (dial_r - 8))
        ny = cy + int(math.sin(needle_a) * (dial_r - 8))
        nc = WII_SUCCESS if self.click_flash > 0 else WII_DANGER
        pygame.draw.line(screen, nc, (cx, cy), (nx, ny), 2)
        pygame.draw.circle(screen, WHITE, (cx, cy), 3)

        # Target indicator
        if self.current_step < len(self.combo):
            target = self.combo[self.current_step]
            ta = math.radians(target * 3.6 - 90)
            for r in range(dial_r + 4, dial_r + 8):
                lo_a = math.radians((target - self.tolerance) * 3.6 - 90)
                hi_a = math.radians((target + self.tolerance) * 3.6 - 90)
                for deg in range(int((target - self.tolerance) * 3.6),
                                 int((target + self.tolerance) * 3.6), 2):
                    da = math.radians(deg - 90)
                    px = cx + int(math.cos(da) * r)
                    py = cy + int(math.sin(da) * r)
                    pygame.draw.rect(screen, WII_SUCCESS, (px, py, 1, 1))

        # Combo progress
        step_y = cy + 75
        for i in range(3):
            sx = cx - 30 + i * 30
            if i < self.current_step:
                pygame.draw.circle(screen, WII_SUCCESS, (sx, step_y), 8)
                num = fonts["small"].render(str(self.combo[i]), True, WHITE)
            elif i == self.current_step:
                pygame.draw.circle(screen, WII_GOLD, (sx, step_y), 8, 2)
                num = fonts["small"].render(
                    self.directions[i], True, WII_GOLD)
            else:
                pygame.draw.circle(screen, (80, 85, 95), (sx, step_y), 8, 1)
                num = fonts["small"].render("?", True, (100, 105, 115))
            screen.blit(num, (sx - num.get_width() // 2,
                              step_y - num.get_height() // 2))

        # Hold progress
        if self.hold_timer > 0:
            hp = self.hold_timer / self.hold_needed
            draw_wii_progress_bar(screen, cx - 30, cy + 90, 60, 6, hp,
                                  color=WII_SUCCESS)

        draw_wii_badge(screen, cx, VIRT_H - 25, "POT 1", fonts["small"])

    def animate_result(self, screen, won, t, fonts):
        draw_wii_bg(screen, top=(50, 55, 65), bottom=(35, 38, 48))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            # Door opens
            open_pct = min(t / 0.4, 1.0)
            w = int(60 * max(0.1, 1 - open_pct * 0.8))
            pygame.draw.ellipse(screen, (80, 85, 95),
                                (cx - w, cy - 60, w * 2, 120))
            if open_pct > 0.3:
                for i in range(int(t * 15)):
                    a = i * 0.6 + t * 5
                    d = t * 50
                    sx = cx + int(math.cos(a) * d)
                    sy = cy + int(math.sin(a) * d)
                    if 0 < sx < VIRT_W and 20 < sy < VIRT_H:
                        pygame.draw.rect(screen, WII_GOLD, (sx, sy, 3, 2))
            txt = fonts["huge"].render("CRACKED!", True, WII_GOLD)
            screen.blit(txt, (cx - txt.get_width() // 2, cy + 70))
        else:
            txt = fonts["huge"].render("LOCKED!", True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))


# ── 13. POPCORN ───────────────────────────────────────────
class PopcornGame(MicroGame):
    name = "POPCORN!"
    hint = "TILT"
    instruction = "Shake it!"
    base_duration = 4.5
    game_type = "action"

    def reset(self, snap=None, speed_mult=1.0):
        self.popped = 0
        self.target = max(5, int(8 * speed_mult))
        self.kernels = []
        for i in range(12):
            self.kernels.append({
                "x": random.uniform(VIRT_W // 2 - 25, VIRT_W // 2 + 25),
                "y": random.uniform(VIRT_H // 2, VIRT_H // 2 + 30),
                "popped": False, "pop_timer": 0.0, "threshold": random.uniform(0.3, 0.8),
            })
        self.shake_energy = 0.0

    def update(self, snap, dt):
        gx = abs(snap["gyX"])
        gy = abs(snap["gyY"])
        gz = abs(snap["gyZ"])
        intensity = (gx + gy + gz) / 100.0
        self.shake_energy = intensity

        for k in self.kernels:
            if k["popped"]:
                continue
            if intensity > k["threshold"]:
                k["pop_timer"] += dt
                if k["pop_timer"] > 0.2:
                    k["popped"] = True
                    k["x"] += random.uniform(-15, 15)
                    k["y"] -= random.uniform(10, 30)
                    self.popped += 1
            else:
                k["pop_timer"] = max(0, k["pop_timer"] - dt)

        if self.popped >= self.target:
            return True
        return None

    def draw(self, screen, snap, tl, tt, fonts):
        draw_wii_bg(screen, top=(255, 245, 220), bottom=(240, 225, 195))
        draw_wii_header(screen, "POPCORN!", fonts["big"], color=ORANGE)
        cx, cy = VIRT_W // 2, VIRT_H // 2

        # Pot
        shake = int(self.shake_energy * 3)
        ox = random.randint(-shake, shake) if shake > 0 else 0
        pot_cx = cx + ox
        pygame.draw.rect(screen, (180, 50, 40),
                         (pot_cx - 30, cy + 30, 60, 25), border_radius=4)
        pygame.draw.rect(screen, (160, 40, 30),
                         (pot_cx - 30, cy + 30, 60, 25), width=2,
                         border_radius=4)
        # Lid
        pygame.draw.rect(screen, (190, 195, 205),
                         (pot_cx - 32, cy + 25, 64, 6), border_radius=3)
        pygame.draw.circle(screen, (170, 175, 185), (pot_cx, cy + 25), 3)

        # Kernels
        for k in self.kernels:
            kx, ky = int(k["x"]) + ox, int(k["y"])
            if k["popped"]:
                # Fluffy white popcorn
                pygame.draw.circle(screen, WHITE, (kx, ky), 4)
                pygame.draw.circle(screen, (245, 240, 220), (kx - 1, ky - 1), 2)
                pygame.draw.circle(screen, (240, 235, 210), (kx + 1, ky + 1), 2)
            else:
                # Yellow kernel
                pygame.draw.circle(screen, (220, 180, 40), (kx, ky), 2)

        # Steam
        if self.shake_energy > 0.2:
            for i in range(3):
                sx = pot_cx + random.randint(-15, 15)
                sy = cy + 20 - random.randint(0, 15)
                pygame.draw.circle(screen, (230, 230, 240), (sx, sy), 1)

        draw_wii_badge(screen, cx, VIRT_H - 25,
                       f"{self.popped}/{self.target}", fonts["med"],
                       active=(self.popped >= self.target))

    def animate_result(self, screen, won, t, fonts):
        draw_wii_bg(screen, top=(255, 245, 220), bottom=(240, 225, 195))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            # Popcorn explosion
            for i in range(20):
                a = i * math.pi * 2 / 20 + t * 2
                d = t * 80
                sx = cx + int(math.cos(a) * d)
                sy = cy + int(math.sin(a) * d)
                if 0 < sx < VIRT_W and 20 < sy < VIRT_H:
                    pygame.draw.circle(screen, WHITE, (sx, sy), 3)
            txt = fonts["huge"].render("DELISH!", True, WII_SUCCESS)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))
        else:
            txt = fonts["huge"].render("UNPOPPED!", True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))


# ── 14. WINDSURF ──────────────────────────────────────────
class WindsurfGame(MicroGame):
    name = "WINDSURF!"
    hint = "POT1+TILT"
    instruction = "Match the wind!"
    base_duration = 6.0
    game_type = "action"

    def reset(self, snap=None, speed_mult=1.0):
        self.speed = 0.0
        self.distance = 0.0
        self.target_distance = 100.0
        self.wind_dir = random.uniform(0.2, 0.8)
        self.wind_change_timer = 0.0
        self.waves = [(random.randint(0, VIRT_W),
                       random.uniform(0.5, 1.5)) for _ in range(20)]

    def update(self, snap, dt):
        # Wind changes
        self.wind_change_timer += dt
        if self.wind_change_timer > 2.0:
            self.wind_dir = random.uniform(0.15, 0.85)
            self.wind_change_timer = 0

        # Sail angle from pot1
        sail = snap["pot1"] / 4095.0
        # How well sail matches wind
        match = 1.0 - abs(sail - self.wind_dir)
        lean = snap["acX"]  # tilt bonus
        lean_bonus = max(0, 0.3 - abs(lean)) * 2

        self.speed += (match * 40 + lean_bonus * 10) * dt
        self.speed *= 0.95  # drag
        self.speed = max(0, min(50, self.speed))
        self.distance += self.speed * dt

        if self.distance >= self.target_distance:
            return True
        return None

    def draw(self, screen, snap, tl, tt, fonts):
        # Ocean
        draw_wii_bg(screen, top=(100, 170, 230), bottom=(30, 80, 140))
        draw_wii_header(screen, "WINDSURF!", fonts["big"], color=WHITE)
        cx, cy = VIRT_W // 2, VIRT_H // 2

        # Waves
        for i, (wx, spd) in enumerate(self.waves):
            wy = 60 + (i * 20 + int(time.time() * spd * 30)) % (VIRT_H - 80)
            wave_x = (wx + int(time.time() * spd * 20)) % VIRT_W
            pygame.draw.arc(screen, (120, 190, 240),
                            (wave_x - 6, wy, 12, 4), 0, 3.14, 1)

        # Wind direction arrow
        wind_x = int(20 + self.wind_dir * (VIRT_W - 40))
        pygame.draw.polygon(screen, (200, 220, 255), [
            (wind_x, 52), (wind_x - 5, 58), (wind_x + 5, 58)])
        draw_clean_text(screen, "WIND", fonts["small"],
                        (200, 220, 255), (wind_x, 62), center=True)

        # Surfer
        surfer_x = cx
        surfer_y = cy + 20
        # Board
        pygame.draw.ellipse(screen, (200, 160, 60),
                            (surfer_x - 12, surfer_y + 4, 24, 6))
        # Person
        pygame.draw.line(screen, WII_TEXT, (surfer_x, surfer_y + 4),
                         (surfer_x, surfer_y - 8), 2)
        pygame.draw.circle(screen, (255, 222, 186), (surfer_x, surfer_y - 12), 4)
        # Sail
        sail_angle = snap["pot1"] / 4095.0
        sail_dx = int((sail_angle - 0.5) * 20)
        pygame.draw.polygon(screen, WHITE, [
            (surfer_x, surfer_y - 20),
            (surfer_x + sail_dx, surfer_y - 8),
            (surfer_x, surfer_y)])

        # Speed meter
        draw_wii_progress_bar(screen, 20, VIRT_H - 40, VIRT_W - 40, 6,
                              self.speed / 50, color=WII_SUCCESS)
        # Distance
        dist_pct = self.distance / self.target_distance
        draw_wii_progress_bar(screen, 20, VIRT_H - 28, VIRT_W - 40, 6,
                              dist_pct, color=WII_BLUE)
        draw_clean_text(screen, f"{int(dist_pct * 100)}%", fonts["small"],
                        WHITE, (cx, VIRT_H - 15), center=True)

    def animate_result(self, screen, won, t, fonts):
        draw_wii_bg(screen, top=(100, 170, 230), bottom=(30, 80, 140))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            txt = fonts["huge"].render("RADICAL!", True, WII_SUCCESS)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))
        else:
            txt = fonts["huge"].render("BECALMED!", True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))


# ── 15. PICKPOCKET ────────────────────────────────────────
class PickpocketGame(MicroGame):
    name = "STEAL!"
    hint = "BTN1"
    instruction = "Wait for it...!"
    base_duration = 10.0
    game_type = "action"
    wave_based = True

    def reset(self, snap=None, speed_mult=1.0):
        self.phase = "approach"  # approach, window, result
        self.approach_time = random.uniform(2.0, 4.0)
        self.window_time = max(0.2, 0.6 * speed_mult)
        self.elapsed = 0.0
        self.hand_progress = 0.0
        self.too_early = False

    def update(self, snap, dt):
        self.elapsed += dt
        if self.phase == "approach":
            self.hand_progress = min(1.0, self.elapsed / self.approach_time)
            if snap["btn1"]:
                if self.hand_progress < 0.85:
                    self.too_early = True
                    return False
            if self.elapsed >= self.approach_time:
                self.phase = "window"
                self.elapsed = 0
            return None

        if self.phase == "window":
            if snap["btn1"]:
                return True  # grabbed it!
            if self.elapsed >= self.window_time:
                return False  # too slow
            return None

        return None

    def draw(self, screen, snap, tl, tt, fonts):
        # Street scene
        draw_wii_bg(screen, top=(80, 75, 90), bottom=(50, 48, 60))
        draw_wii_header(screen, "STEAL!", fonts["big"], color=WII_GOLD)
        cx, cy = VIRT_W // 2, VIRT_H // 2

        # Target person walking
        person_x = cx + 20
        person_y = cy
        pygame.draw.circle(screen, (200, 170, 140), (person_x, person_y - 15), 6)
        pygame.draw.rect(screen, (100, 80, 60),
                         (person_x - 5, person_y - 8, 10, 18), border_radius=2)
        step = int(time.time() * 4) % 2
        pygame.draw.line(screen, (80, 60, 40), (person_x, person_y + 10),
                         (person_x - 3 + step * 6, person_y + 18), 1)
        pygame.draw.line(screen, (80, 60, 40), (person_x, person_y + 10),
                         (person_x + 3 - step * 6, person_y + 18), 1)

        # Back pocket with wallet visible
        pocket_x = person_x + 3
        pocket_y = person_y + 5
        pygame.draw.rect(screen, (80, 65, 45), (pocket_x, pocket_y, 6, 4),
                         border_radius=1)
        pygame.draw.rect(screen, (140, 90, 40), (pocket_x + 1, pocket_y - 2, 4, 3),
                         border_radius=1)

        # Sneaking hand
        hand_x = person_x - 30 + int(self.hand_progress * 30)
        hand_y = pocket_y
        pygame.draw.ellipse(screen, (255, 222, 186),
                            (hand_x - 4, hand_y - 3, 8, 6))
        for fi in range(-2, 3):
            pygame.draw.rect(screen, (255, 222, 186),
                             (hand_x + fi, hand_y + 3, 1, 3))

        if self.too_early:
            draw_clean_text(screen, "TOO EARLY!", fonts["huge"],
                            WII_DANGER, (cx, cy - 50), center=True)
        elif self.phase == "window":
            flash = int(time.time() * 8) % 2
            if flash:
                draw_clean_text(screen, "NOW!", fonts["giant"],
                                WII_GOLD, (cx, cy - 50), center=True)
        else:
            dots = "." * (int(self.elapsed * 2) % 4)
            draw_clean_text(screen, f"Wait{dots}", fonts["big"],
                            WII_TEXT_LIGHT, (cx, cy - 50), center=True)

        draw_wii_badge(screen, cx, VIRT_H - 25, "BTN 1", fonts["small"])

    def animate_result(self, screen, won, t, fonts):
        draw_wii_bg(screen, top=(80, 75, 90), bottom=(50, 48, 60))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            txt = fonts["huge"].render("SWIPED!", True, WII_GOLD)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))
        else:
            msg = "BUSTED!" if self.too_early else "TOO SLOW!"
            txt = fonts["huge"].render(msg, True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))


# ── 16. STACK ─────────────────────────────────────────────
class StackGame(MicroGame):
    name = "STACK!"
    hint = "BTN1"
    instruction = "Stack the blocks!"
    base_duration = 10.0
    game_type = "action"
    wave_based = True

    def reset(self, snap=None, speed_mult=1.0):
        self.blocks = []
        self.block_w = 50.0
        self.current_x = 0.0
        self.current_dir = 1
        self.speed = max(80, 120 / speed_mult)
        self.target_stack = 5
        self.dropping = False
        self.drop_y = 0.0
        self.stack_top_y = float(VIRT_H - 50)
        self.block_h = 12
        self._b1_prev = snap["btn1_presses"] if snap else 0

    def update(self, snap, dt):
        b1_new = snap["btn1_presses"] != self._b1_prev
        self._b1_prev = snap["btn1_presses"]

        if self.dropping:
            self.drop_y += 300 * dt
            target_y = self.stack_top_y - self.block_h
            if self.drop_y >= target_y:
                # Land the block
                if self.blocks:
                    last = self.blocks[-1]
                    overlap_lo = max(self.current_x, last["x"])
                    overlap_hi = min(self.current_x + self.block_w,
                                     last["x"] + last["w"])
                    overlap = overlap_hi - overlap_lo
                    if overlap <= 0:
                        return False  # missed completely
                    self.blocks.append({
                        "x": overlap_lo, "w": overlap,
                        "y": target_y})
                    self.block_w = overlap
                else:
                    self.blocks.append({
                        "x": self.current_x, "w": self.block_w,
                        "y": target_y})
                self.stack_top_y -= self.block_h
                self.dropping = False
                self.current_dir = random.choice([-1, 1])
                self.current_x = 0 if self.current_dir == 1 else VIRT_W - self.block_w
                if len(self.blocks) >= self.target_stack:
                    return True
            return None

        # Slide
        self.current_x += self.speed * self.current_dir * dt
        if self.current_x + self.block_w > VIRT_W - 10:
            self.current_dir = -1
        if self.current_x < 10:
            self.current_dir = 1

        if b1_new:
            self.dropping = True
            self.drop_y = 60.0

        return None

    def draw(self, screen, snap, tl, tt, fonts):
        draw_wii_bg(screen, top=(200, 220, 245), bottom=(170, 195, 225))
        draw_wii_header(screen, "STACK!", fonts["big"])
        cx = VIRT_W // 2

        # Ground
        pygame.draw.rect(screen, (150, 140, 130),
                         (0, VIRT_H - 50, VIRT_W, 50))

        # Stacked blocks
        colors = [WII_BLUE, WII_SUCCESS, WII_GOLD, ORANGE, PURPLE, CYAN,
                  PINK, WII_DANGER]
        for i, b in enumerate(self.blocks):
            c = colors[i % len(colors)]
            pygame.draw.rect(screen, c,
                             (int(b["x"]), int(b["y"]),
                              int(b["w"]), self.block_h),
                             border_radius=2)
            pygame.draw.rect(screen, (min(255, c[0] + 30),
                                       min(255, c[1] + 30),
                                       min(255, c[2] + 30)),
                             (int(b["x"]) + 1, int(b["y"]) + 1,
                              int(b["w"]) - 2, 3),
                             border_radius=1)

        # Sliding block
        if not self.dropping:
            c = colors[len(self.blocks) % len(colors)]
            y = self.stack_top_y - self.block_h
            pygame.draw.rect(screen, c,
                             (int(self.current_x), int(y),
                              int(self.block_w), self.block_h),
                             border_radius=2)
        else:
            c = colors[len(self.blocks) % len(colors)]
            pygame.draw.rect(screen, c,
                             (int(self.current_x), int(self.drop_y),
                              int(self.block_w), self.block_h),
                             border_radius=2)

        draw_wii_badge(screen, cx, VIRT_H - 20,
                       f"{len(self.blocks)}/{self.target_stack}",
                       fonts["med"],
                       active=(len(self.blocks) >= self.target_stack))

    def animate_result(self, screen, won, t, fonts):
        draw_wii_bg(screen, top=(200, 220, 245), bottom=(170, 195, 225))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            txt = fonts["huge"].render("TOWER!", True, WII_SUCCESS)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))
        else:
            txt = fonts["huge"].render("TOPPLED!", True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))


# ── 17. SPACE INVADERS ───────────────────────────────────
class SpaceInvaderGame(MicroGame):
    name = "INVADERS!"
    hint = "POT1+BTN1"
    instruction = "Shoot them down!"
    base_duration = 8.0
    game_type = "action"
    wave_based = True

    def reset(self, snap=None, speed_mult=1.0):
        self.ship_x = float(VIRT_W // 2)
        self.bullets = []
        self.aliens = []
        self.kills = 0
        self.target_kills = 5
        self.fire_cooldown = 0.0
        self._b1_prev = snap["btn1_presses"] if snap else 0
        # Create alien rows
        for row in range(2):
            for col in range(5):
                self.aliens.append({
                    "x": 30 + col * 36,
                    "y": 70 + row * 24,
                    "alive": True,
                    "type": row,
                })
        self.alien_dir = 1
        self.alien_speed = max(15, 25 / speed_mult)
        self.alien_move_timer = 0.0

    def update(self, snap, dt):
        self.ship_x = 15 + (snap["pot1"] / 4095.0) * (VIRT_W - 30)
        self.fire_cooldown = max(0, self.fire_cooldown - dt)

        b1_new = snap["btn1_presses"] != self._b1_prev
        self._b1_prev = snap["btn1_presses"]

        if b1_new and self.fire_cooldown <= 0:
            self.bullets.append([self.ship_x, VIRT_H - 60])
            self.fire_cooldown = 0.3

        # Move bullets
        new_b = []
        for b in self.bullets:
            b[1] -= 200 * dt
            if b[1] > 40:
                new_b.append(b)
        self.bullets = new_b

        # Move aliens
        self.alien_move_timer += dt
        if self.alien_move_timer > 0.5:
            self.alien_move_timer = 0
            # Check edges
            min_x = min((a["x"] for a in self.aliens if a["alive"]), default=50)
            max_x = max((a["x"] for a in self.aliens if a["alive"]), default=VIRT_W - 50)
            if max_x > VIRT_W - 20 and self.alien_dir == 1:
                self.alien_dir = -1
                for a in self.aliens:
                    a["y"] += 8
            elif min_x < 20 and self.alien_dir == -1:
                self.alien_dir = 1
                for a in self.aliens:
                    a["y"] += 8
            for a in self.aliens:
                if a["alive"]:
                    a["x"] += self.alien_speed * self.alien_dir

        # Collision detection
        for b in self.bullets[:]:
            for a in self.aliens:
                if not a["alive"]:
                    continue
                if abs(b[0] - a["x"]) < 8 and abs(b[1] - a["y"]) < 8:
                    a["alive"] = False
                    self.kills += 1
                    if b in self.bullets:
                        self.bullets.remove(b)
                    break

        if self.kills >= self.target_kills:
            return True

        # Check if aliens reached bottom
        for a in self.aliens:
            if a["alive"] and a["y"] > VIRT_H - 70:
                return False

        return None

    def draw(self, screen, snap, tl, tt, fonts):
        screen.fill((5, 5, 15))
        draw_wii_header(screen, "INVADERS!", fonts["big"], color=NEON_GREEN)

        # Stars
        for i in range(30):
            sx = random.Random(i * 17).randint(0, VIRT_W)
            sy = random.Random(i * 31).randint(50, VIRT_H - 50)
            pygame.draw.rect(screen, (100, 100, 120), (sx, sy, 1, 1))

        # Aliens
        for a in self.aliens:
            if not a["alive"]:
                continue
            ax, ay = int(a["x"]), int(a["y"])
            if a["type"] == 0:
                # Squid type
                pygame.draw.rect(screen, NEON_GREEN, (ax - 5, ay - 4, 10, 8))
                pygame.draw.rect(screen, NEON_GREEN, (ax - 7, ay - 2, 2, 4))
                pygame.draw.rect(screen, NEON_GREEN, (ax + 5, ay - 2, 2, 4))
                pygame.draw.rect(screen, BLACK, (ax - 2, ay - 2, 2, 2))
                pygame.draw.rect(screen, BLACK, (ax + 1, ay - 2, 2, 2))
            else:
                # Crab type
                pygame.draw.rect(screen, CYAN, (ax - 6, ay - 3, 12, 6))
                pygame.draw.rect(screen, CYAN, (ax - 3, ay - 5, 6, 2))
                pygame.draw.rect(screen, BLACK, (ax - 2, ay - 2, 2, 2))
                pygame.draw.rect(screen, BLACK, (ax + 1, ay - 2, 2, 2))

        # Bullets
        for b in self.bullets:
            pygame.draw.rect(screen, WHITE, (int(b[0]) - 1, int(b[1]), 2, 5))

        # Ship
        sx = int(self.ship_x)
        sy = VIRT_H - 55
        pygame.draw.polygon(screen, NEON_GREEN, [
            (sx, sy - 5), (sx - 7, sy + 3), (sx + 7, sy + 3)])
        pygame.draw.rect(screen, NEON_GREEN, (sx - 3, sy + 3, 6, 3))

        # Score
        draw_wii_badge(screen, VIRT_W // 2, VIRT_H - 20,
                       f"{self.kills}/{self.target_kills}", fonts["med"],
                       active=(self.kills >= self.target_kills),
                       color=NEON_GREEN)

    def animate_result(self, screen, won, t, fonts):
        screen.fill((5, 5, 15))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            txt = fonts["huge"].render("CLEARED!", True, NEON_GREEN)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))
        else:
            txt = fonts["huge"].render("INVADED!", True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))


# ── 18. BREAKOUT ──────────────────────────────────────────
class BreakoutGame(MicroGame):
    name = "BREAKOUT!"
    hint = "POT1"
    instruction = "Break the bricks!"
    base_duration = 10.0
    game_type = "action"
    wave_based = True

    def reset(self, snap=None, speed_mult=1.0):
        self.paddle_x = float(VIRT_W // 2)
        self.ball_x = float(VIRT_W // 2)
        self.ball_y = float(VIRT_H - 80)
        self.ball_vx = random.choice([-60, 60])
        self.ball_vy = -120.0
        self.bricks = []
        self.target_bricks = 8
        self.broken = 0
        colors = [RED, ORANGE, YELLOW, GREEN, CYAN]
        for row in range(3):
            for col in range(7):
                self.bricks.append({
                    "x": 12 + col * 31, "y": 60 + row * 12,
                    "w": 28, "h": 10, "alive": True,
                    "color": colors[row % len(colors)]})

    def update(self, snap, dt):
        self.paddle_x = 20 + (snap["pot1"] / 4095.0) * (VIRT_W - 40)

        self.ball_x += self.ball_vx * dt
        self.ball_y += self.ball_vy * dt

        # Wall bounces
        if self.ball_x < 5:
            self.ball_x = 5
            self.ball_vx = abs(self.ball_vx)
        if self.ball_x > VIRT_W - 5:
            self.ball_x = VIRT_W - 5
            self.ball_vx = -abs(self.ball_vx)
        if self.ball_y < 50:
            self.ball_y = 50
            self.ball_vy = abs(self.ball_vy)

        # Paddle bounce
        paddle_y = VIRT_H - 55
        if (abs(self.ball_y - paddle_y) < 5 and
                abs(self.ball_x - self.paddle_x) < 18 and
                self.ball_vy > 0):
            self.ball_vy = -abs(self.ball_vy)
            offset = (self.ball_x - self.paddle_x) / 18
            self.ball_vx = offset * 120

        # Brick collisions
        for b in self.bricks:
            if not b["alive"]:
                continue
            if (b["x"] < self.ball_x < b["x"] + b["w"] and
                    b["y"] < self.ball_y < b["y"] + b["h"]):
                b["alive"] = False
                self.ball_vy *= -1
                self.broken += 1
                if self.broken >= self.target_bricks:
                    return True
                break

        # Ball lost
        if self.ball_y > VIRT_H - 20:
            return False

        return None

    def draw(self, screen, snap, tl, tt, fonts):
        screen.fill((10, 10, 20))
        draw_wii_header(screen, "BREAKOUT!", fonts["big"], color=CYAN)

        # Bricks
        for b in self.bricks:
            if not b["alive"]:
                continue
            pygame.draw.rect(screen, b["color"],
                             (b["x"], b["y"], b["w"], b["h"]),
                             border_radius=2)
            pygame.draw.rect(screen, (min(255, b["color"][0] + 40),
                                       min(255, b["color"][1] + 40),
                                       min(255, b["color"][2] + 40)),
                             (b["x"] + 1, b["y"] + 1, b["w"] - 2, 3),
                             border_radius=1)

        # Ball
        bx, by = int(self.ball_x), int(self.ball_y)
        pygame.draw.circle(screen, WHITE, (bx, by), 3)

        # Paddle
        px = int(self.paddle_x)
        py = VIRT_H - 55
        pygame.draw.rect(screen, WII_BLUE, (px - 16, py, 32, 5),
                         border_radius=2)
        pygame.draw.rect(screen, WII_LIGHT_BLUE, (px - 14, py + 1, 28, 2),
                         border_radius=1)

        draw_wii_badge(screen, VIRT_W // 2, VIRT_H - 20,
                       f"{self.broken}/{self.target_bricks}", fonts["med"],
                       active=(self.broken >= self.target_bricks),
                       color=CYAN)

    def animate_result(self, screen, won, t, fonts):
        screen.fill((10, 10, 20))
        cx, cy = VIRT_W // 2, VIRT_H // 2
        if won:
            for i in range(15):
                a = i * math.pi * 2 / 15 + t * 3
                d = t * 60
                sx = cx + int(math.cos(a) * d)
                sy = cy + int(math.sin(a) * d)
                if 0 < sx < VIRT_W and 20 < sy < VIRT_H:
                    c = random.choice([RED, ORANGE, YELLOW, GREEN, CYAN])
                    pygame.draw.rect(screen, c, (sx, sy, 4, 3))
            txt = fonts["huge"].render("SMASHED!", True, CYAN)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))
        else:
            txt = fonts["huge"].render("GAME OVER!", True, WII_DANGER)
            screen.blit(txt, (cx - txt.get_width() // 2, cy - 15))


# ── All game instances ────────────────────────────────────
ALL_GAMES = [
    SkeeBallGame(),
    CraneGame(),
    PinballGame(),
    FryEggGame(),
    SlingshotGame(),
    DJScratchGame(),
    SneezeGame(),
    SwordFightGame(),
    BugSquashGame(),
    TightropeGame(),
    SafeCrackGame(),
    PopcornGame(),
    WindsurfGame(),
    PickpocketGame(),
    StackGame(),
    SpaceInvaderGame(),
    BreakoutGame(),
]


# ═══════════════════════════════════════════════════════════
#  GAME ENGINE
# ═══════════════════════════════════════════════════════════
class GameEngine:
    def __init__(self, serial_port="/dev/ttyACM0"):
        self.serial_port = serial_port
        self.state = HardwareState()
        self.comm = None

        # Load calibration if it exists
        cal = load_calibration()
        if cal:
            apply_calibration(self.state, cal)

        pygame.init()
        self.real_screen = pygame.display.set_mode(
            (SCREEN_W, SCREEN_H), pygame.FULLSCREEN | pygame.DOUBLEBUF)
        self.screen = pygame.Surface((VIRT_W, VIRT_H))
        pygame.display.set_caption("PiWare")
        self.clock = pygame.time.Clock()

        bold_ttf = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        self.fonts = {
            "small": pygame.font.Font(bold_ttf, 8),
            "med":   pygame.font.Font(bold_ttf, 11),
            "big":   pygame.font.Font(bold_ttf, 14),
            "huge":  pygame.font.Font(bold_ttf, 20),
            "giant": pygame.font.Font(bold_ttf, 28),
            "title": pygame.font.Font(bold_ttf, 24),
        }

        # Pre-warm gradient cache for common backgrounds
        make_gradient_surface(VIRT_W, VIRT_H, (242, 247, 252), (218, 228, 238))
        make_gradient_surface(VIRT_W, 44, (248, 250, 253), (236, 238, 245),
                              key=("header_grad",))

        # Touch input
        self.touch_queue = []
        self.touch_res_x, self.touch_res_y = SCREEN_W, SCREEN_H
        td = find_touch_device()
        if td:
            caps = td.capabilities(absinfo=True)
            for code, absinfo in caps.get(ecodes.EV_ABS, []):
                if code in (ecodes.ABS_MT_POSITION_X, ecodes.ABS_X):
                    self.touch_res_x = absinfo.max
                elif code in (ecodes.ABS_MT_POSITION_Y, ecodes.ABS_Y):
                    self.touch_res_y = absinfo.max
            threading.Thread(
                target=touch_reader, args=(td, self.touch_queue), daemon=True
            ).start()
        else:
            print("Warning: no touchscreen found, touch input disabled.")

    def flip(self):
        pygame.transform.scale(self.screen, (SCREEN_W, SCREEN_H),
                               self.real_screen)
        pygame.display.flip()

    def get_touches(self):
        touches = []
        while self.touch_queue:
            rx, ry = self.touch_queue.pop(0)
            touches.append((
                int(rx * VIRT_W / self.touch_res_x),
                int(ry * VIRT_H / self.touch_res_y),
            ))
        return touches

    def check_quit(self):
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                return True
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                return True
        return False

    def center_text(self, text, font_key, color, y):
        txt = self.fonts[font_key].render(text, False, color)
        self.screen.blit(
            txt, (VIRT_W // 2 - txt.get_width() // 2, y))

    def set_led(self, color, on):
        if self.comm:
            self.comm.set_led(color, on)

    def all_leds_off(self):
        for c in ("r", "g", "b"):
            self.set_led(c, False)

    def _led_flash(self, color, times=3, on_time=0.08, off_time=0.08):
        for _ in range(times):
            self.set_led(color, True)
            time.sleep(on_time)
            self.set_led(color, False)
            time.sleep(off_time)

    def led_win(self):
        threading.Thread(
            target=self._led_flash, args=("g",), daemon=True).start()

    def led_lose(self):
        threading.Thread(
            target=self._led_flash, args=("r",), daemon=True).start()

    def draw_timer(self, time_left, time_total):
        pct = max(time_left / time_total, 0)
        if pct > 0.5:
            color = WII_SUCCESS
        elif pct > 0.25:
            color = WII_GOLD
        else:
            color = WII_DANGER
        draw_wii_progress_bar(self.screen, 0, 0, VIRT_W, 5, pct, color=color)

    def draw_hud(self, score, lives, speed_level):
        y = 7
        draw_clean_text(self.screen, str(score), self.fonts["small"],
                        WII_TEXT, (6, y))
        if speed_level > 1:
            draw_wii_badge(self.screen, VIRT_W // 2, y + 4,
                           f"Lv{speed_level}", self.fonts["small"],
                           active=True, color=WII_BLUE)
        for i in range(3):
            hx = VIRT_W - 35 + i * 10
            if i < lives:
                pygame.draw.circle(self.screen, WII_DANGER, (hx, y + 4), 3)
                pygame.draw.circle(self.screen, WII_DANGER, (hx + 4, y + 4), 3)
                pygame.draw.polygon(self.screen, WII_DANGER, [
                    (hx - 3, y + 5), (hx + 2, y + 10), (hx + 7, y + 5)])
            else:
                pygame.draw.circle(self.screen, WII_BORDER, (hx, y + 4), 3, 1)
                pygame.draw.circle(self.screen, WII_BORDER, (hx + 4, y + 4), 3, 1)
                pygame.draw.polygon(self.screen, WII_BORDER, [
                    (hx - 3, y + 5), (hx + 2, y + 10), (hx + 7, y + 5)], 1)

    def flash_screen(self, text, color, bg, duration=0.8, subtitle=""):
        start = time.time()
        while time.time() - start < duration:
            if self.check_quit():
                return
            t = time.time() - start
            draw_wii_bg(self.screen)
            cx, cy = VIRT_W // 2, VIRT_H // 2
            scale = min(t / 0.15, 1.0)
            panel_w = int(180 * scale)
            panel_h = int(80 * scale)
            if panel_w > 20 and panel_h > 10:
                draw_wii_panel(self.screen,
                               (cx - panel_w // 2, cy - panel_h // 2,
                                panel_w, panel_h),
                               border_color=color)
            if scale > 0.5:
                txt = self.fonts["giant"].render(text, True, color)
                self.screen.blit(txt, (cx - txt.get_width() // 2,
                                       cy - txt.get_height() // 2 - 8))
                if subtitle:
                    st = self.fonts["med"].render(subtitle, True, WII_TEXT_LIGHT)
                    self.screen.blit(st, (cx - st.get_width() // 2,
                                          cy + txt.get_height() // 2 - 4))
            self.flip()
            self.clock.tick(FPS)

    def show_instruction_screen(self, game, duration=1.0):
        """WarioWare-style instruction splash shown before each microgame.

        Displays a scale-in panel with three tiers of text:
        - game name (medium, top)  — identifies the game
        - instruction (large, center) — action command; falls back to game.name
        - hint (small, bottom) — control reminder

        Args:
            game: a MicroGame instance with name, instruction and hint attributes.
            duration: how long the screen is shown in seconds (default 1.0).
        """
        instruction = game.instruction or game.name
        start = time.time()
        while time.time() - start < duration:
            if self.check_quit():
                return
            t = time.time() - start
            draw_wii_bg(self.screen)
            cx, cy = VIRT_W // 2, VIRT_H // 2
            scale = min(t / 0.15, 1.0)
            panel_w = int(220 * scale)
            panel_h = int(100 * scale)
            if panel_w > 20 and panel_h > 10:
                draw_wii_panel(self.screen,
                               (cx - panel_w // 2, cy - panel_h // 2,
                                panel_w, panel_h),
                               border_color=WII_GOLD)
            if scale > 0.5:
                name_txt = self.fonts["med"].render(game.name, True, WII_TEXT_LIGHT)
                self.screen.blit(name_txt,
                                 (cx - name_txt.get_width() // 2,
                                  cy - panel_h // 2 + 6))
                instr_txt = self.fonts["huge"].render(instruction, True, YELLOW)
                self.screen.blit(instr_txt,
                                 (cx - instr_txt.get_width() // 2,
                                  cy - instr_txt.get_height() // 2))
                if game.hint:
                    hint_txt = self.fonts["small"].render(game.hint, True,
                                                          WII_TEXT_LIGHT)
                    self.screen.blit(hint_txt,
                                     (cx - hint_txt.get_width() // 2,
                                      cy + panel_h // 2 - hint_txt.get_height() - 4))
            self.flip()
            self.clock.tick(FPS)

    # ── CALIBRATION SCREEN ────────────────────────────────
    def run_calibration(self):
        """MPU-6050 calibration: collect samples while device is still."""
        cx = VIRT_W // 2
        total_samples = 200
        samples = {"acX": [], "acY": [], "acZ": [],
                   "gyX": [], "gyY": [], "gyZ": []}
        phase = "instruct"  # instruct -> collect -> done
        collected = 0
        instruct_timer = 0.0

        while True:
            if self.check_quit():
                return
            dt = 1.0 / FPS

            if phase == "instruct":
                instruct_timer += dt
                draw_wii_bg(self.screen)
                draw_wii_header(self.screen, "CALIBRATE", self.fonts["big"],
                                color=WII_BLUE)
                draw_wii_panel(self.screen,
                               (20, 80, VIRT_W - 40, 120),
                               border_color=WII_BLUE)
                draw_clean_text(self.screen, "Place device",
                                self.fonts["big"], WII_TEXT,
                                (cx, 105), center=True)
                draw_clean_text(self.screen, "flat on table.",
                                self.fonts["big"], WII_TEXT,
                                (cx, 125), center=True)
                draw_clean_text(self.screen, "Don't touch it!",
                                self.fonts["med"], WII_DANGER,
                                (cx, 150), center=True)
                # Animated device icon
                dev_y = 230 + int(math.sin(time.time() * 2) * 3)
                pygame.draw.rect(self.screen, WII_TEXT,
                                 (cx - 20, dev_y, 40, 8), border_radius=2)
                pygame.draw.line(self.screen, WII_BORDER,
                                 (cx - 30, dev_y + 12), (cx + 30, dev_y + 12), 1)
                # Down arrow
                if int(time.time() * 3) % 2:
                    pygame.draw.polygon(self.screen, WII_BLUE, [
                        (cx, dev_y + 20), (cx - 6, dev_y + 14),
                        (cx + 6, dev_y + 14)])

                blink = int(time.time() * 2) % 2
                if blink:
                    draw_clean_text(self.screen, "Press B1 to start",
                                    self.fonts["small"], WII_TEXT_LIGHT,
                                    (cx, VIRT_H - 30), center=True)
                draw_clean_text(self.screen, "B2 to skip",
                                self.fonts["small"], WII_TEXT_LIGHT,
                                (cx, VIRT_H - 16), center=True)

                snap = self.state.snapshot()
                if snap["btn1"]:
                    phase = "collect"
                    time.sleep(0.3)
                if snap["btn2"]:
                    return
                self.flip()
                self.clock.tick(FPS)
                continue

            if phase == "collect":
                snap = self.state.snapshot_raw()
                if snap["connected"]:
                    samples["acX"].append(snap["acX"])
                    samples["acY"].append(snap["acY"])
                    samples["acZ"].append(snap["acZ"])
                    samples["gyX"].append(snap["gyX"])
                    samples["gyY"].append(snap["gyY"])
                    samples["gyZ"].append(snap["gyZ"])
                    collected += 1

                pct = collected / total_samples
                draw_wii_bg(self.screen)
                draw_wii_header(self.screen, "CALIBRATING...",
                                self.fonts["big"], color=WII_BLUE)

                # Spinner
                frame = int(time.time() * 10)
                for i in range(8):
                    angle = (frame * 5 + i * 45) * math.pi / 180
                    px = cx + int(math.cos(angle) * 18)
                    py = 120 + int(math.sin(angle) * 18)
                    brightness = 1.0 if i == frame % 8 else 0.3
                    c = (int(WII_BLUE[0] * brightness),
                         int(WII_BLUE[1] * brightness),
                         int(WII_BLUE[2] * brightness))
                    pygame.draw.circle(self.screen, c, (px, py), 3)

                draw_clean_text(self.screen, "Collecting samples...",
                                self.fonts["med"], WII_TEXT,
                                (cx, 160), center=True)
                draw_wii_progress_bar(self.screen, 30, 180,
                                      VIRT_W - 60, 10, pct,
                                      color=WII_BLUE)
                draw_clean_text(self.screen,
                                f"{collected}/{total_samples}",
                                self.fonts["small"], WII_TEXT_LIGHT,
                                (cx, 200), center=True)

                # Live readings
                draw_wii_panel(self.screen, (20, 220, VIRT_W - 40, 80),
                               border_color=WII_BORDER)
                labels = ["acX", "acY", "acZ", "gyX", "gyY", "gyZ"]
                for i, lbl in enumerate(labels):
                    col = i % 3
                    row = i // 3
                    lx = 35 + col * 65
                    ly = 235 + row * 25
                    val = snap[lbl]
                    draw_clean_text(self.screen, f"{lbl}:{val:.2f}",
                                    self.fonts["small"], WII_TEXT_LIGHT,
                                    (lx, ly))

                if collected >= total_samples:
                    phase = "done"

                self.flip()
                self.clock.tick(FPS)
                continue

            if phase == "done":
                # Compute offsets
                avg = {}
                for key in samples:
                    vals = samples[key]
                    avg[key] = sum(vals) / len(vals) if vals else 0.0

                cal = {
                    "acX": avg["acX"],       # should be ~0 when flat
                    "acY": avg["acY"],       # should be ~0 when flat
                    "acZ": avg["acZ"] - 1.0, # should be ~1g when flat
                    "gyX": avg["gyX"],       # should be ~0 when still
                    "gyY": avg["gyY"],
                    "gyZ": avg["gyZ"],
                }
                save_calibration(cal)
                apply_calibration(self.state, cal)

                # Show results
                show_start = time.time()
                while time.time() - show_start < 2.5:
                    if self.check_quit():
                        return
                    t = time.time() - show_start
                    draw_wii_bg(self.screen)
                    draw_wii_header(self.screen, "CALIBRATED!",
                                    self.fonts["big"], color=WII_SUCCESS)

                    # Checkmark animation
                    scale = min(t / 0.3, 1.0)
                    if scale > 0:
                        pts = [(cx - int(15 * scale), 120),
                               (cx - int(3 * scale), 120 + int(15 * scale)),
                               (cx + int(20 * scale), 120 - int(15 * scale))]
                        pygame.draw.lines(self.screen, WII_SUCCESS,
                                          False, pts, 4)

                    # Offset values
                    draw_wii_panel(self.screen, (15, 160, VIRT_W - 30, 110),
                                   border_color=WII_SUCCESS)
                    draw_clean_text(self.screen, "Offsets Saved:",
                                    self.fonts["med"], WII_TEXT,
                                    (cx, 175), center=True)
                    labels = ["acX", "acY", "acZ", "gyX", "gyY", "gyZ"]
                    for i, lbl in enumerate(labels):
                        col = i % 3
                        row = i // 3
                        lx = 30 + col * 68
                        ly = 195 + row * 25
                        draw_clean_text(self.screen,
                                        f"{lbl}:{cal[lbl]:+.3f}",
                                        self.fonts["small"], WII_TEXT,
                                        (lx, ly))

                    draw_clean_text(self.screen, "Saved to calibration.json",
                                    self.fonts["small"], WII_TEXT_LIGHT,
                                    (cx, 290), center=True)

                    self.flip()
                    self.clock.tick(FPS)
                return

    def run_menu(self):
        """Returns 'serial', 'wifi', or 'lte'."""
        btn_w, btn_h = 180, 32
        gap = 10
        cx = VIRT_W // 2
        x0 = cx - btn_w // 2
        y0 = 215

        buttons = [
            ("serial", "Serial USB",  WII_BLUE,     x0, y0),
            ("wifi",   "WiFi",        WII_SUCCESS,  x0, y0 + btn_h + gap),
            ("lte",    "LTE",         ORANGE,        x0, y0 + 2 * (btn_h + gap)),
        ]
        hi_scores = load_highscores()
        hi_top = hi_scores[0] if hi_scores else 0

        # Pre-render menu header gradient (no scanlines)
        header_grad = make_gradient_surface(VIRT_W, 80,
                                            (242, 247, 252), (230, 235, 242),
                                            key=("menu_header",))

        while True:
            if self.check_quit():
                pygame.quit()
                sys.exit()

            for tx, ty in self.get_touches():
                for mode, _lbl, _c, bx, by in buttons:
                    if pygame.Rect(bx, by, btn_w, btn_h).collidepoint(tx, ty):
                        return mode

            draw_wii_bg(self.screen)

            # Smooth header band (cached gradient, no per-line draws)
            self.screen.blit(header_grad, (0, 0))

            draw_clean_text(self.screen, "PiWare", self.fonts["title"],
                            WII_BLUE, (cx, 50), center=True)
            draw_clean_text(self.screen, "MICROGAMES", self.fonts["med"],
                            WII_TEXT_LIGHT, (cx, 82), center=True)

            pygame.draw.line(self.screen, WII_BORDER,
                             (40, 105), (VIRT_W - 40, 105), 1)

            if hi_top > 0:
                draw_clean_text(self.screen, f"Best: {hi_top}",
                                self.fonts["med"], WII_GOLD,
                                (cx, 125), center=True)

            draw_clean_text(self.screen, "Connection", self.fonts["med"],
                            WII_TEXT_LIGHT, (cx, 195), center=True)

            for mode, label, color, bx, by in buttons:
                rect = (bx, by, btn_w, btn_h)
                draw_wii_button(self.screen, rect, label, self.fonts["med"],
                                color=color)

            draw_clean_text(self.screen, "ESP32 + Raspberry Pi",
                            self.fonts["small"], WII_TEXT_LIGHT,
                            (cx, VIRT_H - 14), center=True)

            self.flip()
            self.clock.tick(FPS)

    def wait_for_connection(self):
        dots = 0
        last_dot = time.time()
        frame = 0
        while True:
            if self.check_quit():
                return False
            snap = self.state.snapshot()
            if snap["connected"]:
                return True
            if time.time() - last_dot > 0.4:
                dots = (dots + 1) % 4
                last_dot = time.time()
            frame += 1
            draw_wii_bg(self.screen)
            cx, cy = VIRT_W // 2, VIRT_H // 2 - 20

            for i in range(8):
                angle = (frame * 3 + i * 45) * math.pi / 180
                px = cx + int(math.cos(angle) * 18)
                py = cy + int(math.sin(angle) * 18)
                brightness = 1.0 if i == (frame // 3) % 8 else 0.3
                c = (int(WII_BLUE[0] * brightness),
                     int(WII_BLUE[1] * brightness),
                     int(WII_BLUE[2] * brightness))
                pygame.draw.circle(self.screen, c, (px, py), 3)

            txt = "Connecting" + "." * dots
            draw_clean_text(self.screen, txt, self.fonts["big"], WII_TEXT,
                            (cx, cy + 35), center=True)
            draw_clean_text(self.screen, "Waiting for ESP32",
                            self.fonts["small"], WII_TEXT_LIGHT,
                            (cx, cy + 55), center=True)
            self.flip()
            self.clock.tick(FPS)

    def title_screen(self):
        """Returns 'arcade', 'practice', 'calibrate', or False (quit)."""
        time.sleep(0.3)
        frame = 0
        btn_w, btn_h = 170, 30
        cx = VIRT_W // 2
        arcade_rect = pygame.Rect(cx - btn_w // 2, 190, btn_w, btn_h)
        practice_rect = pygame.Rect(cx - btn_w // 2, 228, btn_w, btn_h)
        calibrate_rect = pygame.Rect(cx - btn_w // 2, 266, btn_w, btn_h)

        _snap0 = self.state.snapshot()
        _prev_b1 = _snap0["btn1_presses"]
        _prev_b2 = _snap0["btn2_presses"]

        # Pre-render title header gradient
        title_header = make_gradient_surface(VIRT_W, 70,
                                             (242, 247, 252), (230, 235, 242),
                                             key=("title_header",))

        while True:
            if self.check_quit():
                return False
            touches = self.get_touches()
            snap = self.state.snapshot()
            b1_new = snap["btn1_presses"] != _prev_b1
            b2_new = snap["btn2_presses"] != _prev_b2
            _prev_b1 = snap["btn1_presses"]
            _prev_b2 = snap["btn2_presses"]
            if b1_new:
                return "arcade"
            if b2_new:
                return "practice"
            for tx, ty in touches:
                if arcade_rect.collidepoint(tx, ty):
                    return "arcade"
                if practice_rect.collidepoint(tx, ty):
                    return "practice"
                if calibrate_rect.collidepoint(tx, ty):
                    return "calibrate"

            frame += 1
            draw_wii_bg(self.screen)
            self.screen.blit(title_header, (0, 0))

            draw_clean_text(self.screen, "PiWare", self.fonts["title"],
                            WII_BLUE, (cx, 38), center=True)
            draw_clean_text(self.screen, "MICROGAMES", self.fonts["med"],
                            WII_TEXT_LIGHT, (cx, 68), center=True)

            pygame.draw.line(self.screen, WII_BORDER,
                             (30, 85), (VIRT_W - 30, 85), 1)

            dot_c = WII_SUCCESS if snap["connected"] else WII_DANGER
            pygame.draw.circle(self.screen, dot_c, (cx - 30, 100), 3)
            draw_clean_text(self.screen, "Connected", self.fonts["small"],
                            WII_TEXT_LIGHT, (cx - 22, 100), center=False)

            # Cal status
            cal = load_calibration()
            if cal:
                pygame.draw.circle(self.screen, WII_SUCCESS, (cx - 30, 115), 3)
                draw_clean_text(self.screen, "Calibrated", self.fonts["small"],
                                WII_TEXT_LIGHT, (cx - 22, 115))
            else:
                pygame.draw.circle(self.screen, ORANGE, (cx - 30, 115), 3)
                draw_clean_text(self.screen, "Not calibrated",
                                self.fonts["small"], ORANGE, (cx - 22, 115))

            draw_clean_text(self.screen, "Select Mode", self.fonts["med"],
                            WII_TEXT, (cx, 160), center=True)

            draw_wii_button(self.screen,
                            (arcade_rect.x, arcade_rect.y,
                             arcade_rect.w, arcade_rect.h),
                            "B1  Arcade", self.fonts["med"],
                            color=WII_BLUE)
            draw_wii_button(self.screen,
                            (practice_rect.x, practice_rect.y,
                             practice_rect.w, practice_rect.h),
                            "B2  Practice", self.fonts["med"],
                            color=WII_LIGHT_BLUE)
            draw_wii_button(self.screen,
                            (calibrate_rect.x, calibrate_rect.y,
                             calibrate_rect.w, calibrate_rect.h),
                            "Calibrate IMU", self.fonts["med"],
                            color=ORANGE)

            # High scores
            hi_scores = load_highscores()
            if hi_scores:
                panel_y = 310
                draw_wii_panel(self.screen,
                               (25, panel_y, VIRT_W - 50, 80),
                               border_color=WII_BORDER)
                draw_clean_text(self.screen, "High Scores",
                                self.fonts["med"], WII_TEXT,
                                (cx, panel_y + 12), center=True)
                for i, sc in enumerate(hi_scores[:5]):
                    rank_colors = [WII_GOLD, WII_TEXT, ORANGE,
                                   WII_TEXT_LIGHT, WII_TEXT_LIGHT]
                    entry = f"{i + 1}.  {sc}"
                    draw_clean_text(self.screen, entry, self.fonts["small"],
                                    rank_colors[i],
                                    (cx, panel_y + 28 + i * 10), center=True)

            blink = int(time.time() * 2) % 2
            if blink:
                draw_clean_text(self.screen, "Press B1 or B2",
                                self.fonts["small"], WII_TEXT_LIGHT,
                                (cx, VIRT_H - 16), center=True)

            self.flip()
            self.clock.tick(FPS)

    def practice_select_screen(self):
        selected = 0
        level = 1
        scroll_y = 0.0
        _prev_b1 = 0
        _prev_b2 = 0
        _first_snap = True

        max_level = 13
        games = ALL_GAMES
        item_h = 24
        cx = VIRT_W // 2
        back_rect = pygame.Rect(2, 2, 36, 16)

        def level_to_mult(lv):
            return max(0.4, 1.0 - (lv - 1) * 0.05)

        def level_color(lv):
            frac = (lv - 1) / (max_level - 1)
            if frac < 0.33:
                return WII_SUCCESS
            elif frac < 0.66:
                return WII_GOLD
            elif frac < 0.85:
                return ORANGE
            return WII_DANGER

        while True:
            if self.check_quit():
                return None
            snap = self.state.snapshot()
            touches = self.get_touches()

            if _first_snap:
                _prev_b1 = snap["btn1_presses"]
                _prev_b2 = snap["btn2_presses"]
                _first_snap = False

            b1_new = snap["btn1_presses"] != _prev_b1
            b2_new = snap["btn2_presses"] != _prev_b2
            _prev_b1 = snap["btn1_presses"]
            _prev_b2 = snap["btn2_presses"]

            pot_idx = int(snap["pot1"] / 4095 * (len(games) - 1) + 0.5)
            selected = max(0, min(pot_idx, len(games) - 1))

            level = int(snap["pot2"] / 4095 * (max_level - 1) + 0.5) + 1
            level = max(1, min(level, max_level))

            if b1_new:
                return (selected, level_to_mult(level))
            if b2_new:
                return None

            for tx, ty in touches:
                if back_rect.collidepoint(tx, ty):
                    return None
                list_y_start = 88
                for i in range(len(games)):
                    iy = list_y_start + i * item_h - int(scroll_y)
                    if list_y_start - 5 < iy < VIRT_H - 50:
                        if pygame.Rect(10, iy, VIRT_W - 20, item_h).collidepoint(tx, ty):
                            selected = i
                            return (selected, level_to_mult(level))

            target_scroll = max(0, selected * item_h - 5 * item_h)
            scroll_y += (target_scroll - scroll_y) * 0.2

            draw_wii_bg(self.screen)

            draw_wii_panel(self.screen, (0, 0, VIRT_W, 82),
                           border_color=WII_BORDER)
            draw_clean_text(self.screen, "Practice Mode", self.fonts["big"],
                            WII_BLUE, (cx, 14), center=True)
            draw_clean_text(self.screen, "POT1: Select   POT2: Speed",
                            self.fonts["small"], WII_TEXT_LIGHT,
                            (cx, 34), center=True)

            lc = level_color(level)
            draw_clean_text(self.screen, f"Level {level}/{max_level}",
                            self.fonts["med"], lc, (cx, 50), center=True)
            bar_w = VIRT_W - 50
            bar_x, bar_y = 25, 66
            draw_wii_progress_bar(self.screen, bar_x, bar_y, bar_w, 5,
                                  level / max_level, color=lc)

            list_y_start = 88
            for i, game in enumerate(games):
                iy = list_y_start + i * item_h - int(scroll_y)
                if iy < list_y_start - 5 or iy > VIRT_H - 45:
                    continue
                is_sel = (i == selected)
                row_rect = (10, iy, VIRT_W - 20, item_h - 2)
                if is_sel:
                    draw_rounded_rect(self.screen, WII_BLUE, row_rect, 4)
                    nc = WHITE
                    hc = (200, 220, 240)
                elif i % 2 == 0:
                    draw_rounded_rect(self.screen, WII_PANEL, row_rect, 4)
                    nc = WII_TEXT
                    hc = WII_TEXT_LIGHT
                else:
                    nc = WII_TEXT
                    hc = WII_TEXT_LIGHT
                draw_clean_text(self.screen, game.name, self.fonts["med"],
                                nc, (18, iy + item_h // 2), center=False)
                hw = self.fonts["small"].size(game.hint)[0]
                draw_clean_text(self.screen, game.hint, self.fonts["small"],
                                hc, (VIRT_W - 18 - hw, iy + item_h // 2),
                                center=False)

            # Back button
            draw_rounded_rect(self.screen, DARK_GREY,
                              (back_rect.x, back_rect.y,
                               back_rect.w, back_rect.h), 4)
            draw_clean_text(self.screen, "< Back", self.fonts["small"],
                            WHITE, (back_rect.x + 18, back_rect.y + 8),
                            center=True)

            draw_clean_text(self.screen, "B1: Play    B2: Back",
                            self.fonts["small"], WII_TEXT_LIGHT,
                            (cx, VIRT_H - 14), center=True)

            self.flip()
            self.clock.tick(FPS)

    def run_practice(self, game_idx, speed_mult):
        game = ALL_GAMES[game_idx]
        speed_level = max(1, int((1.0 - speed_mult) / 0.05) + 1)
        wins = 0
        plays = 0

        self.all_leds_off()
        self.set_led("b", True)

        back_rect = pygame.Rect(2, 2, 36, 16)
        _prev_b2 = self.state.snapshot()["btn2_presses"]

        while True:
            snap = self.state.snapshot()
            game.reset(snap, speed_mult)

            self.show_instruction_screen(game)
            _prev_b2 = self.state.snapshot()["btn2_presses"]

            if game.wave_based:
                duration = game.base_duration
            else:
                duration = max(3.0, game.base_duration * speed_mult)
            start_time = time.time()
            result = None
            back_pressed = False

            while True:
                if self.check_quit():
                    self.set_led("b", False)
                    return

                dt = 1.0 / FPS
                elapsed = time.time() - start_time
                time_left = duration - elapsed

                if time_left <= 0:
                    result = (game.game_type == "survive")
                    break

                snap = self.state.snapshot()

                # Check B2 press to exit practice
                if snap["btn2_presses"] != _prev_b2:
                    back_pressed = True
                    break
                # Check touch back button
                for tx, ty in self.get_touches():
                    if back_rect.collidepoint(tx, ty):
                        back_pressed = True
                        break
                if back_pressed:
                    break

                check = game.update(snap, dt)
                if check is not None:
                    result = check
                    break

                self.screen.fill(game.bg_color)
                game.draw(self.screen, snap, time_left, duration, self.fonts)
                if not game.wave_based:
                    self.draw_timer(time_left, duration)
                draw_clean_text(self.screen, f"W:{wins}/{plays}",
                                self.fonts["small"], WII_TEXT_LIGHT,
                                (VIRT_W - 6, 6), center=False)
                # Draw back button
                draw_rounded_rect(self.screen, DARK_GREY,
                                  (back_rect.x, back_rect.y,
                                   back_rect.w, back_rect.h), 4)
                draw_clean_text(self.screen, "< Back", self.fonts["small"],
                                WHITE, (back_rect.x + 18, back_rect.y + 8),
                                center=True)
                self.flip()
                self.clock.tick(FPS)

            if back_pressed:
                self.set_led("b", False)
                return

            plays += 1
            if result:
                wins += 1
                self.led_win()
            else:
                self.led_lose()

            anim_start = time.time()
            anim_dur = game.RESULT_DURATION
            while time.time() - anim_start < anim_dur:
                if self.check_quit():
                    self.set_led("b", False)
                    return
                anim_t = time.time() - anim_start
                self.screen.fill(game.bg_color)
                game.animate_result(self.screen, result, anim_t, self.fonts)
                # Draw back button on result screen too
                draw_rounded_rect(self.screen, DARK_GREY,
                                  (back_rect.x, back_rect.y,
                                   back_rect.w, back_rect.h), 4)
                draw_clean_text(self.screen, "< Back", self.fonts["small"],
                                WHITE, (back_rect.x + 18, back_rect.y + 8),
                                center=True)
                # Check touch/B2 during result animation
                snap = self.state.snapshot()
                if snap["btn2_presses"] != _prev_b2:
                    self.set_led("b", False)
                    return
                _prev_b2 = snap["btn2_presses"]
                for tx, ty in self.get_touches():
                    if back_rect.collidepoint(tx, ty):
                        self.set_led("b", False)
                        return
                self.flip()
                self.clock.tick(FPS)

            # Brief pause then loop
            time.sleep(0.3)

    def game_over_screen(self, score, is_high):
        """Wii-style game over panel."""
        self.all_leds_off()
        cx = VIRT_W // 2
        start = time.time()
        duration = 4.0

        if is_high:
            scores = load_highscores()
            scores.append(score)
            save_highscores(scores)

        while time.time() - start < duration:
            if self.check_quit():
                return
            t = time.time() - start

            draw_wii_bg(self.screen)
            draw_wii_header(self.screen, "GAME OVER", self.fonts["big"],
                            color=WII_DANGER)

            # Score panel
            panel_y = 80
            draw_wii_panel(self.screen,
                           (25, panel_y, VIRT_W - 50, 70),
                           border_color=WII_GOLD if is_high else WII_BORDER)
            draw_clean_text(self.screen, "Final Score", self.fonts["med"],
                            WII_TEXT_LIGHT, (cx, panel_y + 15), center=True)
            draw_clean_text(self.screen, str(score), self.fonts["giant"],
                            WII_GOLD, (cx, panel_y + 45), center=True)

            if is_high:
                # New high score animation
                pulse = abs(math.sin(t * 4))
                hc = (int(220 * pulse), int(180 * pulse), int(40 * pulse))
                draw_clean_text(self.screen, "NEW HIGH SCORE!",
                                self.fonts["big"], hc,
                                (cx, panel_y + 80), center=True)
                # Sparkles
                for i in range(6):
                    a = t * 3 + i * math.pi / 3
                    d = 20 + abs(math.sin(t * 2 + i)) * 30
                    sx = cx + int(math.cos(a) * d)
                    sy = panel_y + 45 + int(math.sin(a) * d)
                    if 0 < sx < VIRT_W and 20 < sy < VIRT_H:
                        pygame.draw.rect(self.screen, WII_GOLD,
                                         (sx, sy, 2, 2))

            # High scores list
            hi_scores = load_highscores()
            if hi_scores:
                list_y = 190
                draw_wii_panel(self.screen,
                               (25, list_y, VIRT_W - 50, 90),
                               border_color=WII_BORDER)
                draw_clean_text(self.screen, "Top Scores",
                                self.fonts["med"], WII_TEXT,
                                (cx, list_y + 12), center=True)
                for i, sc in enumerate(hi_scores[:5]):
                    rank_c = [WII_GOLD, WII_TEXT, ORANGE,
                              WII_TEXT_LIGHT, WII_TEXT_LIGHT]
                    is_this = (sc == score and is_high)
                    ec = WII_GOLD if is_this and int(t * 3) % 2 else rank_c[i]
                    entry = f"{i + 1}.  {sc}"
                    draw_clean_text(self.screen, entry, self.fonts["small"],
                                    ec, (cx, list_y + 28 + i * 11),
                                    center=True)

            # Sad Mii
            mii_y = 320
            draw_mii(self.screen, cx, mii_y, size=16, happy=False)

            blink = int(t * 2) % 2
            if blink and t > 1.5:
                draw_clean_text(self.screen, "Press any button",
                                self.fonts["small"], WII_TEXT_LIGHT,
                                (cx, VIRT_H - 16), center=True)

            snap = self.state.snapshot()
            if t > 1.5 and (snap["btn1"] or snap["btn2"]):
                return

            self.flip()
            self.clock.tick(FPS)

    def run_arcade(self):
        """Main arcade loop — WarioWare style."""
        score = 0
        lives = 3
        speed_level = 1
        games_played = 0
        speed_mult = 1.0

        self.all_leds_off()
        self.flash_screen("GET READY!", WII_BLUE, BG, 1.0)

        available = list(range(len(ALL_GAMES)))
        random.shuffle(available)
        game_queue = list(available)

        while lives > 0:
            if self.check_quit():
                return

            if not game_queue:
                random.shuffle(available)
                game_queue = list(available)
            game_idx = game_queue.pop(0)
            game = ALL_GAMES[game_idx]

            snap = self.state.snapshot()
            game.reset(snap, speed_mult)

            # Speed up every 5 games
            games_played += 1
            if games_played % 5 == 0 and speed_level < 13:
                speed_level += 1
                speed_mult = max(0.4, 1.0 - (speed_level - 1) * 0.05)
                self.flash_screen(f"SPEED UP!", ORANGE, BG, 0.6,
                                  subtitle=f"Level {speed_level}")

            # Show instruction screen
            self.show_instruction_screen(game)

            # Run game
            if game.wave_based:
                duration = game.base_duration
            else:
                duration = max(3.0, game.base_duration * speed_mult)
            start_time = time.time()
            result = None

            while True:
                if self.check_quit():
                    return
                dt = 1.0 / FPS
                elapsed = time.time() - start_time
                time_left = duration - elapsed

                if time_left <= 0:
                    result = (game.game_type == "survive")
                    break

                snap = self.state.snapshot()
                check = game.update(snap, dt)
                if check is not None:
                    result = check
                    break

                self.screen.fill(game.bg_color)
                game.draw(self.screen, snap, time_left, duration, self.fonts)
                if not game.wave_based:
                    self.draw_timer(time_left, duration)
                self.draw_hud(score, lives, speed_level)
                self.flip()
                self.clock.tick(FPS)

            # Result
            if result:
                score += 1
                self.led_win()
            else:
                lives -= 1
                self.led_lose()

            # Animate result
            anim_start = time.time()
            anim_dur = game.RESULT_DURATION
            while time.time() - anim_start < anim_dur:
                if self.check_quit():
                    return
                anim_t = time.time() - anim_start
                self.screen.fill(game.bg_color)
                game.animate_result(self.screen, result, anim_t, self.fonts)
                self.flip()
                self.clock.tick(FPS)

            time.sleep(0.15)

        # Game over
        self.game_over_screen(score, is_highscore(score))

    def run(self):
        """Main entry point."""
        mode = self.run_menu()

        if mode == "serial":
            self.comm = SerialBackend(self.state, self.serial_port)
        elif mode == "wifi":
            self.comm = MQTTBackend(self.state, mode="wifi")
        else:
            self.comm = MQTTBackend(self.state, mode="lte")

        self.comm.start()

        if not self.wait_for_connection():
            return

        while True:
            choice = self.title_screen()
            if choice is False:
                break
            elif choice == "calibrate":
                self.run_calibration()
            elif choice == "arcade":
                self.run_arcade()
            elif choice == "practice":
                result = self.practice_select_screen()
                if result is not None:
                    game_idx, spd = result
                    self.run_practice(game_idx, spd)

        self.all_leds_off()
        if self.comm:
            self.comm.stop()
        pygame.quit()


# ═══════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PiWare Microgames")
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="Serial port for ESP32")
    args = parser.parse_args()

    engine = GameEngine(serial_port=args.port)
    engine.run()
