#!/usr/bin/env python3
"""
Touchscreen test UI for Raspberry Pi (headless / no desktop).
Renders to the framebuffer via SDL2 kmsdrm and reads touch via evdev.

Displays button states, potentiometer values from the ESP32
and provides touch buttons to toggle each LED.

Serial protocol (115200 baud):
  ESP32 → Pi:  S,btn1,btn2,pot1,pot2,ledR,ledG,ledB\n
  Pi → ESP32:  R | G | B  (toggle red / green / blue)

Usage (from SSH):
  sudo python3 testscreen.py [--port /dev/ttyUSB0]
"""

import os
import sys
import time
import argparse
import threading

# Configure SDL for framebuffer before importing pygame
os.environ["SDL_VIDEODRIVER"] = "kmsdrm"
os.environ.setdefault("SDL_FBDEV", "/dev/fb0")

import pygame
import serial

from evdev import InputDevice, ecodes, list_devices

# ── Defaults ──────────────────────────────────────────────
DEFAULT_PORT = "/dev/ttyUSB0"
BAUD = 115200
SCREEN_W, SCREEN_H = 720, 1280

# ── Colours ───────────────────────────────────────────────
BG        = (30, 30, 30)
WHITE     = (255, 255, 255)
GREY      = (100, 100, 100)
DARK_GREY = (60, 60, 60)
RED       = (244, 67, 54)
GREEN     = (76, 175, 80)
BLUE      = (33, 150, 243)
DIM_RED   = (80, 30, 25)
DIM_GREEN = (30, 65, 35)
DIM_BLUE  = (20, 55, 80)
PRESSED   = (76, 175, 80)
RELEASED  = (100, 100, 100)


class State:
    """Shared state updated from the serial thread."""
    def __init__(self):
        self.btn1 = False
        self.btn2 = False
        self.pot1 = 0
        self.pot2 = 0
        self.led_r = False
        self.led_g = False
        self.led_b = False
        self.ac_x = 0.0
        self.ac_y = 0.0
        self.ac_z = 0.0
        self.gy_x = 0.0
        self.gy_y = 0.0
        self.gy_z = 0.0
        self.connected = False
        self.lock = threading.Lock()

    def update(self, parts):
        with self.lock:
            self.btn1  = parts[0] == "1"
            self.btn2  = parts[1] == "1"
            self.pot1  = int(parts[2])
            self.pot2  = int(parts[3])
            self.led_r = parts[4] == "1"
            self.led_g = parts[5] == "1"
            self.led_b = parts[6] == "1"
            if len(parts) >= 13:
                self.ac_x = float(parts[7])
                self.ac_y = float(parts[8])
                self.ac_z = float(parts[9])
                self.gy_x = float(parts[10])
                self.gy_y = float(parts[11])
                self.gy_z = float(parts[12])
            self.connected = True

    def snapshot(self):
        with self.lock:
            return (self.btn1, self.btn2, self.pot1, self.pot2,
                    self.led_r, self.led_g, self.led_b,
                    self.ac_x, self.ac_y, self.ac_z,
                    self.gy_x, self.gy_y, self.gy_z, self.connected)


def serial_thread(port, state, ser_ref):
    """Read status lines from ESP32 forever."""
    while True:
        try:
            ser = serial.Serial(port, BAUD, timeout=1)
            ser_ref[0] = ser
            while True:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith("S,"):
                    parts = line[2:].split(",")
                    if len(parts) >= 7:
                        state.update(parts)
        except Exception:
            state.connected = False
            ser_ref[0] = None
            time.sleep(2)


def find_touch_device():
    """Scan /dev/input for the touchscreen device."""
    for path in list_devices():
        dev = InputDevice(path)
        name = dev.name.lower()
        if "touch" in name or "ft5406" in name:
            return dev
    return None


def touch_thread(touch_dev, touch_queue):
    """Read touch events and push tap positions into a queue."""
    touch_x, touch_y = 0, 0
    for event in touch_dev.read_loop():
        if event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_MT_POSITION_X or event.code == ecodes.ABS_X:
                touch_x = event.value
            elif event.code == ecodes.ABS_MT_POSITION_Y or event.code == ecodes.ABS_Y:
                touch_y = event.value
        elif event.type == ecodes.EV_KEY and event.code == ecodes.BTN_TOUCH:
            if event.value == 1:  # finger down
                touch_queue.append((touch_x, touch_y))


def draw_rounded_rect(surface, colour, rect, radius=12):
    pygame.draw.rect(surface, colour, rect, border_radius=radius)


def main():
    parser = argparse.ArgumentParser(description="Touchscreen component tester")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port")
    args = parser.parse_args()

    state = State()
    ser_ref = [None]

    t = threading.Thread(target=serial_thread, args=(args.port, state, ser_ref),
                         daemon=True)
    t.start()

    # Find and start touch input thread
    touch_dev = find_touch_device()
    touch_queue = []
    touch_res_x, touch_res_y = SCREEN_W, SCREEN_H
    if touch_dev:
        caps = touch_dev.capabilities(absinfo=True)
        for code, absinfo in caps.get(ecodes.EV_ABS, []):
            if code in (ecodes.ABS_MT_POSITION_X, ecodes.ABS_X):
                touch_res_x = absinfo.max
            elif code in (ecodes.ABS_MT_POSITION_Y, ecodes.ABS_Y):
                touch_res_y = absinfo.max
        tt = threading.Thread(target=touch_thread, args=(touch_dev, touch_queue),
                              daemon=True)
        tt.start()
    else:
        print("Warning: No touchscreen found, touch input disabled.")

    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H), pygame.FULLSCREEN)
    pygame.display.set_caption("Component Test")
    clock = pygame.time.Clock()

    font_big   = pygame.font.SysFont("sans", 42, bold=True)
    font_med   = pygame.font.SysFont("sans", 32)
    font_small = pygame.font.SysFont("sans", 24)
    font_title = pygame.font.SysFont("sans", 52, bold=True)

    # LED toggle button rectangles (portrait layout: stack vertically)
    btn_w, btn_h = 600, 100
    gap = 20
    x0 = (SCREEN_W - btn_w) // 2
    y_led = 750

    led_buttons = {
        "R": pygame.Rect(x0, y_led,                 btn_w, btn_h),
        "G": pygame.Rect(x0, y_led + btn_h + gap,   btn_w, btn_h),
        "B": pygame.Rect(x0, y_led + 2*(btn_h+gap), btn_w, btn_h),
    }

    running = True
    while running:
        # Handle pygame events (keyboard quit)
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                running = False

        # Handle touch events from evdev
        while touch_queue:
            raw_x, raw_y = touch_queue.pop(0)
            # Map touch coordinates to screen coordinates
            pos = (int(raw_x * SCREEN_W / touch_res_x),
                   int(raw_y * SCREEN_H / touch_res_y))
            for cmd, rect in led_buttons.items():
                if rect.collidepoint(pos) and ser_ref[0]:
                    try:
                        ser_ref[0].write(cmd.encode())
                    except Exception:
                        pass

        (btn1, btn2, pot1, pot2,
         led_r, led_g, led_b,
         ac_x, ac_y, ac_z, gy_x, gy_y, gy_z, connected) = state.snapshot()

        screen.fill(BG)

        # ── Title ──
        title = font_title.render("Component Test", True, WHITE)
        screen.blit(title, (SCREEN_W // 2 - title.get_width() // 2, 20))

        if not connected:
            msg = font_med.render("Waiting for ESP32 serial connection...", True, GREY)
            screen.blit(msg, (SCREEN_W // 2 - msg.get_width() // 2, SCREEN_H // 2))
            pygame.display.flip()
            clock.tick(30)
            continue

        # ── Buttons section ──
        section_y = 100
        lbl = font_med.render("Buttons", True, WHITE)
        screen.blit(lbl, (SCREEN_W // 2 - lbl.get_width() // 2, section_y))

        for i, (name, pressed) in enumerate([("Button 1", btn1), ("Button 2", btn2)]):
            bx = SCREEN_W // 2 - 280 + i * 280
            by = section_y + 50
            col = PRESSED if pressed else DARK_GREY
            draw_rounded_rect(screen, col, pygame.Rect(bx, by, 240, 80))
            txt = font_med.render(name, True, WHITE)
            screen.blit(txt, (bx + 120 - txt.get_width() // 2, by + 8))
            st = font_small.render("PRESSED" if pressed else "RELEASED", True, WHITE)
            screen.blit(st, (bx + 120 - st.get_width() // 2, by + 48))

        # ── Potentiometers section ──
        section_y = 300
        lbl = font_med.render("Potentiometers", True, WHITE)
        screen.blit(lbl, (SCREEN_W // 2 - lbl.get_width() // 2, section_y))

        bar_w = 580
        bar_h = 30
        bar_x = (SCREEN_W - bar_w) // 2

        for i, (name, val) in enumerate([("POT 1", pot1), ("POT 2", pot2)]):
            by = section_y + 50 + i * 100
            pct = min(val / 4095.0, 1.0)
            pct_txt = f"{int(pct * 100)}%"

            txt = font_small.render(f"{name}: {val}", True, WHITE)
            screen.blit(txt, (bar_x, by))

            pct_lbl = font_small.render(pct_txt, True, WHITE)
            screen.blit(pct_lbl, (bar_x + bar_w - pct_lbl.get_width(), by))

            bar_y = by + 35
            draw_rounded_rect(screen, DARK_GREY, pygame.Rect(bar_x, bar_y, bar_w, bar_h), 6)
            fill_w = max(int(bar_w * pct), 1)
            draw_rounded_rect(screen, BLUE, pygame.Rect(bar_x, bar_y, fill_w, bar_h), 6)

        # ── IMU section ──
        section_y = 530
        lbl = font_med.render("IMU (MPU-6050)", True, WHITE)
        screen.blit(lbl, (SCREEN_W // 2 - lbl.get_width() // 2, section_y))
        imu_y = section_y + 45
        accel_txt = font_small.render(
            f"Accel (g)   X:{ac_x:+.2f}  Y:{ac_y:+.2f}  Z:{ac_z:+.2f}", True, WHITE)
        screen.blit(accel_txt, (bar_x, imu_y))
        gyro_txt = font_small.render(
            f"Gyro (\u00b0/s)  X:{gy_x:+.1f}  Y:{gy_y:+.1f}  Z:{gy_z:+.1f}", True, WHITE)
        screen.blit(gyro_txt, (bar_x, imu_y + 35))

        # ── LED toggle buttons ──
        section_y = 690
        lbl = font_med.render("LEDs", True, WHITE)
        screen.blit(lbl, (SCREEN_W // 2 - lbl.get_width() // 2, section_y))

        led_info = [
            ("R", "RED",   led_r, RED,   DIM_RED),
            ("G", "GREEN", led_g, GREEN, DIM_GREEN),
            ("B", "BLUE",  led_b, BLUE,  DIM_BLUE),
        ]
        for cmd, name, is_on, col_on, col_off in led_info:
            rect = led_buttons[cmd]
            col = col_on if is_on else col_off
            draw_rounded_rect(screen, col, rect)
            pygame.draw.rect(screen, col_on, rect, width=3, border_radius=12)
            txt = font_big.render(name, True, WHITE)
            screen.blit(txt, (rect.x + 40, rect.centery - txt.get_height() // 2))
            st = font_med.render("ON" if is_on else "OFF", True, WHITE)
            screen.blit(st, (rect.right - st.get_width() - 40,
                             rect.centery - st.get_height() // 2))

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()


if __name__ == "__main__":
    main()
