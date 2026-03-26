#!/usr/bin/env python3
"""
Touchscreen test UI — MQTT over WiFi edition.
Same UI as the LTE version, but connects to broker via plain MQTT (port 1883).
"""

import os
import sys
import json
import time
import threading

os.environ["SDL_VIDEODRIVER"] = "kmsdrm"
os.environ.setdefault("SDL_FBDEV", "/dev/fb0")

import pygame
import paho.mqtt.client as paho_mqtt
import ssl as _ssl

from evdev import InputDevice, ecodes, list_devices
from mqtt_config import MQTT_CONFIG

# ── Display ───────────────────────────────────────────────
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


class State:
    def __init__(self):
        self.btn1 = False
        self.btn2 = False
        self.pot1 = 0
        self.pot2 = 0
        self.led_r = False
        self.led_g = False
        self.led_b = False
        self.connected = False
        self.lock = threading.Lock()

    def update_from_json(self, data):
        with self.lock:
            self.btn1  = bool(data.get("btn1", 0))
            self.btn2  = bool(data.get("btn2", 0))
            self.pot1  = int(data.get("pot1", 0))
            self.pot2  = int(data.get("pot2", 0))
            self.led_r = bool(data.get("ledR", 0))
            self.led_g = bool(data.get("ledG", 0))
            self.led_b = bool(data.get("ledB", 0))
            self.connected = True

    def snapshot(self):
        with self.lock:
            return (self.btn1, self.btn2, self.pot1, self.pot2,
                    self.led_r, self.led_g, self.led_b, self.connected)


def find_touch_device():
    for path in list_devices():
        dev = InputDevice(path)
        name = dev.name.lower()
        if "touch" in name or "ft5406" in name:
            return dev
    return None


def touch_thread(touch_dev, touch_queue):
    touch_x, touch_y = 0, 0
    for event in touch_dev.read_loop():
        if event.type == ecodes.EV_ABS:
            if event.code in (ecodes.ABS_MT_POSITION_X, ecodes.ABS_X):
                touch_x = event.value
            elif event.code in (ecodes.ABS_MT_POSITION_Y, ecodes.ABS_Y):
                touch_y = event.value
        elif event.type == ecodes.EV_KEY and event.code == ecodes.BTN_TOUCH:
            if event.value == 1:
                touch_queue.append((touch_x, touch_y))


def draw_rounded_rect(surface, colour, rect, radius=12):
    pygame.draw.rect(surface, colour, rect, border_radius=radius)


def main():
    cfg = MQTT_CONFIG
    device_id = cfg["device_id"]
    status_topic = f"{device_id}/testscreen/status"
    led_topics = {
        "R": f"{device_id}/testscreen/led/red",
        "G": f"{device_id}/testscreen/led/green",
        "B": f"{device_id}/testscreen/led/blue",
    }

    state = State()

    # ── MQTT setup (WSS on port 443) ──
    client_id = f"pi-testscreen-wifi-{int(time.time())}"
    mqtt = paho_mqtt.Client(client_id=client_id, transport="websockets")
    mqtt.tls_set(cert_reqs=_ssl.CERT_REQUIRED, tls_version=_ssl.PROTOCOL_TLS)
    mqtt.ws_set_options(path="/mqtt")
    mqtt.username_pw_set(cfg["username"], cfg["password"])

    def on_connect(client, ud, flags, rc):
        if rc == 0:
            client.subscribe(status_topic)
            print(f"[MQTT] Connected, subscribed to {status_topic}")
        else:
            print(f"[MQTT] Connect failed rc={rc}")

    def on_message(client, ud, msg):
        try:
            data = json.loads(msg.payload.decode())
            state.update_from_json(data)
        except Exception:
            pass

    mqtt.on_connect = on_connect
    mqtt.on_message = on_message
    mqtt.connect(cfg["broker"], cfg["port"], 60)
    mqtt.loop_start()

    # ── Touch input ──
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

    # ── Pygame ──
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H), pygame.FULLSCREEN)
    clock = pygame.time.Clock()

    font_big   = pygame.font.SysFont("sans", 42, bold=True)
    font_med   = pygame.font.SysFont("sans", 32)
    font_small = pygame.font.SysFont("sans", 24)
    font_title = pygame.font.SysFont("sans", 52, bold=True)

    btn_w, btn_h = 600, 100
    gap = 20
    x0 = (SCREEN_W - btn_w) // 2
    y_led = 750

    led_buttons = {
        "R": pygame.Rect(x0, y_led,               btn_w, btn_h),
        "G": pygame.Rect(x0, y_led + btn_h + gap, btn_w, btn_h),
        "B": pygame.Rect(x0, y_led + 2*(btn_h+gap), btn_w, btn_h),
    }

    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                running = False

        while touch_queue:
            raw_x, raw_y = touch_queue.pop(0)
            pos = (int(raw_x * SCREEN_W / touch_res_x),
                   int(raw_y * SCREEN_H / touch_res_y))
            for cmd, rect in led_buttons.items():
                if rect.collidepoint(pos):
                    snap = state.snapshot()
                    led_states = {"R": snap[4], "G": snap[5], "B": snap[6]}
                    new_val = "OFF" if led_states[cmd] else "ON"
                    mqtt.publish(led_topics[cmd], new_val)

        (btn1, btn2, pot1, pot2,
         led_r, led_g, led_b, connected) = state.snapshot()

        screen.fill(BG)

        # ── Title ──
        title = font_title.render("Component Test (WiFi)", True, WHITE)
        screen.blit(title, (SCREEN_W // 2 - title.get_width() // 2, 20))

        if not connected:
            msg = font_med.render("Waiting for ESP32 MQTT data...", True, GREY)
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

    mqtt.loop_stop()
    mqtt.disconnect()
    pygame.quit()


if __name__ == "__main__":
    main()
