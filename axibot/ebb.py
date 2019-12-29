from __future__ import absolute_import, division, print_function, unicode_literals
from svgpathtools.paths2svg import big_bounding_box
import logging
import time
import math

import serial
from serial.tools.list_ports import comports
from svgpathtools import parse_path
import numpy as np

from . import config

log = logging.getLogger(__name__)


def percenter(coord1, coord2, coord3):
    px = (coord3[0] - coord1[0]) / float(abs(coord1[0] - coord2[0]))
    py = (coord3[1] - coord1[1]) / float(abs(coord1[1] - coord2[1]))
    return px, py


class EiBotException(Exception):
    pass

swirl = [{'down': 2}, {'left': 2}, {'up': 2}, {'right': 2}]


class EiBotBase:
    def do(self, move):
        kw = move.__dict__.copy()
        name = move.name
        if name in ("pen_up", "pen_down", "xy_accel_move", "xy_move", "ab_move"):
            method = getattr(self, name)
            return method(**kw)
        else:
            raise EiBotException("Don't know how to do move %r / %s" % (move, move))


class EiBotBoard(EiBotBase):
    def __init__(self, ser):
        self.serial = ser
        self.colors = {}
        self.workarea = {"top": 0, "left": 0, "bottom": 5375, "right": 3750}
        self.canvas = {'top': 5375-(3750+100), 'left': 100, 'bottom': 5375-100, 'right': 3750-100}
        self.colors = {
            "purple": [
                {"x": 15, "y": 5, "pen_start": 100, "pen_end": 70, "area": "workarea"},
                {"down": 2},
                {"left": 2},
                {"up": 2},
                {"right": 2},
                {"down": 2},
                {"left": 2},
                {"up": 2},
                {"right": 2},
                {"y": 10, "pen_start": 80, "pen_end": 100},
            ],
            "pink": [
                {"x": 15, "y": 15, "pen_start": 100, "pen_end": 70, "area": "workarea"},
                {"y": 10, "pen_start": 80, "pen_end": 100},
            ],
            "blue": [
                {"x": 25, "y": 5, "pen_start": 100, "pen_end": 70, "area": "workarea"},
                {"y": 10, "pen_start": 80, "pen_end": 100},
            ],
            "rinse": [
                {"x": 90, "y": 10, "pen_start": 100, "pen_end": 20, "area": "workarea"},
                {"down": 2},
                {"left": 2},
                {"up": 2},
                {"right": 2},
                {"down": 2},
                {"left": 2},
                {"up": 2},
                {"right": 2},
                {"down": 2},
                {"left": 2},
                {"up": 2},
                {"right": 2},
                {"down": 2},
                {"left": 2},
                {"up": 2},
                {"right": 2},
                {"left": 20, "pen_start": 60, "pen_end": 100},
            ],
            "black": [
                {"x": 5, "y": 5, "pen_start": 100, "pen_end": 70, "area": "workarea"},
                {"down": 2},
                {"left": 2},
                {"up": 2},
                {"right": 2},
                {"down": 2},
                {"left": 2},
                {"up": 2},
                {"right": 2},
                {"y": 10, "pen_start": 80, "pen_end": 100},
            ],
            "yellow": [
                {"x": 45, "y": 15, "pen_start": 100, "pen_end": 70, "area": "workarea"},
                {"down": 2},
                {"left": 2},
                {"up": 2},
                {"right": 2},
                {"down": 2},
                {"left": 2},
                {"up": 2},
                {"right": 2},
                {"y": 10, "pen_start": 80, "pen_end": 100},
            ],
            "orange": [
                {"x": 35, "y": 15, "pen_start": 100, "pen_end": 70, "area": "workarea"},
                {"down": 2},
                {"left": 2},
                {"up": 2},
                {"right": 2},
                {"down": 2},
                {"left": 2},
                {"up": 2},
                {"right": 2},
                {"y": 10, "pen_start": 80, "pen_end": 100},
            ],
            "red": [
                {"x": 45, "y": 15, "pen_start": 100, "pen_end": 70, "area": "workarea"},
                {"down": 2},
                {"left": 2},
                {"up": 2},
                {"right": 2},
                {"down": 2},
                {"left": 2},
                {"up": 2},
                {"right": 2},
                {"y": 10, "pen_start": 80, "pen_end": 100},
            ],
        }

        self.reset()

    def reset(self):
        self.disable_motors()
        input("rehomed?")
        self.enable_motors(4)
        self.STATE = {
            "x": 0,
            "y": 0,
            "pen_pct": 100,
            "distance": 0,
            "color": None,
            "area": "workarea",
            "color_distance": 0,
        }
        self.move(0, 0, pen_end=0, area="workarea")
        self.move(0, 0, pen_end=100, area="workarea")

    @classmethod
    def list_ports(cls):
        ports = comports()
        for port in ports:
            if port[1].startswith("EiBotBoard"):
                yield port[0]
            elif port[2].upper().startswith("USB VID:PID=04D8:FD92"):
                yield port[0]

    @classmethod
    def open(cls, port):
        ser = serial.Serial(port, timeout=1.0)
        # May need to try several times to get a response from the board?
        # This behavior is taken from the ebb_serial usage, not sure if it's
        # necessary.
        for attempt in range(3):
            ser.write(b"v\r")
            version = ser.readline()
            if version and version.startswith(b"EBB"):
                return cls(ser)

    @classmethod
    def find(cls):
        for port in cls.list_ports():
            if port:
                return cls.open(port)
        raise EiBotException("Could not find a connected EiBotBoard.")

    def close(self):
        # XXX Maybe switch to a context manger for this?
        self.serial.close()

    def robust_readline(self):
        for attempt in range(config.MAX_RETRIES):
            resp = self.serial.readline()
            if resp:
                return resp

    def query(self, cmd):
        self.serial.write(cmd.encode("ascii"))
        resp = self.robust_readline()
        if cmd.strip().lower() not in ("v", "i", "a", "mr", "pi", "qm"):
            # Discard response.
            self.robust_readline()
        return resp

    def command(self, cmd):
        cmd = cmd.encode("ascii")
        log.debug("Sending command: %s", cmd)
        resp = b""
        attemps = 0
        self.serial.write(cmd)
        while not resp.strip().startswith(b"OK"):
            attemps += 1
            resp += self.robust_readline().strip()
            # print("response:", resp)
            if attemps > 3:
                raise EiBotException(
                    "Unexpected response from EBB:\n"
                    "Command: %s\n"
                    "Response: %s" % (cmd.strip(), resp.strip())
                )

    def enable_motors(self, res):
        """
        Enable motors. Available resolutions:
            0, -> Motor disabled
            1, -> 16X microstepping
            2, -> 8X microstepping
            3, -> 4X microstepping
            4, -> 2X microstepping
            5, -> No microstepping
        """
        if res < 0:
            res = 0
        elif res > 5:
            res = 5
        self.command("EM,%s,%s\r" % (res, res))

    def disable_motors(self):
        self.command("EM,0,0\r")

    def query_prg_button(self):
        self.command("QB\r")

    def toggle_pen(self):
        self.command("TP\r")

    def servo_setup(
        self, pen_down_position, pen_up_position, servo_up_speed, servo_down_speed
    ):
        """
        Configure EBB servo control offsets and speeds.

        From the axidraw.py comments:

            "Pen position units range from 0% to 100%, which correspond to a
            typical timing range of 7500 - 25000 in units of 1/(12 MHz). 1%
            corresponds to ~14.6 us, or 175 units of 1/(12 MHz)."

            "Servo speed units are in units of %/second, referring to the
            percentages above.  The EBB takes speeds in units of 1/(12 MHz)
            steps per 24 ms.  Scaling as above, 1% of range in 1 second with
            SERVO_MAX = 28000  and  SERVO_MIN = 7500 corresponds to 205 steps
            change in 1 s That gives 0.205 steps/ms, or 4.92 steps / 24 ms
            Rounding this to 5 steps/24 ms is sufficient."
        """
        slope = float(config.SERVO_MAX - config.SERVO_MIN) / 100.0
        up_setting = int(round(config.SERVO_MIN + (slope * pen_up_position)))
        down_setting = int(round(config.SERVO_MIN + (slope * pen_down_position)))
        up_speed = 5 * servo_up_speed
        down_speed = 5 * servo_down_speed
        self.command("SC,4,%s\r" % up_setting)
        self.command("SC,5,%s\r" % down_setting)
        self.command("SC,11,%s\r" % up_speed)
        self.command("SC,12,%s\r" % down_speed)

    def pen_up(self, delay):
        self.command("SP,1,%s\r" % delay)

    def pen_down(self, delay):
        self.command("SP,0,%s\r" % delay)

    def xy_accel_move(self, dx, dy, v_initial, v_final):
        """
        Move X/Y axes as: "AM,<v_initial>,<v_final>,<axis1>,<axis2><CR>"
        Typically, this is wired up such that axis 1 is the Y axis and axis 2
        is the X axis of motion. On EggBot, Axis 1 is the "pen" motor, and Axis
        2 is the "egg" motor. Note that minimum move duration is 5 ms.
        Important: Requires firmware version 2.4 or higher.

        Not used in "stock" AxiDraw Inkscape driver: theoretically this could
        replace a substantial portion of the motion planning, and eliminate the
        plot_segment... planning, but there are some comments in firmware code
        and such that indicate that it doesn't work correctly yet.
        """
        self.command("AM,%s,%s,%s,%s\r" % (v_initial, v_final, dx, dy))

    def xy_move(self, m1, m2, duration):
        """
        Move M1/M2 axes as: "SM,<move_duration>,<axis1>,<axis2><CR>"

        Move duration is in milliseconds and can be 1 to 16777215.
        m1 and m2 are in steps and are signed 24-bit integers.

        On the AxiDraw, these axes are blended: the coordinate space of the
        control API is rotated 45 degrees from the coordinate space of the
        actual movement. It's unclear why this is.
        """
        assert isinstance(m1, int)
        assert isinstance(m2, int)
        assert isinstance(duration, int)
        assert (duration >= 1) and (duration <= 16777215), (
            "Invalid duration %r" % duration
        )

        # XXX add checks for minimum or maximum step rates

        self.command("SM,%s,%s,%s\r" % (duration, m1, m2))

    def ab_move(self, da, db, duration):
        """
        Issue command to move A/B axes
        as: "XM,<move_duration>,<axisA>,<axisB><CR>"
        Then, <Axis1> moves by <AxisA> + <AxisB>,
        and <Axis2> as <AxisA> - <AxisB>

        Not used in "stock" AxiDraw Inkscape driver.
        """
        self.command("XM,%s,%s,%s\r" % (duration, da, db))

    def pen(self, pct=100):
        if self.STATE["pen_pct"] != pct:
            self.servo_setup(0, pct, 400, 400)
            self.pen_up(300)
            self.STATE["pen_pct"] = pct

    def move(
        self,
        x=None,
        y=None,
        up=None,
        down=None,
        left=None,
        right=None,
        pen_start=None,
        pen_end=None,
        speed=1,
        area=None,
    ):
        if area:
            self.STATE["area"] = area
        dims = getattr(self, self.STATE["area"])

        if pen_start is not None:
            self.pen(pen_start)

        rel_x = 0
        rel_y = 0
        x_steps = self.STATE["x"]
        y_steps = self.STATE["y"]
        self.STATE["rel_x"] = x
        self.STATE["rel_y"] = y
        if x is None:
            if left is not None:
                x_steps = self.STATE["x"] - (
                    (left / 100) * (dims["right"] - dims["left"])
                )
            elif right is not None:
                x_steps = self.STATE["x"] + (
                    (right / 100) * (dims["right"] - dims["left"])
                )
        else:
            x_steps = ((x / 100) * (dims["right"] - dims["left"])) + dims["left"]
        rel_x = self.STATE["x"] - round(x_steps)
        if self.STATE["x"] - rel_x > self.workarea["right"]:
            raise ValueError("too far right:", self.STATE)
        if self.STATE["x"] - rel_x < self.workarea["left"]:
            raise ValueError("too far left:", self.STATE)
        self.STATE["x"] -= rel_x

        if y is None:
            if up is not None:
                y_steps = self.STATE["y"] - (
                    (up / 100) * (dims["bottom"] - dims["top"])
                )
            elif down is not None:
                y_steps = self.STATE["y"] + (
                    (down / 100) * (dims["bottom"] - dims["top"])
                )
        else:
            y_steps = ((y / 100) * (dims["bottom"] - dims["top"])) + dims["top"]
        rel_y = self.STATE["y"] - round(y_steps)
        if self.STATE["y"] - rel_y > self.workarea["bottom"]:
            raise ValueError("too far down:", self.STATE)
        if self.STATE["y"] - rel_y < self.workarea["top"]:
            raise ValueError("too far hight:", self.STATE)
        self.STATE["y"] -= rel_y

        dist = int((rel_x ** 2 + rel_y ** 2) ** 0.5)
        # print("move", x_steps, y_steps, "dist:", dist)
        if dist:
            self.STATE["distance"] += dist
            if self.STATE["pen_pct"] < 100:
                self.STATE["color_distance"] += dist

            self.ab_move(int(rel_x), int(rel_y), max(3, min(20000, int(dist / speed))))
        if pen_end is not None:
            self.pen(pen_end)

    def get_color(self, color, _return=False):
        if _return:
            old_pos = {
                "x": self.STATE["rel_x"],
                "y": self.STATE["rel_y"],
                "pen_start": 100,
                "pen_end": self.STATE["pen_pct"],
                "area": self.STATE["area"],
            }
        if color:
            for cords in self.colors[color]:
                self.move(**cords)
            self.pen(100)
            self.STATE["color_distance"] = 0
            self.STATE["color"] = color
            if _return:
                self.move(**old_pos)

    def draw_line(self, coords, color=None, max_color_distance=1000):
        if color:
            if color != self.STATE["color"]:
                print("rinse")
                self.get_color("rinse")
                print("get color:", color, "old_color:", self.STATE["color"])
                self.get_color(color)

        for c in coords:
            self.move(**c)
            last_c = c
            if color and self.STATE["color_distance"] > max_color_distance:
                print("refilling:", color, self.STATE["color_distance"])
                self.get_color(color, _return=True)

    def draw_svg_path(
        self, path, bbox=None, color=None, max_color_distance=1000, resolution=1
    ):
        last_x = None
        last_y = None
        path_alt = parse_path(path)
        xmin, xmax, ymin, ymax = bbox
        for p in path_alt.continuous_subpaths():
            steps = np.linspace(0, 100, max(2, int(p.length() / resolution))) / 100
            cmds = []
            for s in steps:
                px, py = percenter(
                    (xmin, ymin), (xmax, ymax), (p.point(s).real, p.point(s).imag)
                )
                cmds.append(
                    {"x": px * 100, "y": py * 100, "area": "canvas", "speed": 1}
                )
            if (
                self.STATE["rel_x"] != cmds[0]["x"]
                or self.STATE["rel_y"] != cmds[0]["y"]
            ):
                cmds[0]["pen_start"] = 100
                cmds[0]["pen_end"] = 0
            print("DRAWING:", cmds)
            self.draw_line(cmds, color=color, max_color_distance=max_color_distance)

    def draw_stroke_data(
        self, stroke_data, bbox, max_color_distance=1000, resolution=1
    ):
        xmin, xmax, ymin, ymax = bbox
        for stroke in stroke_data:
            cmds = []
            for p in stroke["points"]:
                px, py = percenter((xmin, ymin), (xmax, ymax), (p["x"], p["y"]))
                cmds.append(
                    {"x": px * 100, "y": py * 100, "area": "canvas", "speed": 1}
                )
            if (
                self.STATE["rel_x"] != cmds[0]["x"]
                or self.STATE["rel_y"] != cmds[0]["y"]
            ):
                cmds[0]["pen_start"] = 100
                cmds[0]["pen_end"] = 0
            print("DRAWING:", cmds)

            self.draw_line(
                cmds, color=stroke["color"], max_color_distance=max_color_distance
            )


class MockEiBotBoard(EiBotBase):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.last_speed = 0
        self.max_speed = 0
        self.max_acceleration = 0

    def close(self):
        pass

    def query(self, cmd):
        raise NotImplementedError("Mock EBB doesn't know how to handle queries.")

    def command(self, cmd):
        cmd = cmd.encode("ascii")
        log.debug("Sending command: %s", cmd)
        self.serial.write(cmd)
        resp = self.robust_readline()
        if not resp.strip().startswith(b"OK"):
            if resp:
                raise EiBotException(
                    "Unexpected response from EBB:\n"
                    "Command: %s\n"
                    "Response: %s" % (cmd.strip(), resp.strip())
                )

    def enable_motors(self, res):
        log.warn("Mock EBB: enable_motors")

    def disable_motors(self):
        log.warn("Mock EBB: disable_motors")

    def query_prg_button(self):
        log.warn("Mock EBB: query_prg_button")

    def toggle_pen(self):
        log.warn("Mock EBB: toggle_pen")

    def servo_setup(
        self, pen_down_position, pen_up_position, servo_up_speed, servo_down_speed
    ):
        log.warn("Mock EBB: servo_setup")

    def pen_up(self, delay):
        log.warn("Mock EBB: pen_up delay %s", delay)
        time.sleep(delay / 1000.0)

    def pen_down(self, delay):
        log.warn("Mock EBB: pen_down delay:%s", delay)
        time.sleep(delay / 1000.0)

    def xy_accel_move(self, dx, dy, v_initial, v_final):
        log.warn("Mock EBB: xy_accel_move / Cannot simulate delay.")

    def xy_move(self, m1, m2, duration):
        dx = m1 + m2
        dy = m1 - m2
        dist = math.sqrt((dx ** 2) + (dy ** 2))
        speed = dist / duration
        if speed > self.max_speed:
            self.max_speed = speed
        accel = speed - self.last_speed
        if accel > self.max_acceleration:
            self.max_acceleration = accel
        self.last_speed = speed
        self.x += dx
        self.y += dy
        log.warn(
            "Mock EBB: xy_move m1:%s m2:%s duration:%s -> %s, %s",
            m1,
            m2,
            duration,
            self.x,
            self.y,
        )
        time.sleep(duration / 1000.0)

    def ab_move(self, da, db, duration):
        log.warn("Mock EBB: ab_move")
        time.sleep(duration / 1000.0)
