import sys
import serial
import serial.tools.list_ports
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

# ================= Configuration =================
BAUD_RATE = 115200
DEFAULT_PORT = "COM6"


# =================================================

def open_serial():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[WARN] No active COM ports found. Running in visual-test mode.")
        return None

    target_port = None
    for p in ports:
        desc = p.description.lower()
        if any(chip in desc for chip in ["cp210", "ch340", "uart", "serial"]):
            target_port = p.device
            break

    if not target_port:
        available_names = [p.device for p in ports]
        target_port = DEFAULT_PORT if DEFAULT_PORT in available_names else available_names[0]

    print(f"Connecting to ESP32 on {target_port}...")
    try:
        ser = serial.Serial(target_port, BAUD_RATE, timeout=0.02)
        print(f"[OK] Connected successfully to {target_port}")
        print("[TIP] Press SPACEBAR at any time to re-align Yaw to 0°!")
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {target_port}: {e}")
        print("Tip: Make sure the Arduino Serial Monitor is completely closed!")
        return None


def parse_telemetry(line):
    """
    Parses format: Roll:0.00,Pitch:0.00,Yaw:0.00
    """
    try:
        parts = line.strip().split(',')
        data = {}
        for part in parts:
            if ':' in part:
                k, v = part.split(':', 1)
                data[k.strip()] = float(v.strip())
        if 'Roll' in data and 'Pitch' in data and 'Yaw' in data:
            return data['Roll'], data['Pitch'], data['Yaw'], True
        return 0.0, 0.0, 0.0, False
    except Exception:
        return 0.0, 0.0, 0.0, False


def draw_f450_frame():
    glLineWidth(4.0)

    # 1. Quadcopter X-Arms
    glBegin(GL_LINES)
    glColor3f(1.0, 0.2, 0.2)  # Front-Right (Red)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.42, 0.42, 0.0)

    glColor3f(1.0, 0.2, 0.2)  # Front-Left (Red)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(-0.42, 0.42, 0.0)

    glColor3f(0.85, 0.85, 0.85)  # Rear-Right (White)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.42, -0.42, 0.0)

    glColor3f(0.85, 0.85, 0.85)  # Rear-Left (White)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(-0.42, -0.42, 0.0)
    glEnd()

    # 2. Center Carbon Plate
    glLineWidth(2.0)
    glBegin(GL_LINE_LOOP)
    glColor3f(0.0, 0.8, 1.0)
    glVertex3f(0.13, 0.13, 0.02)
    glVertex3f(-0.13, 0.13, 0.02)
    glVertex3f(-0.13, -0.13, 0.02)
    glVertex3f(0.13, -0.13, 0.02)
    glEnd()

    # 3. Motor Rotors
    motors = [(0.42, 0.42, 0.0), (0.42, -0.42, 0.0), (-0.42, -0.42, 0.0), (-0.42, 0.42, 0.0)]
    for mx, my, mz in motors:
        glPushMatrix()
        glTranslatef(mx, my, mz)
        glColor3f(0.2, 0.95, 0.3)
        glBegin(GL_LINE_LOOP)
        for i in range(20):
            theta = 2.0 * 3.1415926 * i / 20
            v = pygame.math.Vector2(0.17, 0).rotate_rad(theta)
            glVertex3f(v.x, v.y, 0.0)
        glEnd()
        glPopMatrix()

    # 4. Heading Indicator (Front Yellow Arrow)
    glLineWidth(3.5)
    glBegin(GL_LINES)
    glColor3f(1.0, 0.9, 0.0)
    glVertex3f(0.0, 0.13, 0.03)
    glVertex3f(0.0, 0.50, 0.03)
    glEnd()


def draw_ground_grid():
    glLineWidth(1.0)
    glColor3f(0.20, 0.20, 0.24)
    glBegin(GL_LINES)
    for i in range(-6, 7):
        glVertex3f(i * 0.4, -2.4, 0.0)
        glVertex3f(i * 0.4, 2.4, 0.0)
        glVertex3f(-2.4, i * 0.4, 0.0)
        glVertex3f(2.4, i * 0.4, 0.0)
    glEnd()


def main():
    pygame.init()
    display = (1000, 750)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    pygame.display.set_caption("Prora Flight Controller — 3D Attitude Visualizer (Press SPACE to Zero Yaw)")

    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glMatrixMode(GL_MODELVIEW)
    glEnable(GL_DEPTH_TEST)

    ser = open_serial()
    clock = pygame.time.Clock()

    roll, pitch, yaw = 0.0, 0.0, 0.0
    frame_count = 0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                running = False
            # Press SPACEBAR to instantly re-center Yaw to 0°
            if event.type == KEYDOWN and event.key == K_SPACE:
                if ser:
                    ser.write(b'z')
                yaw = 0.0

        # Drain serial buffer to always render the newest attitude packet with zero lag
        if ser:
            last_line = None
            while ser.in_waiting > 0:
                try:
                    last_line = ser.readline().decode('utf-8', errors='ignore')
                except Exception:
                    break

            if last_line:
                r, p, y, ok = parse_telemetry(last_line)
                if ok:
                    roll, pitch, yaw = r, p, y

        frame_count += 1
        if frame_count % 6 == 0:
            pygame.display.set_caption(
                f"Prora 3D [SPACE = Zero Yaw] | Roll: {roll:6.1f}° | Pitch: {pitch:6.1f}° | Yaw: {yaw:6.1f}°"
            )

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        # Camera positioned cleanly behind and above the hovering drone
        gluLookAt(0.0, -1.9, 1.15,
                  0.0, 0.0, 0.30,
                  0.0, 0.0, 1.0)

        draw_ground_grid()

        glPushMatrix()

        # Fixed hover height above grid so rotations are always centered on screen
        glTranslatef(0.0, 0.0, 0.35)

        # Intrinsic Z-X-Y Euler rotation sequence (Yaw -> Pitch -> Roll)
        # Positive +yaw matches standard right-hand rule around Z-up
        glRotatef(yaw, 0.0, 0.0, 1.0)
        glRotatef(-pitch, 1.0, 0.0, 0.0)
        glRotatef(roll, 0.0, 1.0, 0.0)

        draw_f450_frame()
        glPopMatrix()

        pygame.display.flip()
        clock.tick(60)

    if ser:
        ser.close()
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()