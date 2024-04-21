# sudo pip3 install adafruit-circuitpython-servokit
# pip3 install --index-url https://pypi.org/simple pygame
import json
import math
import socket

import pygame

# Initialize pygame
pygame.init()

# Initialize the joystick module
pygame.joystick.init()

# Check for connected joysticks
joystick_count = pygame.joystick.get_count()

pi_ip = "192.168.178.193"
port = 9999
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

arm_max_angle = 180.
neck_max_angle = 175.
neck_min_angle = 5.
plate_min_angle = 45.
plate_max_angle = 135.
delay_ms = 6


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

if joystick_count == 0:
    raise Exception("No joystick detected.")
else:
    # Get the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Print joystick details
    print("Joystick Name:", joystick.get_name())
    print("Number of Axes:", joystick.get_numaxes())


    try:
        # Main loop to read joystick input
        while True:
            pygame.event.pump()  # Pump events to get the latest input
            positions = []
            positions_str = []

            # Read stick positions
            for i in range(joystick.get_numaxes()):
                axis_value = joystick.get_axis(i)
                positions.append(axis_value)
                positions_str.append(f"{axis_value:.2f}")

            left_x = positions[0]
            left_y = -positions[1]
            right_x = positions[2]
            right_y = -positions[3]

            left_angle = math.atan2(left_y, left_x)
            left_distance = min(math.sqrt(left_x**2 + left_y**2), 1.)
            right_angle = math.atan2(right_y, right_x)
            right_distance = min(math.sqrt(right_x**2 + right_y**2), 1.)

            left_arm_max = 0
            right_arm_max = 0
            if math.pi/2 <= left_angle < math.pi:
                left_arm_max = 1.
                right_arm_max = 1 - (left_angle - math.pi / 2) * 2 / math.pi
            elif 0 <= left_angle < math.pi / 2:
                left_arm_max = left_angle * 2 / math.pi
                right_arm_max = 1.
            elif -math.pi/2 <= left_angle < 0:
                left_arm_max = 0
                right_arm_max = 1+left_angle * 2 / math.pi
            else:
                left_arm_max = -1 - left_angle * 2 / math.pi
                right_arm_max = 0

            left_arm_angle = arm_max_angle - arm_max_angle * left_distance * left_arm_max
            right_arm_angle = arm_max_angle * left_distance * right_arm_max
            neck_angle = neck_min_angle + (neck_max_angle - neck_min_angle) * (clamp(right_y, -1, 1) + 1) / 2
            plate_angle = plate_min_angle + (plate_max_angle - plate_min_angle) * (clamp(right_x, -1, 1) + 1) / 2

            data = {"left_arm": left_arm_angle, "right_arm": right_arm_angle, "neck": neck_angle, "plate": 90.}
            json_data = json.dumps(data)
            sock.sendto(json_data.encode(), (pi_ip, port))

            # print(f"left_angle: {left_angle}, left_distance: {left_distance}, right_angle: {right_angle}, right_distance: {right_distance}")
            print(f"left_arm_angle: {left_arm_angle}, right_arm_angle: {right_arm_angle}")
            pygame.time.wait(delay_ms)

    except KeyboardInterrupt:
        pass

pygame.quit()
