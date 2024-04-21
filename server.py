import socket
import json
from adafruit_servokit import ServoKit
import time
import math

kit = ServoKit(channels=16)

# Define the listening address and port
UDP_IP = "0.0.0.0"
UDP_PORT = 9999

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the address and port
sock.bind((UDP_IP, UDP_PORT))

print("UDP Server started")

neck_pin = 0
right_arm_pin = 1
left_arm_pin = 2
plate_pin = 3

while True:
    # Receive data from the sender
    data, addr = sock.recvfrom(1024)

    # Decode the received data from bytes to string
    received_data = data.decode()

    # Parse the JSON data
    json_data = json.loads(received_data)

    print("Received:", json_data)
    kit.servo[left_arm_pin].angle = json_data["left_arm"]
    kit.servo[right_arm_pin].angle = json_data["right_arm"]
    kit.servo[neck_pin].angle = json_data["neck"]
    kit.servo[plate_pin].angle = json_data["plate"]
