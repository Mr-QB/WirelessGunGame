import serial
import pygame
import sys
import threading
import numpy as np
from pykalman import KalmanFilter

# Configure serial port
ser = serial.Serial("/dev/ttyUSB0", 115200)  # Baud rate is 115200

# Initialize Pygame
pygame.init()
# screen = pygame.display.set_mode(
#     (0, 0), pygame.FULLSCREEN
# )  # Pygame window in full-screen mode
screen = pygame.display.set_mode((800, 600))  # Pygame window in normal mode
pygame.display.set_caption("Mouse Control with MPU6050")

# Get screen size
screen_width, screen_height = pygame.display.get_surface().get_size()

# Initial mouse coordinates
mouse_x = screen_width // 2
mouse_y = screen_height // 2

# Mouse movement speed
speed_factor = 100  # Adjust the speed to increase smoothness

# Threshold to ignore small velocity values (reduce drift)
threshold = 0.01

# Initialize Kalman Filter for gx and gz
kf_gx = KalmanFilter(initial_state_mean=0, n_dim_obs=1)
kf_gx.transition_matrices = [1]
kf_gx.observation_matrices = [1]
kf_gx.transition_covariance = 1e-2
kf_gx.observation_covariance = 1e-4
kf_gx.initial_state_covariance = 1.0

kf_gz = KalmanFilter(initial_state_mean=0, n_dim_obs=1)
kf_gz.transition_matrices = [1]
kf_gz.observation_matrices = [1]
kf_gz.transition_covariance = 1e-2
kf_gz.observation_covariance = 1e-4
kf_gz.initial_state_covariance = 1.0

# Lock object for data processing thread
lock = threading.Lock()

# Variables to store gx and gz values
gx, gz = 0, 0


def read_serial():
    global gx, gz
    while True:
        try:
            if ser.in_waiting > 0:
                # Read data as bytes and ignore decoding errors
                line = ser.readline().decode("utf-8", errors="ignore").strip()

                # Process DATAG data (velocity changes)
                if line.startswith("DATAG"):
                    try:
                        _, gx_value, _, gz_value = line.split(",")
                        gx_value, gz_value = float(gx_value), float(gz_value)

                        # Update gx and gz values using Kalman Filter
                        global kf_gx, kf_gz
                        filtered_gx, _ = kf_gx.filter_update(
                            kf_gx.initial_state_mean,
                            kf_gx.initial_state_covariance,
                            observation=gx_value,
                        )
                        filtered_gz, _ = kf_gz.filter_update(
                            kf_gz.initial_state_mean,
                            kf_gz.initial_state_covariance,
                            observation=gz_value,
                        )

                        with lock:
                            gx = filtered_gx[0]
                            gz = filtered_gz[0]

                    except ValueError:
                        # Skip conversion error if data is not formatted correctly
                        continue
        except Exception as e:
            print(f"Error: {e}")


# Create a thread to read serial data
serial_thread = threading.Thread(target=read_serial)
serial_thread.daemon = True  # Ensure the thread stops when the program ends
serial_thread.start()

# Main loop for Pygame
clock = pygame.time.Clock()

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Get gx and gz values from the serial reading thread
    with lock:
        current_gx, current_gz = gx, gz

    # Update mouse coordinates only if velocity exceeds the threshold
    if abs(current_gx) > threshold:
        mouse_x += -current_gz * speed_factor
    if abs(current_gz) > threshold:
        mouse_y += current_gx * speed_factor

    # Ensure the mouse doesn't go off the screen
    mouse_x = max(0, min(screen_width, mouse_x))
    mouse_y = max(0, min(screen_height, mouse_y))

    # Update the screen
    screen.fill((0, 0, 0))  # Clear the screen
    pygame.draw.circle(
        screen, (255, 0, 0), (int(mouse_x), int(mouse_y)), 10
    )  # Draw a circle representing the mouse
    pygame.display.update()

    # Limit frame rate (60 FPS)
    clock.tick(60)
