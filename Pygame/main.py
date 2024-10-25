import pygame
import random
import sys
import os
import serial
import threading
from pykalman import KalmanFilter


class ClickGame:
    def __init__(self):
        pygame.init()
        self.width, self.height = (1200, 800)
        self.screen = pygame.display.set_mode(
            (self.width, self.height), pygame.RESIZABLE
        )
        pygame.display.set_caption("Click Game")
        self.clock = pygame.time.Clock()
        self.fps = 60

        self.num_circles = 10
        self.circle_radius = 50
        self.circles = []
        self.fragments = []
        self.score = 0
        self.button_pressed = False

        self.mouse_x, self.mouse_y = (
            self.width // 2,
            self.height // 2,
        )  # Initial crosshair coordinates
        self.threshold = 0.01  # Threshold to ignore small values
        self.speed_factor = 100  # Adjust the speed of the crosshair movement

        self.lock = threading.Lock()

        # Initialize serial and data reading stream
        try:
            self.ser = serial.Serial("/dev/ttyUSB0", 115200)
            print("Serial connection established successfully.")
        except Exception as e:
            print(f"Failed to establish serial connection: {e}")
            sys.exit(1)

        self.reading_thread = threading.Thread(target=self.read_serial_data)
        self.reading_thread.daemon = True
        self.reading_thread.start()
        print("Serial reading thread started.")

        # Kalman Filters for gx and gz
        self.kf_gx = KalmanFilter(initial_state_mean=0, n_dim_obs=1)
        self.kf_gx.transition_matrices = [1]
        self.kf_gx.observation_matrices = [1]
        self.kf_gx.transition_covariance = 1e-2
        self.kf_gx.observation_covariance = 1e-4
        self.kf_gx.initial_state_covariance = 1.0

        self.kf_gz = KalmanFilter(initial_state_mean=0, n_dim_obs=1)
        self.kf_gz.transition_matrices = [1]
        self.kf_gz.observation_matrices = [1]
        self.kf_gz.transition_covariance = 1e-2
        self.kf_gz.observation_covariance = 1e-4
        self.kf_gz.initial_state_covariance = 1.0

        # Read resource directory path
        asset_folder = "Pygame/assets"
        try:
            self.image = pygame.image.load(
                os.path.join(asset_folder, "images", "hong.png")
            ).convert_alpha()
            self.image = pygame.transform.scale(
                self.image, (self.circle_radius * 2, self.circle_radius * 2)
            )
            self.image = self.apply_circle_mask(self.image, self.circle_radius)
            print("Image Size:", self.image.get_size())
        except pygame.error as e:
            print(f"Unable to load image: {e}")
            sys.exit()

        self.create_circles()

    def read_serial_data(self):
        while True:
            try:
                if self.ser.in_waiting > 0:
                    # Read data as bytes and ignore decoding errors
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()

                    # DATAG data processing (variable velocity)
                    if line.startswith("DATAG"):
                        try:
                            _, gx_value, _, gz_value = line.split(",")
                            gx_value, gz_value = float(gx_value), float(gz_value)

                            # Update gx and gz values ​​using Kalman Filter
                            filtered_gx, _ = self.kf_gx.filter_update(
                                self.kf_gx.initial_state_mean,
                                self.kf_gx.initial_state_covariance,
                                observation=gx_value,
                            )
                            filtered_gz, _ = self.kf_gz.filter_update(
                                self.kf_gz.initial_state_mean,
                                self.kf_gz.initial_state_covariance,
                                observation=gz_value,
                            )

                            with self.lock:
                                self.gx = (
                                    filtered_gx[0].data[0]
                                    if hasattr(filtered_gx[0], "data")
                                    else filtered_gx[0]
                                )
                                self.gz = (
                                    filtered_gz[0].data[0]
                                    if hasattr(filtered_gz[0], "data")
                                    else filtered_gz[0]
                                )

                        except ValueError:
                            print(f"Skipping invalid data: {line}")
                            continue

                    # Handle BUTTON data (button state)
                    elif line.startswith("BUTTON"):
                        try:
                            _, button_state = line.split(",")

                            if button_state == "PRESSED":
                                if (
                                    not self.button_pressed
                                ):  # Only fire when button state changes
                                    self.handle_button_pressed()
                                    self.button_pressed = True
                            else:  # button_state == "RELEASED"
                                self.button_pressed = False

                        except ValueError:
                            print(f"Skipping invalid button data: {line}")
                            continue

            except Exception as e:
                print(f"Lỗi: {e}")
                # Close serial port and reopen if error occurs
                if self.ser and self.ser.is_open:
                    self.ser.close()
                    print("Closed serial port.")
                try:
                    self.ser = serial.Serial("/dev/ttyUSB0", 115200)
                    print("Reconnected to serial port.")
                except Exception as re:
                    print(f"Failed to reconnect to serial port: {re}")

    def update_mouse_position(self):
        # Update the position of the center according to the gx and gz values

        if abs(self.gz) > self.threshold:
            self.mouse_x += -self.gz * self.speed_factor
        if abs(self.gx) > self.threshold:
            self.mouse_y += self.gx * self.speed_factor

        # Make sure the coordinates do not exceed the screen size
        self.mouse_x = max(min(self.mouse_x, self.width), 0)
        self.mouse_y = max(min(self.mouse_y, self.height), 0)

    def handle_button_pressed(self):
        # Handle when button is pressed
        for circle in self.circles:
            if (self.mouse_x - circle["x"]) ** 2 + (
                self.mouse_y - circle["y"]
            ) ** 2 <= circle["radius"] ** 2:
                self.create_fragments(circle["x"], circle["y"], circle["radius"])
                self.circles.remove(circle)
                self.score += 1
                break

    def create_circle_surface(self, radius):
        surface = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
        pygame.draw.circle(surface, (255, 255, 255, 0), (radius, radius), radius)
        return surface

    def apply_circle_mask(self, image, radius):
        radius = int(radius)
        mask_surface = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
        pygame.draw.circle(mask_surface, (255, 255, 255, 255), (radius, radius), radius)

        mask = pygame.mask.from_surface(mask_surface)

        image_resized = pygame.transform.scale(image, (radius * 2, radius * 2))
        image_masked = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)

        for x in range(radius * 2):
            for y in range(radius * 2):
                if mask.get_at((x, y)):
                    image_masked.set_at((x, y), image_resized.get_at((x, y)))
                else:
                    image_masked.set_at((x, y), (0, 0, 0, 0))  # Transparent

        return image_masked

    def create_circles(self):
        self.circles = []
        for _ in range(self.num_circles):
            x = random.randint(self.circle_radius, self.width - self.circle_radius)
            y = random.randint(self.circle_radius, self.height - self.circle_radius)
            dx = random.uniform(-5, 5)
            dy = random.uniform(-5, 5)
            self.circles.append(
                {"x": x, "y": y, "dx": dx, "dy": dy, "radius": self.circle_radius}
            )

    def create_fragments(self, x, y, radius):
        for _ in range(10):
            fragment_dx = random.uniform(-5, 5)
            fragment_dy = random.uniform(-5, 5)
            fragment_radius = radius / 2
            self.fragments.append(
                CircleFragment(
                    x, y, fragment_dx, fragment_dy, fragment_radius, self.image
                )
            )

    def update_circles(self):
        for circle in self.circles:
            circle["x"] += circle["dx"]
            circle["y"] += circle["dy"]

            if (
                circle["x"] - circle["radius"] <= 0
                or circle["x"] + circle["radius"] >= self.width
            ):
                circle["dx"] *= -1
            if (
                circle["y"] - circle["radius"] <= 0
                or circle["y"] + circle["radius"] >= self.height
            ):
                circle["dy"] *= -1

    def update_fragments(self):
        self.fragments = [frag for frag in self.fragments if frag.radius > 1]
        for fragment in self.fragments:
            fragment.update()

    def draw_circles(self):
        for circle in self.circles:
            pygame.draw.circle(
                self.screen,
                (255, 255, 255),
                (int(circle["x"]), int(circle["y"])),
                circle["radius"],
            )

    def draw_fragments(self):
        for fragment in self.fragments:
            pygame.draw.circle(
                self.screen,
                (255, 255, 255),
                (int(fragment.x), int(fragment.y)),
                fragment.radius,
            )

    def draw(self):
        self.screen.fill((0, 0, 0))
        self.draw_circles()
        self.draw_fragments()
        pygame.draw.circle(
            self.screen,
            (255, 0, 0),
            (self.mouse_x, self.mouse_y),
            10,
        )
        pygame.display.flip()

    def run(self):
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            self.update_circles()
            self.update_fragments()
            self.update_mouse_position()
            self.draw()
            self.clock.tick(self.fps)


class CircleFragment:
    def __init__(self, x, y, dx, dy, radius, image):
        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy
        self.radius = radius
        self.image = pygame.transform.scale(image, (int(radius * 2), int(radius * 2)))

    def update(self):
        self.x += self.dx
        self.y += self.dy
        self.radius *= 0.98  # Size reduction speed
        if self.radius < 1:
            self.radius = 0


if __name__ == "__main__":
    game = ClickGame()
    game.run()
