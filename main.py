import pygame
import sys
import random
import numpy as np

WIDTH, HEIGHT = 800, 800

# Define colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)

# Define simulation constants
TICK_MS: int = 10
MAX_ACCELERATION: float = 1e-5

def get_1d_acceleration(target: float, position: float, velocity: float) -> float:
    direction = 1 if target > position else -1

    distance_to_target = target - position
    abs_distance_decelerate = 0.5 * (velocity ** 2) / MAX_ACCELERATION

    if velocity * distance_to_target >= 0 and abs(distance_to_target) <= abs_distance_decelerate:
        return -direction * MAX_ACCELERATION
    else:
        return direction * MAX_ACCELERATION

class Control:
    target: np.ndarray
    position: np.ndarray
    velocity: np.ndarray

    def __init__(self, position: np.ndarray, target: np.ndarray, velocity: np.ndarray):
        self.position = position
        self.target = target
        self.velocity = velocity

    def draw(self, screen):
        color = WHITE

        if np.linalg.norm(self.velocity) > 0.0001:
            unit_to_target = self.target - self.position
            unit_to_target /= np.linalg.norm(unit_to_target)
            unit_velocity = self.velocity / np.linalg.norm(self.velocity)
            color = GREEN if np.dot(unit_to_target, unit_velocity) > 0 else BLUE

        pygame.draw.circle(screen, color, (self.position[0] * WIDTH, self.position[1] * HEIGHT), radius=5)
        pygame.draw.circle(screen, RED, (self.target[0] * WIDTH, self.target[1] * HEIGHT), radius=2)

    def update(self):
        self.velocity[0] += get_1d_acceleration(target=self.target[0], velocity=self.velocity[0], position=self.position[0])
        self.velocity[1] += get_1d_acceleration(target=self.target[1], velocity=self.velocity[1], position=self.position[1])

        self.position[0] += self.velocity[0]
        self.position[1] += self.velocity[1]


def main():
    # Initialize Pygame
    pygame.init()

    # Set up the screen
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Control to Target")

    # Instantiate control
    control = Control(position=np.random.uniform(size=2), target=np.random.uniform(size=2), velocity=np.random.uniform(high=0.001, size=2))

    # Main game loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update control
        control.target = np.array([pygame.mouse.get_pos()[0] / WIDTH, pygame.mouse.get_pos()[1] / HEIGHT])
        control.update()

        # Drawing
        screen.fill(BLACK)
        control.draw(screen)
        
        # Update the display
        pygame.display.flip()
        pygame.time.delay(TICK_MS)

    # Quit Pygame
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
