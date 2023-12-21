import pygame
import sys
import numpy as np
import math

WIDTH, HEIGHT = 800, 800

# Define colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)

# Define simulation constants
TICK_MS: int = 30
MAX_ACCELERATION: float = 3e-2 / TICK_MS

# Max velocity to reach target
# (v^2 / 2a) = x
# 2ax = v^2
# sqrt(2ax) = v

def get_target_velocity(target: np.ndarray, position: np.ndarray) -> np.ndarray:
    to_target = target - position
    target_distance = np.linalg.norm(to_target)
    unit_to_target = to_target / target_distance

    target_velocity_scalar = math.sqrt(2 * MAX_ACCELERATION * target_distance)
    return target_velocity_scalar * unit_to_target

def to_pygame_point(arr: np.ndarray) -> tuple[float, float]:
    return (arr[0] * WIDTH, arr[1] * HEIGHT)

class Control:
    target: np.ndarray
    position: np.ndarray
    velocity: np.ndarray

    def __init__(self, position: np.ndarray, target: np.ndarray, velocity: np.ndarray):
        self.position = position
        self.target = target
        self.velocity = velocity

    def draw(self, screen):
        if not np.isclose(np.linalg.norm(self.velocity), 0):
            unit_to_target = self.target - self.position
            unit_to_target /= np.linalg.norm(unit_to_target)
            unit_velocity = self.velocity / np.linalg.norm(self.velocity)

            # Draw a triangle to indicate heading of the control
            velocity_perpendicular = np.array([unit_velocity[1], -unit_velocity[0]])
            arrow_points = [
                self.position + unit_velocity * 0.02,
                self.position + velocity_perpendicular * 0.01,
                self.position - velocity_perpendicular * 0.01,
            ]
            pygame.draw.polygon(screen, WHITE, list(map(to_pygame_point, arrow_points)))
        else:
            pygame.draw.circle(screen, WHITE, to_pygame_point(self.position), radius=5)

        # Draw target
        pygame.draw.circle(screen, RED, to_pygame_point(self.target), radius=2)
        
        

    def update(self):
        target_velocity = get_target_velocity(self.target, self.position)
        if np.linalg.norm(target_velocity) < 0.001:
            target_velocity = np.zeros(2)

        acceleration = target_velocity - self.velocity
        acceleration_scalar = np.linalg.norm(acceleration)

        if acceleration_scalar > MAX_ACCELERATION:
            acceleration /= acceleration_scalar
            acceleration *= MAX_ACCELERATION

        self.velocity += acceleration
        self.position += self.velocity

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
