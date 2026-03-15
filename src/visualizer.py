import pygame
import sys

class Visualizer:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((800, 400))
        self.scale = 50 # 1 meter = 50 pixels
        self.origin = (50, 200) # Offset to see (0,0)

    def draw(self, pose, path_y):
        self.screen.fill((30, 30, 30)) # Dark background
        
        # 1. Draw Path (Straight line)
        py = self.origin[1] - (path_y * self.scale)
        pygame.draw.line(self.screen, (255, 255, 255), (0, py), (800, py), 2)
        
        # 2. Draw Robot
        rx = self.origin[0] + (pose[0] * self.scale)
        ry = self.origin[1] - (pose[1] * self.scale)
        
        # Draw robot body
        pygame.draw.circle(self.screen, (0, 255, 0), (int(rx), int(ry)), 10)
        
        # Draw heading direction line
        end_x = rx + 15 * np.cos(pose[2])
        end_y = ry - 15 * np.sin(pose[2])
        pygame.draw.line(self.screen, (255, 0, 0), (rx, ry), (end_x, end_y), 3)

        pygame.display.flip()

# Visualizer Loop
# viz = Visualizer()
# while True:
#    pose = vsi.receive('pose')
#    viz.draw(pose, 0.0)