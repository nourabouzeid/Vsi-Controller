import pygame
import numpy as np


class Visualizer:
    def __init__(self, width=800, height=600, scale=50):
        pygame.init()

        self.width = width
        self.height = height
        self.scale = scale

        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Line Following Robot")

        self.clock = pygame.time.Clock()

    def world_to_screen(self, x, y):
        sx = int(self.width / 2 + x * self.scale)
        sy = int(self.height / 2 - y * self.scale)
        return sx, sy

    def draw_line(self, m, c):
        x1 = -self.width / (2 * self.scale)
        x2 = self.width / (2 * self.scale)

        y1 = m * x1 + c
        y2 = m * x2 + c

        p1 = self.world_to_screen(x1, y1)
        p2 = self.world_to_screen(x2, y2)

        pygame.draw.line(self.screen, (0, 200, 0), p1, p2, 3)

    def draw_robot(self, pose):
        x, y, theta = pose
        px, py = self.world_to_screen(x, y)

        radius = 8
        pygame.draw.circle(self.screen, (0, 100, 255), (px, py), radius)

        # heading arrow
        arrow_length = 20
        hx = px + arrow_length * np.cos(theta)
        hy = py - arrow_length * np.sin(theta)

        pygame.draw.line(self.screen, (255, 0, 0), (px, py), (hx, hy), 3)

    def update(self, pose, path):
        m, c = path

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        self.screen.fill((30, 30, 30))

        self.draw_line(m, c)
        self.draw_robot(pose)

        pygame.display.flip()
        self.clock.tick(60)