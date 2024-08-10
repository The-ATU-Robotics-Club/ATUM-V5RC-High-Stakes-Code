import robot
import sigma
import numpy as np
import pygame

pygame.init()

TEN_MS_PER_FRAME = 100
TIME_SCALE = 1
SCREEN_SIZE = 900
black = 0x000000
white = 0xFFFFFF
green = 0x0A5640
gold = 0xFFCF00
red = 0xFF0000
purple = 0xFF00FF
fieldImage = pygame.image.load("proto/field.png")
clock = pygame.time.Clock()
joystick = pygame.joystick.Joystick(0)
joystick.init()
screen = pygame.display.set_mode([SCREEN_SIZE, SCREEN_SIZE])


def mToPix(value):
    return value * (SCREEN_SIZE / 12) / 0.3048


def center(x, y):
    x = mToPix(x) + SCREEN_SIZE / 2
    y = SCREEN_SIZE / 2 - mToPix(y)
    return (x, y)


class RobotView:
    def __init__(self, robotModel, dirLengthMult, thickness):
        self.robotModel = robotModel
        self.diameter = mToPix(robotModel.rb)
        self.dirLength = dirLengthMult * self.diameter
        self.thickness = thickness

    def draw(self):
        x, y, h, _, _ = self.robotModel.state
        x, y = center(x, y)
        h = np.pi / 2 - h
        pygame.draw.circle(screen, gold, (x, y), self.diameter, self.thickness)
        xEnd = x + self.dirLength * np.cos(h)
        yEnd = y - self.dirLength * np.sin(h)
        pygame.draw.line(screen, green, (x, y), (xEnd, yEnd), self.thickness)
        pygame.draw.circle(screen, white, (x, y), self.thickness)


robotModel = robot.RobotModel(
    robot.State(0, 0, 0, 0, 0), 3, 0.19, 0.0508, 0.08, 1.667, 6.8, 0.01
)

robotView = RobotView(robotModel, 1.5, 5)


def tank():
    global joystick
    left = -12 * joystick.get_axis(1)
    right = -12 * joystick.get_axis(3)
    return robot.Input(left, right)


def runSim(loop):
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        screen.fill(black)
        screen.blit(fieldImage, (0, 0))
        loop()
        pygame.display.flip()
        clock.tick(TEN_MS_PER_FRAME * TIME_SCALE)
    pygame.quit()
