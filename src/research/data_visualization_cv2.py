from rplidar import RPLidar
from math import sin, cos, pi, floor
import cv2
import pygame
from time import time
from collections import OrderedDict

# Set up pygame and the display
pygame.init()
lcd = pygame.display.set_mode((640, 640))
pygame.mouse.set_visible(False)
lcd.fill((0, 0, 0))
pygame.display.update()

max_distance = 0


def process_data(data):
    global max_distance
    lcd.fill((0, 0, 0))
    for key, value in data.items():
        if value > 0:                  # ignore initially ungathered data points
            max_distance = max([min([5000, value]), max_distance])
            radians = key * pi / 180.0
            x = distance * cos(radians)
            y = distance * sin(radians)
            # print(angle, distance, x, y)
            point = (360 + int(x / max_distance * 119), 120 + int(y / max_distance * 119))
            lcd.set_at(point, pygame.Color(255, 255, 255))
    pygame.display.update()


PORT_NAME = '/dev/ttyUSB0'
TIME_LIMIT = time() + 10

x_coordinates = []
y_coordinates = []

lidar = RPLidar(PORT_NAME)
lidar.connect()
scan_data = OrderedDict()
try:
    # print(lidar.info)
    for scan in lidar.iter_measures():
        angle = scan[2]
        # distance = round(scan[3] / 10, 2)
        distance = scan[3] / 10
        if min([359, floor(angle)]) < 46 or min([359, floor(angle)]) > 314:
            scan_data[min([359, floor(angle)])] = distance
        if floor(angle) == 359:
            # print(scan_data)
            process_data(scan_data)
            scan_data = OrderedDict()


except KeyboardInterrupt:
    print('Stopping.')
lidar.stop()
lidar.disconnect()
