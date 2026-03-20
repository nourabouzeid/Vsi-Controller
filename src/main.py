from controller import PIDController
from simulator import Robot
from visualizer import Visualizer
from visualizer import Plotter
import numpy as np
import time

path = [
    ((-7.0, 0), 7.0, (0, np.pi)),
    ((7.0, 0), 7.0, (np.pi, 0)),

]
# path = (1, 0)

isCurved = True
# isCurved = False
robot = Robot(x = -14, y = 0, path = path, isCurved = isCurved)
controller = PIDController()
visualizer = Visualizer(path = path, isCurved = isCurved)
plotter = Plotter(path = path, isCurved = isCurved)



prev_time = time.perf_counter()
total_time = 0
v = 0
w = 0
for t in range(800):
    print(robot.pose)
    if(robot.pose[0] >= 14):
        break
    current_time = time.perf_counter()
    dt = current_time - prev_time
    total_time+=dt
    prev_time = current_time

    error = robot.update(v, w, dt)
    v, w = controller.update(error, dt)

    visualizer.update(robot.pose)
    plotter.update(robot.pose, error, dt)


print(f"Execution time: {total_time:.6f} seconds")
plotter.plot()