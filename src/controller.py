import numpy as np

class PIDController:
    def __init__(self, kp_theta, kp_y, ki, kd, v_max):
        self.kp_theta = kp_theta
        self.kp_y = kp_y
        self.ki = ki
        self.kd = kd
        self.v_max = v_max
        
        self.prev_heading_error = 0
        self.integral_heading_error = 0
        self.dt = 0.1 

    def compute(self, pose, path):
        rx, ry, r_theta = pose
        m, c = path

        # 1. Calculate Lateral Error 
        # Formula for distance from point (rx, ry) to line mx - y + c = 0
        lateral_error = (m * rx - ry + c) / np.sqrt(m**2 + 1)

        # 2. Calculate Heading Error
        path_theta = np.arctan(m)
        
        # We want theta_error to be normalized between -pi and pi
        heading_error = path_theta - r_theta
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

        # 3. PID Math
        self.integral_heading_error += heading_error * self.dt
        derivative_heading_error = (heading_error - self.prev_heading_error) / self.dt
        
        w = (self.kp_theta * heading_error) + (self.kp_y * lateral_error) + (self.ki * self.integral_heading_error) + (self.kd * derivative_heading_error)
        v = self.v_max * max(0,np.cos(heading_error))

        self.prev_heading_error = heading_error

        return np.array([v, w])