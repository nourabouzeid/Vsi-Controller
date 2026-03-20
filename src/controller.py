import numpy as np

class PIDController:
    def __init__(self, kp_theta = 4, kp_y = 4, ki = 0, kd = 0 , vmax = 1.8):
        self.kp_theta = kp_theta
        self.kp_y = kp_y
        self.ki = ki
        self.kd = kd
        self.vmax = vmax
        
        self.prev_heading_error = 0
        self.integral_heading_error = 0

    def update(self, error, dt):
        heading_error, lateral_error = error


        # PID
        self.integral_heading_error += heading_error * dt
        derivative_heading_error = (heading_error - self.prev_heading_error) / dt
        
        w = (self.kp_theta * heading_error) + (self.kp_y * lateral_error) + (self.ki * self.integral_heading_error) + (self.kd * derivative_heading_error)

        # Make turns smoother
        # v = self.vmax * max(0.5, np.cos(heading_error))

        self.prev_heading_error = heading_error

        return self.vmax, w