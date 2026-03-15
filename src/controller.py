import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.prev_cte = 0
        self.integral_cte = 0
        self.dt = 0.1 

    def compute(self, pose, path):
        rx, ry, r_theta = pose
        m, c = path

        # 1. Calculate Lateral Error (Cross-Track Error)
        # Formula for distance from point (rx, ry) to line mx - y + c = 0
        cte = (m * rx - ry + c) / np.sqrt(m**2 + 1)

        # 2. Calculate Heading Error
        path_theta = np.arctan(m)
        
        # We want theta_error to be normalized between -pi and pi
        heading_error = path_theta - r_theta
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

        # 3. PID Math
        self.integral_cte += cte * self.dt
        derivative_cte = (cte - self.prev_cte) / self.dt
        
        # Steering command (w)
        # We combine CTE correction and Heading alignment
        w = (self.kp * cte) + (self.ki * self.integral_cte) + (self.kd * derivative_cte)
        
        # Add a proportional gain for heading to help the robot face the right way
        # Usually, a separate Kp_theta is used
        w += 1.0 * heading_error 

        self.prev_cte = cte
        
        # 4. Output [v, w]
        v = 1.0 # Constant forward speed
        return np.array([v, w])