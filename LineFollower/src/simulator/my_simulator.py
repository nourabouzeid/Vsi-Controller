import numpy as np

class Robot:
    def __init__(self, x=0.0, y=0.0, theta=0.0,
                 linear_noise_std=0, angular_noise_std=0,
                 pose_noise_std=0, path = None, isCurved = False):
        
        self.pose = np.array([x, y, theta])
        self.linear_noise_std = linear_noise_std
        self.angular_noise_std = angular_noise_std
        self.pose_noise_std = pose_noise_std  
        self.path = path
        self.isCurved = isCurved


    def normalize(self, a):
        a %= 2*np.pi
        if a > np.pi:
            a -= 2*np.pi
        return a

    def line_error(self):
        m, c = self.path

        lateral_error = (m * self.pose[0] - self.pose[1] + c) / np.sqrt(m**2 + 1)

        path_theta = np.arctan(m)
        heading_error = self.normalize(path_theta - self.pose[2])

        return heading_error, lateral_error

    def curved_error(self):
        x_r, y_r, theta_r = self.pose
        min_dist_to_path = float('inf')
        best_lateral = 0.0
        best_heading = 0.0

        for center, radius, angle_range in self.path:
            cx, cy = center
            start_angle, end_angle = angle_range

            span = end_angle - start_angle
            direction = -1 if span >= 0 else 1  
            arc_len = abs(span)

            dx, dy = x_r - cx, y_r - cy
            dist_to_center = np.sqrt(dx**2 + dy**2)
            angle_to_robot = np.arctan2(dy, dx)

            rel_angle = (self.normalize(angle_to_robot - start_angle)) 

            is_within_arc = (rel_angle <= arc_len)

            if is_within_arc:
                target_angle = angle_to_robot
            else:
                dist_s = np.sqrt((x_r - (cx + radius * np.cos(start_angle)))**2 +
                                 (y_r - (cy + radius * np.sin(start_angle)))**2)
                dist_e = np.sqrt((x_r - (cx + radius * np.cos(end_angle)))**2 +
                                 (y_r - (cy + radius * np.sin(end_angle)))**2)
                target_angle = start_angle if dist_s < dist_e else end_angle

            closest_x = cx + radius * np.cos(target_angle)
            closest_y = cy + radius * np.sin(target_angle)
            dist_to_arc = np.sqrt((x_r - closest_x)**2 + (y_r - closest_y)**2)

            if dist_to_arc < min_dist_to_path:
                min_dist_to_path = dist_to_arc

                best_lateral = direction * (dist_to_center - radius)

                path_theta = target_angle + direction * (np.pi / 2)
                best_heading = self.normalize(path_theta - theta_r)

        return best_heading, best_lateral


    def update(self, v, w, dt):
        noisy_v = v + np.random.normal(0, self.linear_noise_std)
        noisy_w = w + np.random.normal(0, self.angular_noise_std)

        x, y, theta = self.pose

        new_x = x + noisy_v * np.cos(theta) * dt
        new_y = y + noisy_v * np.sin(theta) * dt
        new_theta = theta + noisy_w * dt

        new_x += np.random.normal(0, self.pose_noise_std)
        new_y += np.random.normal(0, self.pose_noise_std)
        new_theta += np.random.normal(0, self.pose_noise_std / 2)

        self.pose = np.array([new_x, new_y, new_theta])

        if self.isCurved:
            return self.curved_error()
        else :
            return self.line_error()