import pygame
import numpy as np
import matplotlib.pyplot as plt


class Visualizer:
    def __init__(self, width=1920, height=1080, scale=50, path=[], isCurved=False):
        pygame.init()
        self.width = width
        self.height = height
        self.scale = scale
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Line Following Robot")
        self.clock = pygame.time.Clock()
        self.path = path
        self.isCurved = isCurved

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

    def draw_arc(self, center, radius, angle_range):
        cx, cy = center
        start_angle, end_angle = angle_range
        span = (end_angle - start_angle + 2 * np.pi) % (2 * np.pi)
        num_points = max(30, int(np.degrees(span)))
        angles = np.linspace(start_angle, start_angle + span, num_points)
        points = []
        for a in angles:
            wx = cx + radius * np.cos(a)
            wy = cy + radius * np.sin(a)
            points.append(self.world_to_screen(wx, wy))
        if len(points) >= 2:
            pygame.draw.lines(self.screen, (0, 200, 0), False, points, 3)

    def draw_curved_path(self, path):
        for center, radius, angle_range in path:
            self.draw_arc(center, radius, angle_range)

    def draw_robot(self, pose):
        x, y, theta = pose
        px, py = self.world_to_screen(x, y)
        radius = 8
        pygame.draw.circle(self.screen, (0, 100, 255), (px, py), radius)
        arrow_length = 20
        hx = px + arrow_length * np.cos(theta)
        hy = py - arrow_length * np.sin(theta)
        pygame.draw.line(self.screen, (255, 0, 0), (px, py), (hx, hy), 3)

    def update(self, pose):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
        self.screen.fill((30, 30, 30))
        if self.isCurved:
            self.draw_curved_path(self.path)
        else:
            m, c = self.path
            self.draw_line(m, c)
        self.draw_robot(pose)
        pygame.display.flip()
        self.clock.tick(60)


class Plotter:
    def __init__(self, path, isCurved, 
                 settling_threshold=0.1,  # lateral error band to be considered "settled"
                 ):  # number of steps at end to average for steady-state
        self.x_history = []
        self.y_history = []
        self.lateral_history = []
        self.heading_history = []
        self.time_history = []
        self.path = path
        self.isCurved = isCurved
        self.settling_threshold = settling_threshold
        self.ss_period = 2
        self.t = 0.0

    def update(self, pose, error, dt):
        x, y, _ = pose
        heading_error, lateral_error = error
        self.x_history.append(x)
        self.y_history.append(y)
        self.lateral_history.append(lateral_error)
        self.heading_history.append(heading_error)
        self.time_history.append(self.t)
        self.t += dt

    def compute_kpis(self):
        lateral = np.array(self.lateral_history)
        times   = np.array(self.time_history)

        steady_state_segments = []
        in_ss = False
        start_idx = None

        # --- 1. Extract steady-state segments ---
        for i in range(len(lateral)):
            
            if abs(lateral[i]) <= self.settling_threshold:
                if not in_ss:
                    in_ss = True
                    start_idx = i
            else:
                if in_ss:
                    duration = times[i-1] - times[start_idx]
                    
                    if duration >= self.ss_period:
                        steady_state_segments.append((start_idx, i))
                    
                    in_ss = False

        # Handle end case
        if in_ss:
            duration = times[-1] - times[start_idx]
            if duration >= self.ss_period:
                steady_state_segments.append((start_idx, len(lateral)))

        # --- 2. Compute KPIs per segment ---
        kpis = []

        prev_end = 0  # start of signal for first segment

        for (start, end) in steady_state_segments:
            
            segment_errors = lateral[start:end]

            transient_start = times[prev_end]
            # Settling time = when this steady state starts
            settling_time = times[start] - transient_start
            

            # Steady-state error = mean inside segment
            steady_state_error = np.mean(np.abs(segment_errors))

            # Overshoot = max deviation BEFORE entering steady state
            if start > prev_end:
                segment = lateral[prev_end:start]

                if len(segment) > 0:
                    idx = np.argmax(segment)
                    overshoot = segment[idx]
                    overshoot_time = times[prev_end + idx]
                else:
                    overshoot = 0.0
                    overshoot_time = times[start]
            else:
                overshoot = 0.0

            kpis.append({
                "settling_time": settling_time,
                "steady_state_error": steady_state_error,
                "overshoot": overshoot,
                "overshoot_time": overshoot_time,
                "transient_start": transient_start
            })

            prev_end = end  # next segment starts after this

        return kpis
    
    def print_kpis(self):
        kpis_list = self.compute_kpis()

        print("=" * 45)
        print("              KPI Summary")
        print("=" * 45)

        if not kpis_list:
            print("No valid steady-state segments detected.")
            print("=" * 45)
            return

        for idx, kpis in enumerate(kpis_list, 1):
            print(f"\nSegment {idx}:")
            print(f"  Overshoot:          {kpis['overshoot']:+.4f} m")
            print(f"  Settling Time:      {kpis['settling_time']:.2f} s")
            print(f"  Steady-State Error: {kpis['steady_state_error']:.4f} m")

        print("\n" + "=" * 45)

    def _sample_arc(self, center, radius, angle_range, num_points=100):
        cx, cy = center
        start_angle, end_angle = angle_range
        span = (end_angle - start_angle + 2 * np.pi) % (2 * np.pi)
        angles = np.linspace(start_angle, start_angle + span, num_points)
        xs = cx + radius * np.cos(angles)
        ys = cy + radius * np.sin(angles)
        return xs, ys

    def plot(self):
        kpis_list = self.compute_kpis()
        times = np.array(self.time_history)
        lateral = np.array(self.lateral_history)

        fig, axes = plt.subplots(2, 2, figsize=(14, 8))
        fig.suptitle("Robot Path Following — KPI Dashboard", fontsize=14)

        # --- Plot 1: Trajectory vs Path ---
        ax = axes[0, 0]
        if self.isCurved:
            for i, (center, radius, angle_range) in enumerate(self.path):
                xs, ys = self._sample_arc(center, radius, angle_range)
                ax.plot(xs, ys, color='tab:blue', label="Desired Path" if i == 0 else None)

                cx, cy = center
                start_angle, end_angle = angle_range

                ax.plot(cx + radius * np.cos(start_angle),
                        cy + radius * np.sin(start_angle), 'b>', markersize=6)
                ax.plot(cx + radius * np.cos(end_angle),
                        cy + radius * np.sin(end_angle), 'bs', markersize=6)
        else:
            m, c = self.path
            x_vals = np.linspace(min(self.x_history) - 1, max(self.x_history) + 1, 100)
            ax.plot(x_vals, m * x_vals + c, label="Desired Path", color='tab:blue')

        ax.plot(self.x_history, self.y_history, color='tab:orange', label="Robot Trajectory")
        ax.set_title("Trajectory vs Path")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.legend()
        ax.grid()
        ax.axis("equal")

        # --- Plot 2: Lateral Error over Time ---
        ax = axes[0, 1]
        ax.plot(times, lateral, color='tab:red', label="Lateral Error")

        ax.axhline(self.settling_threshold,  color='gray', linestyle='--', linewidth=1)
        ax.axhline(-self.settling_threshold, color='gray', linestyle='--', linewidth=1)

        for i, kpis in enumerate(kpis_list):
            st = kpis['settling_time']
            os = kpis['overshoot']
            ts = kpis['transient_start']
            # Settling time (vertical line)
            ax.axvline(ts + st, linestyle='--', color='green',
                    label="Settling Time" if i == 0 else None)

            # Overshoot as a point (better than full line)
            ot = kpis['overshoot_time']
            ax.scatter(ot, os, color='purple', zorder=3,
                    label="Overshoot" if i == 0 else None)

        ax.set_title("Lateral Error")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Error (m)")
        ax.legend()
        ax.grid()

        # --- Plot 3: Heading Error over Time ---
        ax = axes[1, 0]
        ax.plot(times, np.degrees(self.heading_history),
                color='tab:purple', label="Heading Error")
        ax.set_title("Heading Error")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Error (degrees)")
        ax.legend()
        ax.grid()

        # --- Plot 4: KPI Summary Text ---
        ax = axes[1, 1]
        ax.axis('off')

        if not kpis_list:
            summary = "No steady-state segments detected."
        else:
            lines = ["KPI Summary", "─" * 35]

            for i, k in enumerate(kpis_list, 1):
                lines.append(f"Segment {i}:")
                lines.append(f"  Overshoot:          {k['overshoot']:+.4f} m")
                lines.append(f"  Settling Time:      {k['settling_time']:.2f} s")
                lines.append(f"  Steady-State Error: {k['steady_state_error']:.4f} m")
                lines.append("")

            lines.append("─" * 35)
            lines.append(f"Settling Threshold:   ±{self.settling_threshold} m")

            summary = "\n".join(lines)

        ax.text(0.05, 0.5, summary,
                transform=ax.transAxes,
                fontsize=11,
                verticalalignment='center',
                fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout()
        plt.show()

        self.print_kpis()