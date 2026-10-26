import math

class Robot:

    def __init__(self, x, y,theta_orientation, wheel_radius, wheel_base):
        self.x_position = x
        self.y_position = y
        self.theta_orientation = theta_orientation
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base

    def update_pose(self,omega_left, omega_right, dt):
        x_dot = (self.wheel_radius/2 ) * (omega_left + omega_right) *  math.cos(self.theta_orientation)
        y_dot = (self.wheel_radius/2 ) * (omega_left + omega_right) *  math.sin(self.theta_orientation)
        theta_dot = (self.wheel_radius / self.wheel_base) * (omega_right -  omega_left)

        self.x_position += x_dot * dt
        self.y_position += y_dot * dt
        self.theta_orientation += theta_dot * dt

    def print_state(self):
        print(f"Pose is: ({self.x_position:.1f}, {self.y_position:.1f}, {(self.theta_orientation * 180 / math.pi):.1f} deg)")
        