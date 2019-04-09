import math

from robots.mathbot import State, Specs, Mathbot
from controllers import go_to_goal
from helpers.rectangle import Rectangle
from helpers.quadtree import QuadTree

from math import log1p

from helpers.body import Body

#simulator_initialized = False

class Supervisor:
    __slots__ = 'robot', 'go_to_goal_ctrl', 'to_stop', 'estimated_state', 'left_ticks', 'right_ticks', \
                'distance_from_goal', 'obstacles', 'quad_tree'

    def __init__(self, robot_ref, go_to_goal_ctrl):
        self.robot= robot_ref
        self.go_to_goal_ctrl = go_to_goal_ctrl
        self.to_stop = False
        self.estimated_state = State(robot_ref.state.x, robot_ref.state.y, robot_ref.state.phi)
        self.left_ticks = 0
        self.right_ticks = 0
        self.distance_from_goal = 0
##        self.obstacles = []
##        self.quad_tree: QuadTree = None

##        obstacles_array = [
##            (State(0.5, 0.5, 0), [(-0.2, -0.2), (0.2, -0.2), (0.2, 0.2), (-0.2, 0.2)])
##        ]

##        for obstacle in obstacles_array:
##            self.obstacles.append(Body(obstacle[0], obstacle[1]))


    def calculate_distance_from_goal(self):
        x_goal = self.go_to_goal_ctrl.goal.x
        y_goal = self.go_to_goal_ctrl.goal.y
        self.distance_from_goal = math.sqrt(math.pow(self.estimated_state.x - x_goal, 2) + math.pow(self.estimated_state.y - y_goal, 2))

    def estimate_position(self):

        dt_left = self.robot.left_wheel.nr_of_ticks - self.left_ticks
        dt_right = self.robot.right_wheel.nr_of_ticks - self.right_ticks
        self.left_ticks += dt_left
        self.right_ticks += dt_right

        x = self.estimated_state.x
        y = self.estimated_state.y
        phi = self.estimated_state.phi

        meters_per_tick = 2*math.pi*self.robot.specs.wheel_radius / self.robot.specs.cpr

        distance_left = dt_left*meters_per_tick
        distance_right = dt_right*meters_per_tick
        distance_center = (distance_left + distance_right) / 2

        phi_dt = (distance_right - distance_left) / self.robot.specs.base_length

        self.estimated_state.x = x + distance_center*math.cos(phi)
        self.estimated_state.y = y + distance_center * math.sin(phi)
        phi_new = phi + phi_dt
        self.estimated_state.phi = math.atan2(math.sin(phi_new), math.cos(phi_new))

    def at_goal(self):
##        return self.distance_from_goal < (self.robot.specs.base_length / 2)
        return self.distance_from_goal < self.robot.specs.base_length

    def ensure_angular_speed(self, transl_speed, angular_speed):
        radius = self.robot.specs.wheel_radius
        base_length = self.robot.specs.base_length

        angular_wheel_min = self.robot.specs.angular_wheel_min
        angular_wheel_max = self.robot.specs.angular_wheel_max

        if abs(transl_speed) > 0:
            v_limit = max(min(abs(transl_speed), (radius / 2)*(2*angular_wheel_max)), (radius / 2)*(2*angular_wheel_min))
            w_limit = max(min(abs(angular_speed), (radius / base_length)*(angular_wheel_max - angular_wheel_min)), 0)

            desired_left, desired_right = self.robot.uni_to_diff(v_limit, w_limit)

            left_right_max = max(desired_left, desired_right)
            left_right_min = min(desired_left, desired_right)

            if left_right_max > angular_wheel_max:
                shift_ang_left = desired_left - (left_right_max - angular_wheel_max)
                shift_ang_right = desired_right - (left_right_max - angular_wheel_max)
            elif left_right_min < angular_wheel_min:
                shift_ang_left = desired_left + (angular_wheel_min - left_right_min)
                shift_ang_right = desired_right + (angular_wheel_min - left_right_min)
            else:
                shift_ang_left = desired_left
                shift_ang_right = desired_right

            shift_v, shift_w = self.robot.diff_to_uni(shift_ang_left, shift_ang_right)
            transl_speed = math.copysign(1, transl_speed)*shift_v
            angular_speed = math.copysign(1, angular_speed)*shift_w
        else:
            w_min = radius / base_length * (2 * angular_wheel_min)
            w_max = radius / base_length * (2 * angular_wheel_max)
            if abs(angular_speed) > w_min:
                angular_speed = math.copysign(1, angular_speed)*max(min(abs(angular_speed), w_max), w_min)
            else:
                angular_speed = 0

        return transl_speed, angular_speed

    def get_ir_distances(self):
        ir_distances = [max(min((log1p(3960) - log1p(sensor.read_distance()))/30 + sensor.specs.range_min,
                        sensor.specs.range_max), sensor.specs.range_min) for sensor in self.robot.sensors]
        return ir_distances

    def check_for_collisions(self):
        if self.quad_tree is None:
            self.quad_tree = QuadTree(self.obstacles)

        for sensor in self.robot.sensors:
            sensor.get_envelope_i(True)
            rect = Rectangle(sensor.get_bounding_rectangle())
            sensor.update_distance()
            for obstacle in self.quad_tree.find_items(rect):
                sensor.update_distance(obstacle)

        rect = Rectangle(self.robot.get_bounding_rectangle())
        for obstacle in self.quad_tree.find_items(rect):
            if self.robot.has_collision(obstacle):
                print("Collision")
                self.to_stop = True

    def execute(self, dt):
        self.estimate_position()
        self.calculate_distance_from_goal()
        
        self.robot.update_ticks()

        transl_speed, angular_speed = self.go_to_goal_ctrl.execute(self.estimated_state, self.robot, [], dt)
        transl_speed_ens, angular_speed_ens = self.ensure_angular_speed(transl_speed, angular_speed)
        angular_left, angular_right = self.robot.uni_to_diff(transl_speed_ens, angular_speed_ens)

        self.robot.move(angular_left, angular_right, dt)
        print(self.estimated_state.x, self.estimated_state.y, self.estimated_state.phi)
        #self.robot.update_body()

        #self.check_for_collisions()

        if self.at_goal():
            self.robot.move(0.0, 0.0, 1)
            self.to_stop = True

