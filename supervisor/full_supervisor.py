import math

from robots.mathbot import State, Mathbot
from controllers.go_to_goal import GoToGoal
from controllers.follow_wall import FollowWall
from controllers.avoid_obstacles import AvoidObstacles
from controllers.hold import Hold
from helpers.rectangle import Rectangle
from helpers.quadtree import QuadTree

from math import log1p

from helpers.body import Body


class Supervisor:

    __slots__ = 'robot', 'gtg_ctrl', 'to_stop', 'estimated_state', 'left_ticks', 'right_ticks', \
                'distance_from_goal', 'obstacles', 'quad_tree', 'ao_ctrl', 'sensor_distances', 'min_dist', \
                'fw_ctrl', 'hold_ctrl', 'current_controller', 'controller_states'

    def __init__(self, robot_ref, go_to_goal_ctrl, follow_wall_ctrl, avoid_obstacles_ctrl):

        self.robot = robot_ref
        self.gtg_ctrl = go_to_goal_ctrl
        self.fw_ctrl = follow_wall_ctrl
        self.ao_ctrl = avoid_obstacles_ctrl
        self.hold_ctrl = Hold()
        self.controller_states = {
            self.gtg_ctrl: [(self.at_goal, self.hold_ctrl), (self.at_wall, self.fw_ctrl)],
            self.fw_ctrl: [(self.at_goal, self.hold_ctrl), (self.at_obstacle, self.ao_ctrl),
                           (self.clear, self.gtg_ctrl)],
            self.ao_ctrl: [(self.at_goal, self.hold_ctrl), (self.free, self.fw_ctrl)],
            self.hold_ctrl: []
        }
        self.current_controller = self.gtg_ctrl

        self.to_stop = False
        self.estimated_state = State(robot_ref.state.x, robot_ref.state.y, robot_ref.state.phi)
        self.robot.update_ticks()
        self.left_ticks = self.robot.left_wheel.nr_of_ticks
        self.right_ticks = self.robot.right_wheel.nr_of_ticks
        self.distance_from_goal = 0
        # self.obstacles = []
        # self.quad_tree: QuadTree = None

        # obstacles_array = [

        #     (State(0.5, 0.5, -math.pi / 7), [(-0.2, -0.2), (0.6, -0.2), (0.6, 0.7), (-0.2, 0.7)]),
        #     (State(0, 0, 0), [(0.3, -0.2), (0.5, -0.2), (0.5, 0.7), (0.3, 0.7)]),
        #     (State(0, 0, 0), [(0.6, -0.2), (0.7, -0.2), (0.3, 0.7), (0.2, 0.7)])
        # ]

        # for obstacle in obstacles_array:
        #     self.obstacles.append(Body(obstacle[0], obstacle[1]))
        self.sensor_distances = []
        self.min_dist = 0

    def calculate_distance_from_goal(self):
        x_goal = self.gtg_ctrl.goal.x
        y_goal = self.gtg_ctrl.goal.y
        self.distance_from_goal = math.sqrt(math.pow(self.estimated_state.x - x_goal, 2)
                                            + math.pow(self.estimated_state.y - y_goal, 2))

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

    def at_obstacle(self):
        return self.min_dist < self.robot.sensors[0].specs.range_max / 3

    def free(self):
        return self.min_dist > self.robot.sensors[0].specs.range_max / 1.5

    def at_wall(self):
        return self.min_dist < self.robot.sensors[0].specs.range_max / 2

    def set_direction(self):
        direction = 1
        if self.robot.sensors[self.sensor_distances.index(self.min_dist)].state_r.phi > 0:
            direction *= -1

        self.fw_ctrl.direction = direction

    def clear(self):
        go_to_goal_angle = self.gtg_ctrl.get_heading_angle(self.estimated_state)
        ao_angle = self.ao_ctrl.get_heading_angle(self.estimated_state, self.sensor_distances)
        # x.y = |x|*|y|*cos(x,y)
        return self.distance_from_goal < self.fw_ctrl.initial_distance and math.cos(go_to_goal_angle - ao_angle) > 0

    def ensure_angular_speed(self, transl_speed, angular_speed):
        radius = self.robot.specs.wheel_radius
        base_length = self.robot.specs.base_length

        angular_wheel_min = self.robot.specs.angular_wheel_min
        angular_wheel_max = self.robot.specs.angular_wheel_max

        if abs(transl_speed) > 0:
            v_limit = max(min(abs(transl_speed), (radius / 2)*(2*angular_wheel_max)),
                          (radius / 2)*(2*angular_wheel_min))
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
        # return [sensor.distance for sensor in self.robot.sensors]
        return self.robot.spi.get_sensors()

    # def check_for_collisions(self):
    #     if self.quad_tree is None:
    #         self.quad_tree = QuadTree(self.obstacles)

    #     for sensor in self.robot.sensors:
    #         rect = Rectangle(sensor.get_bounding_rectangle())
    #         sensor.update_distance()
    #         for obstacle in self.quad_tree.find_items(rect):
    #             sensor.update_distance(obstacle)

    #     rect = Rectangle(self.robot.get_bounding_rectangle())
    #     for obstacle in self.quad_tree.find_items(rect):
    #         if self.robot.has_collision(obstacle):
    #             print("Collision")
    #             self.current_controller = self.hold_ctrl
    #             self.to_stop = True
    #             return True

    #     return False

    def execute(self, dt):
        self.estimate_position()
        self.calculate_distance_from_goal()

        # self.robot.update_body()

        # self.check_for_collisions()

        self.sensor_distances = self.get_ir_distances()
        self.robot.update_ticks()
        self.min_dist = min(self.sensor_distances)

        if self.current_controller in self.controller_states:
            for f, c in self.controller_states[self.current_controller]:
                if f():
                    if c == self.fw_ctrl:
                        self.set_direction()
                        c.restart(self.distance_from_goal)
                    else:
                        c.restart()
                    self.current_controller = c
                    print("Switched to {}".format(c.__class__.__name__))
                    break

        transl_speed, angular_speed = self.current_controller.execute(self.estimated_state,
                                                                      self.robot, self.sensor_distances, dt)
        transl_speed_ens, angular_speed_ens = self.ensure_angular_speed(transl_speed, angular_speed)
        angular_left, angular_right = self.robot.uni_to_diff(transl_speed_ens, angular_speed_ens)

        self.robot.move(angular_left, angular_right, dt)
        print(self.estimated_state.x, self.estimated_state.y, self.estimated_state.phi)
        #print(self.left_ticks, self.right_ticks)
        #print(self.sensor_distances)

        if self.at_goal():
            self.robot.move(0.0, 0.0, 1)
            self.to_stop = True
