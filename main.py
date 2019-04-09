import math
import time
#from threading import Thread

#from plot.plot import DynamicPlot

from controllers.go_to_goal import GoToGoal, Goal
from controllers.follow_wall import FollowWall
from controllers.avoid_obstacles import AvoidObstacles

from helpers.body import State
from robots.mathbot import Mathbot, Specs

from controllers.params import Params
from supervisor.full_supervisor import Supervisor as FullSupervisor
from supervisor.supervisor import Supervisor as GTGSupervisor

from helpers.spi import SpiMaster


# Robot specification
wheel_radius = 0.02
base_length = 0.1
cpr = 930.0
min_rpm = 10
max_rpm = 130
v_max = 2 * math.pi / 60 * max_rpm * wheel_radius

# goal
x_goal = 0.5
y_goal = 0.5

# params
k_p = 10
k_i = 0.2
k_d = 2


def milliseconds():
    return int(round(time.time() * 1000))


last_time = milliseconds()
current_time = milliseconds()
dt = 0


def watch_time():
    global last_time, current_time, dt
    last_time = current_time
    current_time = milliseconds()
    diff = current_time - last_time
    if diff == 0:
        diff = 1
        time.sleep(0.001)
    dt = diff / 1000


mathbot = None
supervisor = None
spi = None

def simulation():
    global mathbot, supervisor
    mathbot = Mathbot(State(0, 0, 0), Specs(wheel_radius, base_length, cpr, v_max, min_rpm, max_rpm))

    go_to_goal_ctrl = GoToGoal(Params(k_p, k_i, k_d), Goal(x_goal, y_goal))

    ao_ctrl = AvoidObstacles(Params(k_p, k_i, k_d), mathbot)

    fw_ctrl = FollowWall(Params(k_p, k_i, k_d), mathbot)

    supervisor = FullSupervisor(mathbot, go_to_goal_ctrl, fw_ctrl, ao_ctrl)

    global last_time, current_time
    last_time = milliseconds()
    current_time = milliseconds()
    global change
    while not supervisor.to_stop:
        watch_time()
        supervisor.execute(dt)
        change = True
    global to_stop
    to_stop = True

def main():
    global mathbot, supervisor
    mathbot = Mathbot(State(0, 0, 0), Specs(wheel_radius, base_length, cpr, v_max, min_rpm, max_rpm), SpiMaster())

    go_to_goal_ctrl = GoToGoal(Params(k_p, k_i, k_d), Goal(x_goal, y_goal))

    ao_ctrl = AvoidObstacles(Params(k_p, k_i, k_d), mathbot)

    fw_ctrl = FollowWall(Params(k_p, k_i, k_d), mathbot)

##    supervisor = FullSupervisor(mathbot, go_to_goal_ctrl, fw_ctrl, ao_ctrl)
    supervisor = GTGSupervisor(mathbot, go_to_goal_ctrl)

    global last_time, current_time
    last_time = milliseconds()
    current_time = milliseconds()
    while not supervisor.to_stop:
        watch_time()
        supervisor.execute(dt)
        time.sleep(0.05)
    print("End")

if __name__ == '__main__':
    # sim = Thread(target=simulation)
    # sim.start()
    # if mathbot is None:
    #     while True:
    #         if mathbot is not None:
    #             break
    # if supervisor is None:
    #     while True:
    #         if supervisor is not None:
    #             break

    # d = DynamicPlot(mathbot, supervisor, "#FF8C00")
    # while not False:
    #     d.on_running()
    #     time.sleep(0.02)
    #     change = False
    # print("End")
    main()
