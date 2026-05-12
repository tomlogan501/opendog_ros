from cmd_manager.cmd_manager_node import CmdManager_ROS 
from cmd_manager.opendog_variables import Body, Leg, Cmds
from body_motion_planner.body_motion_planner import BodyMotionPlanner
from gait_generator.gait_planner import GaitPlanner
from multiprocessing import Process
import threading
import time
import rclpy

leg = Leg()
body = Body()
cmd = Cmds()


# lilnk  objects
# leg.foot_zero_pnt[:,2] = body.height  = cmd.body.height
cmd.gait.cycle_time = 0.8
cmd.gait.swing_time = 0.2
cmd.leg.foot_zero_pnt[0,0] = -10
cmd.leg.foot_zero_pnt[1,0] = -10
cmd.leg.foot_zero_pnt[2,0] = -70
cmd.leg.foot_zero_pnt[3,0] = -70
cmd.gait.stance_step_h = 0


cmd_manager = CmdManager_ROS(set_msgs=cmd, send_msgs=[leg, body])
gait_planner = GaitPlanner(cmd, leg, body)
bmp = BodyMotionPlanner(cmd, leg, body, gait_planner)

gait_planner.len_zmp_wavegait = 50


def main(args=None):

    thread_cmd_manager = threading.Thread(target=cmd_manager.start)
    thread_gait_planner = threading.Thread(target=gait_planner.run)
    thread_bmp = threading.Thread(target=bmp.run)

    bmp.set_init_pose()

    thread_cmd_manager.start()
    thread_bmp.start()
    thread_gait_planner.start()

    while rclpy.ok():
        try:
            time.sleep(1)
        except  KeyboardInterrupt:
            print("Fin boucle principale")

    gait_planner.running = False
    thread_gait_planner.join

    bmp.running = False
    thread_bmp.join

if __name__ == '__main__':
    main()
    