from kawapai.robot import KawaBot
from DracaenaPLC import DracaenaPLC, DracaenaCodes
import time


class DracaenaRobot(KawaBot):
    def __init__(self, plc: DracaenaPLC):
        super().__init__(host="192.168.0.1", port=23)

        self.plc = plc

        self.abort_kill_all()
        self.motor_power_off()
        self.reset_error()

        self.motor_power_on()
        self.initiate_kawabot()
        self.connect_to_movement_server()
        self.connect_to_pose_update_server()

        self.move_to_home()
        
    def move_to_home(self):
        self.jmove_joint_check_pos(0, -34, -55, 0, -72, 90)
        
    def move_to_drop_point(self):
        self.jmove_joint_check_pos(-90, -15, -55, 0, -72, 90)
        
    def remove_leaf(self, x, y, z, tool_angle, centre_x, centre_y):
        # Round values as the robot can only receive integers
        x_rounded = round(x)
        y_rounded = round(y)
        z_rounded = round(z) if z > 200 else 200

        # Calculate the coordinates towards the centre during the pulling motion
        x_pull = round(x + (centre_x - x) / 4)
        y_pull = round(y + (centre_y - y) / 4)
        z_pull = z_rounded - 30 if z_rounded - 30 > 180 else 180
        z_above = 410
        z_down = 110

        self.jmove_cartesian_check_pos(x_rounded, y_rounded, z_above, tool_angle, 90, 0)
        self.lmove_cartesian_check_pos(x_rounded, y_rounded, z_rounded, tool_angle, 90, 0)
        self.jmove_cartesian_check_pos(x_pull, y_pull, z_pull, tool_angle, 90, 0)
        self.plc.send(DracaenaCodes.GripperClose.value)
        time.sleep(1)
        self.jmove_cartesian_check_pos(x_pull, y_pull, z_down, tool_angle, 90, 0)
        self.move_to_home()
        self.move_to_drop_point()
        self.plc.send(DracaenaCodes.GripperOpen.value)
        time.sleep(1)
        self.move_to_home()

    def exit(self):
        self.close_movement_server()
        self.close_pose_update_server()
        self.motor_power_off()
        self.disconnect()
