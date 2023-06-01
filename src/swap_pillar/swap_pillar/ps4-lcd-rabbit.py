import rclpy
import cv2
import math
import numpy as np
import datetime
from rclpy.node import Node
from sensor_msgs.msg import Joy, Image
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from rclpy import qos


class Ps4(Node):
    def __init__(self):
        super().__init__("xbox_control_node")
        self.dat = self.create_subscription(
            Joy, "joy", self.sub_callback, qos_profile=qos.qos_profile_sensor_data
        )
        self.dat

        self.cam = self.create_subscription(
            Float64,
            "distance_xy",
            self.sub_callback_cam,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.cam

        self.joyCon = self.create_subscription(
            String,
            "joy_connection_status",
            self.sub_callback_joyCon,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.joyCon

        self.sent_drive = self.create_publisher(
            Twist, "control_drive_topic", qos_profile=qos.qos_profile_system_default
        )
        self.sent_drive_timer = self.create_timer(0.05, self.sent_drive_callback)

        self.all = [
            "X",
            "O",
            "Dummy2",
            "S",
            "T",
            "Dummy5",
            "L1",
            "R1",
            "Dummy8",
            "Dummy9",
            "L",
            "R",
            "Dummy12",
            "Dummy13",
            "Dummy14",
            "PS",
        ]  # ? XBOX
        self.all2 = ["LX", "LY", "RX", "RY", "LT", "RT", "AX", "AY"]  # ? XBOX
        # self.all = [
        #     "X",
        #     "O",
        #     "T",
        #     "S",
        #     "L1",
        #     "R1",
        #     "L2",
        #     "R2",
        #     "L",
        #     "R",
        #     "PS",
        # ]  # ? PS4
        # self.all2 = ["LX", "LY", "LT", "RX", "RY", "RT", "AX", "AY"]  # ? PS4
        self.button = {element: 0 for element in self.all}
        self.axes = {element: 0 for element in self.all2}

        self.button["L2"] = 0
        self.button["R2"] = 0

        self.pwm = 0
        self.state = 0
        self.counter = 0
        self.state2 = 0
        self.counter2 = 0

        self.distance = 0.0
        self.state_auto = 0

        self.assis_shoot = 0
        self.list_debount_r = []
        self.list_debount_l = []

        self.param_pwm_motor0 = 163.0  # เสาฝั่งตรงข้าม
        self.param_pwm_motor1 = 205.0  # เสากลาง
        self.param_pwm_motor2 = 122.0  # เสาใกล้

        self.param_distance = 10

        self.preShoot = -1
        self.stateShoot = 0
        self.preDriveMode = -1
        self.stateDriveMode = 0

        self.joyState = False

        self.cap = cv2.VideoCapture(0)
        self.width_frame, self.height_frame = (760, 480)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width_frame)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height_frame)
        self.debugState = False
        self.cheatState = 0

    def sub_callback(self, msg_in):  # subscription topic
        self.new_dat = msg_in
        # ? XBOX
        if msg_in.axes[5] == -1:
            self.button["L2"] = 1
        else:
            self.button["L2"] = 0
        if msg_in.axes[4] == -1:
            self.button["R2"] = 1
        else:
            self.button["R2"] = 0

        for index, element in enumerate(self.all):
            self.button[element] = msg_in.buttons[index]
        #     print(f"{self.all[index]}  :  {self.button[element]}")

        for index, element in enumerate(self.all2):
            if msg_in.axes[index] <= 0.1 and msg_in.axes[index] >= -0.1:
                self.axes[element] = 0
            else:
                self.axes[element] = msg_in.axes[index]

    def sub_callback_joyCon(self, msg_in):
        self.connectionString = msg_in.data
        if self.connectionString == "JoyCon":
            self.joyState = True
        else:
            self.joyState = False

    def sub_callback_cam(self, msg_in):
        self.distance = msg_in.data

    def sent_drive_callback(self):  # publisher drive topic
        limit = 0.1
        in_limit = -1 * limit
        msg = Twist()

        x = 0.0
        y = 0.0
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0

        if self.preDriveMode != self.button["L"]:
            self.preDriveMode = self.button["L"]
            if self.button["L"] == 1:
                self.stateDriveMode += 1

        if self.state_auto == 0:
            if (self.axes["AX"] != 0) or (self.axes["AY"] != 0):
                y = -1 * self.axes["AX"]
                x = -1 * self.axes["AY"]
                if self.stateDriveMode == 1:
                    x = np.interp(x, [-1, 1], [-0.392, 0.392])
                    y = np.interp(y, [-1, 1], [-0.392, 0.392])
                else:
                    self.stateDriveMode = 0

            else:
                y = -1 * self.axes["LX"]
                x = -1 * self.axes["LY"]
                if self.stateDriveMode > 1:
                    self.stateDriveMode = 0
                # if self.stateDriveMode == 1:
                #     if y != 0.0:
                #         y = (abs(y) / y) * 0.55
                #     if (x > limit) and (y > limit):
                #         x = 0.707
                #         y = 0.707
                #     elif (x < in_limit) and (y > limit):
                #         x = -0.707
                #         y = 0.707
                #     elif (x < in_limit) and (y < in_limit):
                #         x = -0.707
                #         y = -0.707
                #     elif (x > limit) and (y < in_limit):
                #         x = 0.707
                #         y = -0.707
                #     else:
                #         x = 0.0
                #         y = 0.0
                # elif self.stateDriveMode == 2:
                #     self.stateDriveMode = 0

        if (
            (self.button["PS"] == 1)
            and (self.axes["AX"] == 0)
            and (self.axes["AY"] == 0)
            and (self.axes["LY"] == 0)
            and (self.axes["LX"] == 0)
        ):
            self.state_auto = 1
        else:
            self.state_auto = 0

        if self.state_auto == 1:
            if self.distance > self.param_distance:
                x = 0.707
                y = 0.707
            elif self.distance < (self.param_distance * -1):
                x = -0.707
                y = 0.707
            else:
                self.state_auto = 0
                x = 0.0
                y = 0.0

        y = y * -1
        turn = np.interp(self.axes["RX"], [-1, 1], [-0.392, 0.392])
        theta = math.atan2(y, x)
        power = math.hypot(x, y)
        sin = math.sin(theta - math.pi / 4)
        cos = math.cos(theta - math.pi / 4)
        Max = max(abs(sin), abs(cos))
        leftFront = power * cos / Max + turn
        rightFront = power * sin / Max + turn
        leftBack = power * sin / Max - turn
        rightBack = power * cos / Max - turn

        if (power + abs(turn)) > 1:
            leftFront /= power + abs(turn)
            rightFront /= power + abs(turn)
            leftBack /= power + abs(turn)
            rightBack /= power + abs(turn)

        msg.linear.x = float(round(leftFront * 255))
        msg.linear.y = float(round(rightFront * 255))
        msg.angular.x = float(round(leftBack * 255))
        msg.angular.y = float(round(rightBack * 255))

        # //------------------------------------------------------------------------------------------------//
        if (self.button["L2"] == 1) and (self.button["R2"] == 1):
            self.assis_shoot = 0
        elif self.button["L2"] == 1:
            self.assis_shoot += 0.5
        elif self.button["R2"] == 1:
            self.assis_shoot -= 0.5

        # //------------------------------------------------------------------------------------------------//
        if (self.button["R"] == 1) and (self.counter == 0):
            self.state += 1
            self.counter = 4
        if self.counter > 0:
            self.counter -= 1
        if self.state > 2:
            self.state = 0
        if self.state < 0:
            self.state = 1
        if self.state == 0:
            self.pwm = self.param_pwm_motor2 - self.assis_shoot
        elif self.state == 1:
            self.pwm = self.param_pwm_motor1 - self.assis_shoot
        elif self.state == 2:
            self.pwm = self.param_pwm_motor0 + self.assis_shoot

        if self.pwm > 255:
            self.pwm = 255.0
        elif self.pwm < 0:
            self.pwm = 0.0

        # if self.button["X"] == 1:
        #     msg.linear.z = self.pwm

        if self.preShoot != self.button["X"]:
            self.preShoot = self.button["X"]
            if self.button["X"] == 1:
                self.stateShoot += 1

        if self.stateShoot == 0:
            msg.linear.z = 0.0
        elif self.stateShoot == 1:
            msg.linear.z = self.pwm
        elif self.stateShoot == 2:
            self.stateShoot = 0
        # //------------------------------------------------------------------------------------------------//

        if self.button["T"] == 1:
            msg.angular.z = 1.0
        elif self.button["L1"] == 1 and self.button["R1"] == 1:
            msg.angular.z = 30.0
        elif self.button["L1"] == 1:
            msg.angular.z = 10.0
        elif self.button["S"] == 1:
            msg.angular.z = 20.0

        elif self.button["R1"] == 1:
            msg.angular.z = 999.0
        elif self.button["O"] == 1:
            msg.angular.z = 888.0
        # //------------------------------------------------------------------------------------------------//

        if not self.joyState:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
        # //------------------------------------------------------------------------------------------------//
        tempMsg = [
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        ]
        for key_press_axes in self.axes:
            if self.axes[key_press_axes] != 0:
                if self.cheatState == 0 and self.axes["AY"] == 1:
                    self.cheatState = 1
                elif self.cheatState == 1 and self.axes["AY"] == -1:
                    self.cheatState = 2
                elif self.cheatState == 2 and self.axes["AX"] == 1:
                    self.cheatState = 3
                elif self.cheatState == 3 and self.axes["AX"] == -1:
                    self.cheatState = 4
                elif self.axes["LT"] != 0:
                    self.cheatState = self.cheatState
                elif self.axes["RT"] != 0:
                    self.cheatState = self.cheatState
                else:
                    self.cheatState = 0
        for key_press_button in self.button:
            if self.button[key_press_button] != 0:
                if self.cheatState == 4 and self.button["X"] == 1:
                    self.cheatState = 5
                elif self.cheatState == 5:
                    if self.debugState:
                        self.debugState = False
                    else:
                        self.debugState = True
                    self.cheatState = 0
                else:
                    self.cheatState = 0

        self.gui(tempMsg)

        # //------------------------------------------------------------------------------------------------//
        self.get_logger().info(str(self.pwm))
        self.sent_drive.publish(msg)

    def gui(self, msg_temp):
        block = [
            [50, int(self.height_frame - 50), 100, 50],
            [150, int(self.height_frame - 50), 100, 50],
            [250, int(self.height_frame - 50), 100, 50],
        ]

        text_msg_temp = [
            "msg.linear.x",
            "msg.linear.y",
            "msg.linear.z",
            "msg.angular.x",
            "msg.angular.y",
            "msg.angular.z",
        ]

        ret, frame = self.cap.read()
        if not ret:
            exit()

        frame = cv2.resize(frame, (self.width_frame, self.height_frame))
        frame = cv2.flip(frame, 1)
        frame = self.draw_power_bar(frame, self.pwm)

        self.draw_mode(frame=frame, block=block, mode=self.state)

        if self.debugState:
            cv2.rectangle(
                frame, (0, 0), (self.width_frame, self.height_frame), (0, 0, 0), -1
            )
            for i, value in enumerate(msg_temp):
                text_msg_size, _ = cv2.getTextSize(
                    f"{text_msg_temp[i]}: {value}", cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3
                )
                cv2.putText(
                    frame,
                    f"{text_msg_temp[i]}: {value}",
                    (
                        int((self.width_frame / 2) - (text_msg_size[0] / 2)),
                        int(i * 80 + 50),
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.5,
                    (0, 255, 0),
                    3,
                )

        cv2.namedWindow("Webcam", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("Webcam", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow("Webcam", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            # closing all open windows
            cv2.destroyAllWindows()
            exit()

    def draw_mode(self, frame, block, mode):
        text_block = ["N", "M", "F"]
        current_time = datetime.datetime.now().strftime("%H:%M:%S")
        text_size_time, _ = cv2.getTextSize(
            current_time, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3
        )
        for index, arr in enumerate(block):
            text = f"{text_block[index]}"
            x, y, width, height = arr
            if index == mode:
                cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 0), -1)
            else:
                cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 0, 255), -1)
            cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 0, 0), 2)

            text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3)
            text_x = x + int((width - text_size[0]) / 2)
            text_y = y + int((height + text_size[1]) / 2)
            cv2.putText(
                frame,
                text,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.5,
                (255, 0, 0),
                3,
            )
        cv2.putText(
            frame,
            current_time,
            (int((self.width_frame - text_size_time[0]) / 2), 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.5,
            (0, 255, 0),
            3,
            cv2.LINE_AA,
        )

    def interpolate_color(self, start_color, end_color, value):
        b = int(start_color[0] + (end_color[0] - start_color[0]) * value)
        g = int(start_color[1] + (end_color[1] - start_color[1]) * value)
        r = int(start_color[2] + (end_color[2] - start_color[2]) * value)
        return (b, g, r)

    def get_zone_color(self, value_pwm):
        color_zone1 = (0, 255, 0)
        color_zone2 = (0, 165, 255)
        color_zone3 = (0, 0, 255)

        if value_pwm <= 85:
            return color_zone1
        elif value_pwm <= 60:
            value = (value_pwm - 85) / (60 - 85)
            return self.interpolate_color(color_zone1, color_zone2, value)
        else:
            value = (value_pwm - 60) / (255 - 60)
            return self.interpolate_color(color_zone2, color_zone3, value)

    def draw_power_bar(self, frame, value_pwm):
        pwm_height = int((value_pwm / 255) * self.height_frame)
        pwm_color = self.get_zone_color(value_pwm)
        text_size_pwm, _ = cv2.getTextSize(
            f"{value_pwm}", cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3
        )
        text_x_pwm = 50 + int((150 - text_size_pwm[0]) / 2)
        text_y_pwm = int((80 + text_size_pwm[1]) / 2)
        cv2.rectangle(
            frame,
            (0, (self.height_frame - pwm_height)),
            (50, self.height_frame),
            pwm_color,
            -1,
        )
        cv2.rectangle(
            frame, (0, 0), (50, (self.height_frame - pwm_height)), (0, 0, 0), -1
        )
        cv2.rectangle(frame, (0, 0), (50, self.height_frame), (0, 0, 0), 2)

        cv2.rectangle(
            frame,
            (50, 0),
            (202, 80),
            (255, 255, 255),
            -1,
        )
        cv2.rectangle(
            frame,
            (50, 0),
            (202, 80),
            (0, 0, 0),
            2,
        )
        cv2.putText(
            frame,
            f"{value_pwm}",
            (text_x_pwm, text_y_pwm),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.5,
            (0, 255, 0),
            3,
            cv2.LINE_AA,
        )

        return frame


def main():
    rclpy.init()

    sub = Ps4()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
