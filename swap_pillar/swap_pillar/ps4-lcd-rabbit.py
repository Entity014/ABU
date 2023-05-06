import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from rclpy import qos

# import board
# import digitalio
# import adafruit_character_lcd.character_lcd as characterlcd
import math


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

        self.sent_drive = self.create_publisher(
            Twist, "control_drive_topic", qos_profile=qos.qos_profile_system_default
        )
        self.sent_drive_timer = self.create_timer(0.05, self.sent_drive_callback)

        self.button = {}
        self.all = ["X", "O", "T", "S", "L1", "R1", "L2", "R2", "L", "R", "PS"]
        for index, element in enumerate(self.all):
            self.button[element] = 0

        self.axes = {}
        # self.all2 = ["LX", "LY", "RX", "LT", "RT", "RY"]
        self.all2 = ["LX", "LY", "RX", "RY", "RT", "RT", "AX", "AY"]
        for index, element in enumerate(self.all2):
            self.axes[element] = 0
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

        self.param_pwm_motor1 = 165.0  # ค่าสำหรับห่วงไกล
        self.param_pwm_motor2 = 105.0  # ค่าสำหรับห่วงใกล้
        self.param_distance = 10

        for a in self.all:
            self.button[a] = 0
        for a in self.all2:
            self.axes[a] = 0
        self.axes["AX"] = 0
        self.axes["AY"] = 0

        self.preShoot = -1
        self.stateShoot = 0
        self.preDriveMode = -1
        self.stateDriveMode = 0

        # TODO: LCD
        # lcd_columns = 20
        # lcd_rows = 4
        # lcd_rs = digitalio.DigitalInOut(board.D26)
        # lcd_en = digitalio.DigitalInOut(board.D19)
        # lcd_d4 = digitalio.DigitalInOut(board.D13)
        # lcd_d5 = digitalio.DigitalInOut(board.D6)
        # lcd_d6 = digitalio.DigitalInOut(board.D5)
        # lcd_d7 = digitalio.DigitalInOut(board.D9)

        # self.lcd = characterlcd.Character_LCD_Mono(
        #     lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7, lcd_columns, lcd_rows
        # )
        #

        # botRightTri = bytes(
        #     [0b00000, 0b00000, 0b00000, 0b00000, 0b00001, 0b00111, 0b01111, 0b11111]
        # )

        # botBlock = bytes(
        #     [0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111]
        # )

        # botLeftTri = bytes(
        #     [0b00000, 0b00000, 0b00000, 0b00000, 0b10000, 0b11100, 0b11110, 0b11111]
        # )

        # topRightTri = bytes(
        #     [0b11111, 0b01111, 0b00111, 0b00001, 0b00000, 0b00000, 0b00000, 0b00000]
        # )

        # topBlock = bytes(
        #     [0b11111, 0b11111, 0b11111, 0b11111, 0b00000, 0b00000, 0b00000, 0b00000]
        # )

        # topLeftTri = bytes(
        #     [0b11111, 0b11110, 0b11100, 0b10000, 0b00000, 0b00000, 0b00000, 0b00000]
        # )

        # fullTopRightTri = bytes(
        #     [0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b01111, 0b00111, 0b00001]
        # )

        # fullTopLeftTri = bytes(
        #     [0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11110, 0b11100, 0b10000]
        # )

        # obj_num = [
        #     botRightTri,
        #     botBlock,
        #     botLeftTri,
        #     topRightTri,
        #     topBlock,
        #     topLeftTri,
        #     fullTopRightTri,
        #     fullTopLeftTri,
        # ]

        # for i in range(len(obj_num)):
        #     self.lcd.create_char(i, obj_num[i])

    def sub_callback(self, msg_in):  # subscription topic
        self.new_dat = msg_in
        self.button["X"] = msg_in.buttons[0]
        self.button["O"] = msg_in.buttons[1]
        self.button["T"] = msg_in.buttons[4]
        self.button["S"] = msg_in.buttons[3]
        self.button["L1"] = msg_in.buttons[6]
        self.button["R1"] = msg_in.buttons[7]
        if msg_in.axes[5] < 0:
            self.button["L2"] = 1
        else:
            self.button["L2"] = 0
        if msg_in.axes[4] < 0:
            self.button["R2"] = 1
        else:
            self.button["R2"] = 0
        self.button["L"] = msg_in.buttons[10]
        self.button["R"] = msg_in.buttons[11]
        self.button["PS"] = msg_in.buttons[-1]
        # for index, element in enumerate(self.all):
        #     self.button[element] = msg_in.buttons[index]
        # 			print(f"{self.all[index]}  :  {self.button[element]}")

        for index, element in enumerate(self.all2):
            if msg_in.axes[index] <= 0.1 and msg_in.axes[index] >= -0.1:
                self.axes[element] = 0
            else:
                self.axes[element] = msg_in.axes[index]
        self.axes["AX"] = msg_in.axes[-2]
        self.axes["AY"] = msg_in.axes[-1]

    def sub_callback_cam(self, msg_in):
        self.distance = msg_in.data

    def sent_drive_callback(self):  # publisher drive topic
        limit = 0.1
        in_limit = -1 * limit
        msg = Twist()

        x = 0.0
        y = 0.0

        if self.preDriveMode != self.button["L"]:
            self.preDriveMode = self.button["L"]
            if self.button["L"] == 1:
                self.stateDriveMode += 1

        if self.state_auto == 0:
            if (self.axes["AX"] > 0) or (self.axes["AY"] > 0):
                y = -1 * (self.axes["AX"])
                x = -1 * self.axes["AY"]
            elif (self.axes["AX"] < 0) or (self.axes["AY"] < 0):
                y = -1 * (self.axes["AX"])
                x = -1 * self.axes["AY"]
            else:
                y = -1 * (self.axes["LX"])
                x = -1 * self.axes["LY"]
                if self.stateDriveMode == 1:
                    if (x > limit) and (y > limit):
                        x = 0.707
                        y = 0.707
                    elif (x < in_limit) and (y > limit):
                        x = -0.707
                        y = 0.707
                    elif (x < in_limit) and (y < in_limit):
                        x = -0.707
                        y = -0.707
                    elif (x > limit) and (y < in_limit):
                        x = 0.707
                        y = -0.707
                    else:
                        x = 0.0
                        y = 0.0
                elif self.stateDriveMode == 2:
                    self.stateDriveMode = 0

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
        turn = self.axes["RX"]
        theta = math.atan2(y, x)
        power = math.hypot(x, y)
        sin = math.sin(theta - math.pi / 4)
        cos = math.cos(theta - math.pi / 4)
        Max = max(abs(sin), abs(cos))
        leftFront = power * cos / Max + turn
        rightFront = power * sin / Max - turn
        leftBack = power * sin / Max + turn
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
        if self.state > 1:
            self.state = 0
        if self.state < 0:
            self.state = 1
        if self.state == 0:
            self.pwm = self.param_pwm_motor2 - self.assis_shoot
        elif self.state == 1:
            self.pwm = self.param_pwm_motor1 - self.assis_shoot

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
        else:
            msg.angular.z = -1 * self.pwm

        self.get_logger().info(str(self.pwm))
        self.sent_drive.publish(msg)
        # self.displayInt(self.pwm, 6, 0, 3, False)

    # TODO: LCD Functions
    # def displayInt(self, n: int, x: bytes, y: bytes, digits: bytes, leading: bool):
    #     if n < 0:
    #         n = abs(n)

    #     numString = []

    #     for i in range(digits):
    #         numString.append(-1)
    #     ind = digits - 1

    #     while ind:
    #         numString[ind] = int(n % 10)
    #         n /= 10
    #         ind -= 1

    #     numString[0] = int(n % 10)

    #     for i in range(digits):
    #         if (numString[i] == 0) and (not leading) and (i < digits - 1):
    #             self.clearNumber((i * 4) + x, y)
    #         else:
    #             self.displayNumber(numString[i], (i * 4) + x, y)
    #             leading = True

    # def clearNumber(self, x: bytes, y: bytes):
    #     self.lcd.cursor_position(x, y)
    #     self.lcd.message = "   "
    #     self.lcd.cursor_position(x, y + 1)
    #     self.lcd.message = "   "
    #     self.lcd.cursor_position(x, y + 2)
    #     self.lcd.message = "   "
    #     self.lcd.cursor_position(x, y + 3)
    #     self.lcd.message = "   "

    # def displayNumber(self, n: bytes, x: bytes, y: bytes):
    #     if n == 0:
    #         self.lcd.cursor_position(x, y)
    #         self.lcd.message = "\x00\x01\x02"
    #         self.lcd.cursor_position(x, y + 1)
    #         self.lcd.message = "\xFF\x00\xFF"
    #         self.lcd.cursor_position(x, y + 2)
    #         self.lcd.message = "\xFF\x05\xFF"
    #         self.lcd.cursor_position(x, y + 3)
    #         self.lcd.message = "\x03\x04\x05"
    #     elif n == 1:
    #         self.lcd.cursor_position(x, y)
    #         self.lcd.message = "\x00\x01\xFE"
    #         self.lcd.cursor_position(x, y + 1)
    #         self.lcd.message = "\x05\xFF\xFE"
    #         self.lcd.cursor_position(x, y + 2)
    #         self.lcd.message = "\xFE\xFF\xFE"
    #         self.lcd.cursor_position(x, y + 3)
    #         self.lcd.message = "\xFE\x04\xFE"
    #     elif n == 2:
    #         self.lcd.cursor_position(x, y)
    #         self.lcd.message = "\x00\x01\x02"
    #         self.lcd.cursor_position(x, y + 1)
    #         self.lcd.message = "\x04\x00\x07"
    #         self.lcd.cursor_position(x, y + 2)
    #         self.lcd.message = "\xFF\x05\xFE"
    #         self.lcd.cursor_position(x, y + 3)
    #         self.lcd.message = "\x04\x04\x04"
    #     elif n == 3:
    #         self.lcd.cursor_position(x, y)
    #         self.lcd.message = "\x00\x01\x02"
    #         self.lcd.cursor_position(x, y + 1)
    #         self.lcd.message = "\x04\x00\x07"
    #         self.lcd.cursor_position(x, y + 2)
    #         self.lcd.message = "\x01\x03\xFF"
    #         self.lcd.cursor_position(x, y + 3)
    #         self.lcd.message = "\x03\x04\x05"
    #     elif n == 4:
    #         self.lcd.cursor_position(x, y)
    #         self.lcd.message = "\x00\xFE\x01"
    #         self.lcd.cursor_position(x, y + 1)
    #         self.lcd.message = "\xFF\xFE\xFF"
    #         self.lcd.cursor_position(x, y + 2)
    #         self.lcd.message = "\x04\x04\xFF"
    #         self.lcd.cursor_position(x, y + 3)
    #         self.lcd.message = "\xFE\xFE\x04"
    #     elif n == 5:
    #         self.lcd.cursor_position(x, y)
    #         self.lcd.message = "\x01\x01\x01"
    #         self.lcd.cursor_position(x, y + 1)
    #         self.lcd.message = "\x06\x01\x02"
    #         self.lcd.cursor_position(x, y + 2)
    #         self.lcd.message = "\x01\xFE\xFF"
    #         self.lcd.cursor_position(x, y + 3)
    #         self.lcd.message = "\x03\x04\x05"
    #     elif n == 6:
    #         self.lcd.cursor_position(x, y)
    #         self.lcd.message = "\x00\x01\x02"
    #         self.lcd.cursor_position(x, y + 1)
    #         self.lcd.message = "\xFF\x01\x02"
    #         self.lcd.cursor_position(x, y + 2)
    #         self.lcd.message = "\xFF\xFE\xFF"
    #         self.lcd.cursor_position(x, y + 3)
    #         self.lcd.message = "\x03\x04\x05"
    #     elif n == 7:
    #         self.lcd.cursor_position(x, y)
    #         self.lcd.message = "\x01\x01\x01"
    #         self.lcd.cursor_position(x, y + 1)
    #         self.lcd.message = "\xFE\x00\x07"
    #         self.lcd.cursor_position(x, y + 2)
    #         self.lcd.message = "\xFF\x05\xFE"
    #         self.lcd.cursor_position(x, y + 3)
    #         self.lcd.message = "\x04\xFE\xFE"
    #     elif n == 8:
    #         self.lcd.cursor_position(x, y)
    #         self.lcd.message = "\x00\x01\x02"
    #         self.lcd.cursor_position(x, y + 1)
    #         self.lcd.message = "\x06\x01\x07"
    #         self.lcd.cursor_position(x, y + 2)
    #         self.lcd.message = "\xFF\xFE\xFF"
    #         self.lcd.cursor_position(x, y + 3)
    #         self.lcd.message = "\x03\x04\x05"
    #     elif n == 9:
    #         self.lcd.cursor_position(x, y)
    #         self.lcd.message = "\x00\x01\x02"
    #         self.lcd.cursor_position(x, y + 1)
    #         self.lcd.message = "\x06\x01\xFF"
    #         self.lcd.cursor_position(x, y + 2)
    #         self.lcd.message = "\xFE\xFE\xFF"
    #         self.lcd.cursor_position(x, y + 3)
    #         self.lcd.message = "\x03\x04\x05"


def main():
    rclpy.init()

    sub = Ps4()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
