import board
import busio
import digitalio
import adafruit_character_lcd.character_lcd as characterlcd
import adafruit_character_lcd.character_lcd_i2c as character_lcd_i2c
import time

# Define GPIO pins for the LCD
lcd_rs = digitalio.DigitalInOut(board.D26)
lcd_en = digitalio.DigitalInOut(board.D19)
lcd_d4 = digitalio.DigitalInOut(board.D13)
lcd_d5 = digitalio.DigitalInOut(board.D6)
lcd_d6 = digitalio.DigitalInOut(board.D5)
lcd_d7 = digitalio.DigitalInOut(board.D9)

# Define LCD column and row size
lcd_columns = 16
lcd_rows = 2

# i2c = board.I2C()
# i2c = busio.I2C(board.SCL, board.SDA)
# lcd = character_lcd_i2c.Character_LCD_I2C(i2c, lcd_columns, lcd_rows, address=0x27)

# Initialize LCD
lcd = characterlcd.Character_LCD_Mono(
    lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7, lcd_columns, lcd_rows
)

leftSide = bytes(
    [0b00111, 0b01111, 0b01111, 0b01111, 0b01111, 0b01111, 0b01111, 0b00111]
)

upperBar = bytes(
    [0b11111, 0b11111, 0b11111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000]
)

rightSide = bytes(
    [0b11100, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11100]
)

leftEnd = bytes(
    [0b01111, 0b00111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00011, 0b00111]
)

lowerBar = bytes(
    [0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111]
)

rightEnd = bytes(
    [0b11110, 0b11100, 0b00000, 0b00000, 0b00000, 0b00000, 0b11000, 0b11100]
)

middleBar = bytes(
    [0b11111, 0b11111, 0b11111, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111]
)

lowerEnd = bytes(
    [0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00111, 0b01111]
)

obj_num = [
    leftSide,
    upperBar,
    rightSide,
    leftEnd,
    lowerBar,
    rightEnd,
    middleBar,
    lowerEnd,
]


def displayInt(n: int, x: bytes, y: bytes, digits: bytes, leading: bool):
    if n < 0:
        n = abs(n)

    numString = []

    for i in range(digits):
        numString.append(-1)
    ind = digits - 1

    while ind:
        numString[ind] = int(n % 10)
        n /= 10
        ind -= 1

    numString[0] = int(n % 10)

    for i in range(digits):
        if (numString[i] == 0) and (not leading) and (i < digits - 1):
            clearNumber((i * 3) + x, y)
        else:
            displayNumber(numString[i], (i * 3) + x, y)
            leading = True


def clearNumber(x: bytes, y: bytes):
    lcd.cursor_position(x, y)
    lcd.message = "   "
    lcd.cursor_position(x, y + 1)
    lcd.message = "   "


def displayNumber(n: bytes, x: bytes, y: bytes):
    if n == 0:
        lcd.cursor_position(x, y)
        lcd.message = "\x00\x01\x02"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\x00\x04\x02"
    elif n == 1:
        lcd.cursor_position(x, y)
        lcd.message = "  \x02"
        lcd.cursor_position(x, y + 1)
        lcd.message = "  \x02"
    elif n == 2:
        lcd.cursor_position(x, y)
        lcd.message = "\x03\x06\x02"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\x00\x04\x04"
    elif n == 3:
        lcd.cursor_position(x, y)
        lcd.message = "\x03\x06\x02"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\x07\x04\x02"
    elif n == 4:
        lcd.cursor_position(x, y)
        lcd.message = "\x00\x04\x02"
        lcd.cursor_position(x, y + 1)
        lcd.message = "  \x02"
    elif n == 5:
        lcd.cursor_position(x, y)
        lcd.message = "\x00\x06\x02"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\x07\x04\x02"
    elif n == 6:
        lcd.cursor_position(x, y)
        lcd.message = "\x00\x06\x05"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\x00\x04\x02"
    elif n == 7:
        lcd.cursor_position(x, y)
        lcd.message = "\x01\x01\x02"
        lcd.cursor_position(x, y + 1)
        lcd.message = "  \x02"
    elif n == 8:
        lcd.cursor_position(x, y)
        lcd.message = "\x00\x06\x02"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\x00\x04\x02"
    elif n == 9:
        lcd.cursor_position(x, y)
        lcd.message = "\x00\x06\x02"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\x07\x04\x02"


for i in range(len(obj_num)):
    lcd.create_char(i, obj_num[i])

displayInt(120, 6, 0, 3, False)

# Clean up LCD
# lcd.clear()
