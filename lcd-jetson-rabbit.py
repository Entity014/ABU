import board
import busio
import digitalio
import adafruit_character_lcd.character_lcd as characterlcd
import adafruit_character_lcd.character_lcd_i2c as character_lcd_i2c

# Define GPIO pins for the LCD
lcd_rs = digitalio.DigitalInOut(board.D26)
lcd_en = digitalio.DigitalInOut(board.D19)
lcd_d4 = digitalio.DigitalInOut(board.D13)
lcd_d5 = digitalio.DigitalInOut(board.D6)
lcd_d6 = digitalio.DigitalInOut(board.D5)
lcd_d7 = digitalio.DigitalInOut(board.D9)

# Define LCD column and row size
lcd_columns = 20
lcd_rows = 4

# i2c = board.I2C()
# i2c = busio.I2C(board.SCL, board.SDA)
# lcd = character_lcd_i2c.Character_LCD_I2C(i2c, lcd_columns, lcd_rows, address=0x27)

# Initialize LCD
lcd = characterlcd.Character_LCD_Mono(
    lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7, lcd_columns, lcd_rows
)

botRightTri = bytes(
    [0b00000, 0b00000, 0b00000, 0b00000, 0b00001, 0b00111, 0b01111, 0b11111]
)

botBlock = bytes(
    [0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111]
)

botLeftTri = bytes(
    [0b00000, 0b00000, 0b00000, 0b00000, 0b10000, 0b11100, 0b11110, 0b11111]
)

topRightTri = bytes(
    [0b11111, 0b01111, 0b00111, 0b00001, 0b00000, 0b00000, 0b00000, 0b00000]
)

topBlock = bytes(
    [0b11111, 0b11111, 0b11111, 0b11111, 0b00000, 0b00000, 0b00000, 0b00000]
)

topLeftTri = bytes(
    [0b11111, 0b11110, 0b11100, 0b10000, 0b00000, 0b00000, 0b00000, 0b00000]
)

fullTopRightTri = bytes(
    [0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b01111, 0b00111, 0b00001]
)

fullTopLeftTri = bytes(
    [0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11110, 0b11100, 0b10000]
)


obj_num = [
    botRightTri,
    botBlock,
    botLeftTri,
    topRightTri,
    topBlock,
    topLeftTri,
    fullTopRightTri,
    fullTopLeftTri,
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
            clearNumber((i * 4) + x, y)
        else:
            displayNumber(numString[i], (i * 4) + x, y)
            leading = True


def clearNumber(x: bytes, y: bytes):
    lcd.cursor_position(x, y)
    lcd.message = "   "
    lcd.cursor_position(x, y + 1)
    lcd.message = "   "
    lcd.cursor_position(x, y + 2)
    lcd.message = "   "
    lcd.cursor_position(x, y + 3)
    lcd.message = "   "


def displayNumber(n: bytes, x: bytes, y: bytes):
    if n == 0:
        lcd.cursor_position(x, y)
        lcd.message = "\x00\x01\x02"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\xFF\x00\xFF"
        lcd.cursor_position(x, y + 2)
        lcd.message = "\xFF\x05\xFF"
        lcd.cursor_position(x, y + 3)
        lcd.message = "\x03\x04\x05"
    elif n == 1:
        lcd.cursor_position(x, y)
        lcd.message = "\x00\x01\xFE"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\x05\xFF\xFE"
        lcd.cursor_position(x, y + 2)
        lcd.message = "\xFE\xFF\xFE"
        lcd.cursor_position(x, y + 3)
        lcd.message = "\xFE\x04\xFE"
    elif n == 2:
        lcd.cursor_position(x, y)
        lcd.message = "\x00\x01\x02"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\x04\x00\x07"
        lcd.cursor_position(x, y + 2)
        lcd.message = "\xFF\x05\xFE"
        lcd.cursor_position(x, y + 3)
        lcd.message = "\x04\x04\x04"
    elif n == 3:
        lcd.cursor_position(x, y)
        lcd.message = "\x00\x01\x02"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\x04\x00\x07"
        lcd.cursor_position(x, y + 2)
        lcd.message = "\x01\x03\xFF"
        lcd.cursor_position(x, y + 3)
        lcd.message = "\x03\x04\x05"
    elif n == 4:
        lcd.cursor_position(x, y)
        lcd.message = "\x00\xFE\x01"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\xFF\xFE\xFF"
        lcd.cursor_position(x, y + 2)
        lcd.message = "\x04\x04\xFF"
        lcd.cursor_position(x, y + 3)
        lcd.message = "\xFE\xFE\x04"
    elif n == 5:
        lcd.cursor_position(x, y)
        lcd.message = "\x01\x01\x01"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\x06\x01\x02"
        lcd.cursor_position(x, y + 2)
        lcd.message = "\x01\xFE\xFF"
        lcd.cursor_position(x, y + 3)
        lcd.message = "\x03\x04\x05"
    elif n == 6:
        lcd.cursor_position(x, y)
        lcd.message = "\x00\x01\x02"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\xFF\x01\x02"
        lcd.cursor_position(x, y + 2)
        lcd.message = "\xFF\xFE\xFF"
        lcd.cursor_position(x, y + 3)
        lcd.message = "\x03\x04\x05"
    elif n == 7:
        lcd.cursor_position(x, y)
        lcd.message = "\x01\x01\x01"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\xFE\x00\x07"
        lcd.cursor_position(x, y + 2)
        lcd.message = "\xFF\x05\xFE"
        lcd.cursor_position(x, y + 3)
        lcd.message = "\x04\xFE\xFE"
    elif n == 8:
        lcd.cursor_position(x, y)
        lcd.message = "\x00\x01\x02"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\x06\x01\x07"
        lcd.cursor_position(x, y + 2)
        lcd.message = "\xFF\xFE\xFF"
        lcd.cursor_position(x, y + 3)
        lcd.message = "\x03\x04\x05"
    elif n == 9:
        lcd.cursor_position(x, y)
        lcd.message = "\x00\x01\x02"
        lcd.cursor_position(x, y + 1)
        lcd.message = "\x06\x01\xFF"
        lcd.cursor_position(x, y + 2)
        lcd.message = "\xFE\xFE\xFF"
        lcd.cursor_position(x, y + 3)
        lcd.message = "\x03\x04\x05"


for i in range(len(obj_num)):
    lcd.create_char(i, obj_num[i])

displayInt(120, 6, 0, 3, False)

# Clean up LCD
# lcd.clear()
