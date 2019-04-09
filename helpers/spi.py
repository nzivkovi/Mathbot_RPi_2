import Adafruit_GPIO.SPI as SPI
import time

GET_SENSOR_5 = 0xff
GET_SENSOR_4 = 0xfe
GET_SENSOR_3 = 0xfd
GET_SENSOR_2 = 0xfc
GET_SENSOR_1 = 0xfb

GET_TICKS_L = 0xfa
GET_TICKS_R = 0xf9

SET_ANGULAR_VELOCITY_L = 0xf8
SET_ANGULAR_VELOCITY_R = 0xf7

MOCK_SENSOR_DISTANCES = [30, 30, 30, 30, 30]

def milliseconds():
    return int(round(time.time() * 1000))


class SpiMaster:
    __slots__ = ['spi']

    def __init__(self):
        self.spi = SPI.SpiDev(0, 0, max_speed_hz=1000)

    def send_velocities(self, left, right):
        left *= 1000
        right *= 1000
        #print(int(left), int(right))
        self._send_value(SET_ANGULAR_VELOCITY_L, int(left))
        time1 = milliseconds()
        self._send_value(SET_ANGULAR_VELOCITY_R, int(right))
        time2 = milliseconds()
        print(time2, time1)

    def get_ticks(self):
        ticks_left = self._read_value(GET_TICKS_L)
        ticks_right = self._read_value(GET_TICKS_R)
        #print(ticks_left, ticks_right)
        return ticks_left, ticks_right

    def get_sensors(self):
        sensor_distances = 5 * [0.0]
        for i in range(5):
            sensor_distances[i] = self.get_sensor(i) / 100.0
        return sensor_distances
        #return MOCK_SENSOR_DISTANCES
            
    def get_sensor(self, i):
        if i == 0:
            command = GET_SENSOR_1
        elif i == 1:
            command = GET_SENSOR_2
        elif i == 2:
            command = GET_SENSOR_3
        elif i == 3:
            command = GET_SENSOR_4
        elif i == 4:
            command = GET_SENSOR_5
        return self._read_value(command)

    def _send_value(self, command: int, value: int):
        self.spi.write([command])
        # time.sleep(0.005)
        for i in [value >> i & 0xff for i in (24, 16, 8, 0)]:
            self.spi.write([i])

    def _read_value(self, command: int):
        self.spi.write([command])
        time.sleep(0.005)
        byte_array = bytearray()
        for i in range(4):
            byte_array += self.spi.read(1)
        value = int.from_bytes(byte_array, byteorder='little', signed=True)
        return value


if __name__ == '__main__':
    master = SpiMaster()
    #master.send_velocities(10, 10)
    #time.sleep(5)
    #master.send_velocities(-13, -13)
    while(True):
      #master.get_ticks()
      print(master.get_sensors())
      #print(master.ticks_left)
      #print(master.ticks_right)

