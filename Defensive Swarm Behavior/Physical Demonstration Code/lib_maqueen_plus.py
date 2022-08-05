# maqueen library
import microbit
import machine
import time

class Robot:
    def __init__(self):
        self._vit = 0
        microbit.i2c.init(freq=100000, sda=microbit.pin20, scl=microbit.pin19)
        # Motors
        self.MG = 0
        self.MD = 1

        # ServoMotors
        self.S1 = 1
        self.S2 = 2
        self.S3 = 3

    def run(self, mot, sens, vit):
        # mot left:0 ; mot right:1
        # sens forward : 1; sens backward :2
        # speed max :255; stop :0
        buf = bytearray(3)
        if mot == self.MG:
            buf[0] = 0x00
        else:
            buf[0] = 0x02
        buf[1] = sens
        buf[2] = vit
        microbit.i2c.write(0x10, buf)

    def stop(self):
        """Stop the robot
        """
        self.run(self.MG, 1, 0)
        self.run(self.MD, 1, 0)
        microbit.display.show('S')

    def ultrasonic(self):
        """Get the distance between the robot and an object.

        Returns:
            [float]: distance to the object it one is detected else max value.
        """
        #trig_pin = pin13, echo_pin = pin14
        microbit.pin13.write_digital(1)
        time.sleep_ms(10)
        microbit.pin13.write_digital(0)

        microbit.pin14.read_digital()
        t2 = machine.time_pulse_us(microbit.pin14, 1)
        distance = (t2/2) / 29.1#t2 *100 / 582#343 * (t2 / (2*400))

        return distance # in cm

    def motor_speed(self, mot):
        """Get the linear speed of a given motor

        Args:
            mot (int): object attribut define in constructor (MG, MD)

        Returns:
            [type]: [description]
        """
        microbit.i2c.write(0x10, b'\x00')
        # 0x10 -> adresse / 4 -> lecture de 4 bytes
        speed_x = microbit.i2c.read(0x10, 4)
        return_speed = -1

        # 0 -> G / 1 -> D
        if(mot == self.MG):
            if(round(speed_x[1]) < 20 and round(speed_x[1]) != 0):
                return_speed = round(speed_x[1]) + 255
            else:
                return_speed = round(speed_x[1])

        elif(mot == self.MD):
            if(round(speed_x[3]) < 20 and round(speed_x[3]) != 0):
                return_speed = round(speed_x[3]) + 255
            else:
                return_speed = round(speed_x[3])

        return return_speed
