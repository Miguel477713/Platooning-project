from ultrasonic import Ultrasonic
from motor import Ordinary_Car
from servo import Servo
from infrared import Infrared
from adc import ADC
import time
import math

class Car:
    def __init__(self):
        self.car_record_time = time.time()
        self.motor = None
        self.infrared = None
    def mode_infrared(self):
        if (time.time() - self.car_record_time) > 0.2:
            self.car_record_time = time.time()
            infrared_value = self.infrared.read_all_infrared()
            #print("infrared_value: " + str(infrared_value))
            if infrared_value == 2:
                self.motor.set_motor_model(800,800,800,800)
            elif infrared_value == 4:
                self.motor.set_motor_model(-1500,-1500,2500,2500)
            elif infrared_value == 6:
                self.motor.set_motor_model(-2000,-2000,4000,4000)
            elif infrared_value == 1:
                self.motor.set_motor_model(2500,2500,-1500,-1500)
            elif infrared_value == 3:
                self.motor.set_motor_model(4000,4000,-2000,-2000)
            elif infrared_value == 7:
                self.motor.set_motor_model(0,0,0,0)
def test_car_infrared():
    car = Car()
    try:
        while True:
            car.mode_infrared()
    except KeyboardInterrupt:
        car.close()
        print("\nEnd of program")

if __name__ == '__main__':
    test_car_infrared()
