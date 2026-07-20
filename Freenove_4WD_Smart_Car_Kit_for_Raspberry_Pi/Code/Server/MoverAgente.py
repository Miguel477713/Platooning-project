import curses
import time
from pca9685 import PCA9685

class Ordinary_Car:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.set_pwm_freq(50)
    def duty_range(self, duty1, duty2, duty3, duty4):
        if duty1 > 4095:
            duty1 = 4095
        elif duty1 < -4095:
            duty1 = -4095        
        if duty2 > 4095:
            duty2 = 4095
        elif duty2 < -4095:
            duty2 = -4095  
        if duty3 > 4095:
            duty3 = 4095
        elif duty3 < -4095:
            duty3 = -4095
        if duty4 > 4095:
            duty4 = 4095
        elif duty4 < -4095:
            duty4 = -4095
        return duty1,duty2,duty3,duty4
    def left_upper_wheel(self,duty):
        if duty>0:
            self.pwm.set_motor_pwm(0,0)
            self.pwm.set_motor_pwm(1,duty)
        elif duty<0:
            self.pwm.set_motor_pwm(1,0)
            self.pwm.set_motor_pwm(0,abs(duty))
        else:
            self.pwm.set_motor_pwm(0,4095)
            self.pwm.set_motor_pwm(1,4095)
    def left_lower_wheel(self,duty):
        if duty>0:
            self.pwm.set_motor_pwm(3,0)
            self.pwm.set_motor_pwm(2,duty)
        elif duty<0:
            self.pwm.set_motor_pwm(2,0)
            self.pwm.set_motor_pwm(3,abs(duty))
        else:
            self.pwm.set_motor_pwm(2,4095)
            self.pwm.set_motor_pwm(3,4095)
    def right_upper_wheel(self,duty):
        if duty>0:
            self.pwm.set_motor_pwm(6,0)
            self.pwm.set_motor_pwm(7,duty)
        elif duty<0:
            self.pwm.set_motor_pwm(7,0)
            self.pwm.set_motor_pwm(6,abs(duty))
        else:
            self.pwm.set_motor_pwm(6,4095)
            self.pwm.set_motor_pwm(7,4095)
    def right_lower_wheel(self,duty):
        if duty>0:
            self.pwm.set_motor_pwm(4,0)
            self.pwm.set_motor_pwm(5,duty)
        elif duty<0:
            self.pwm.set_motor_pwm(5,0)
            self.pwm.set_motor_pwm(4,abs(duty))
        else:
            self.pwm.set_motor_pwm(4,4095)
            self.pwm.set_motor_pwm(5,4095)
    def set_motor_model(self, duty1, duty2, duty3, duty4):
        duty1,duty2,duty3,duty4=self.duty_range(duty1,duty2,duty3,duty4)
        self.left_upper_wheel(duty1)
        self.left_lower_wheel(duty2)
        self.right_upper_wheel(duty3)
        self.right_lower_wheel(duty4)

    def close(self):
        self.set_motor_model(0,0,0,0)
        self.pwm.close()

PWM = Ordinary_Car()  

def BottomEvent(stdscr):
    curses.curs_set(0) # Ocultar el cursor del mouse
    stdscr.nodelay(True) # No bloquear esperando la tecla
    stdscr.clear()
    stdscr.addstr(0, 0, "Presiona las flechas (Presiona 'q' para salir)")

    while True:
        try:
            tecla = stdscr.getch()

            if tecla == curses.KEY_UP:
                stdscr.addstr(2, 0, "? Presionaste ARRIBA   ")
                PWM.set_motor_model(2000,2000,2000,2000)
                time.sleep(1)
            elif tecla == curses.KEY_DOWN:
                stdscr.addstr(2, 0, "? Presionaste ABAJO    ")
                PWM.set_motor_model(-2000,-2000,-2000,-2000)   #Back
                time.sleep(1)
            elif tecla == curses.KEY_LEFT:
                stdscr.addstr(2, 0, "? Presionaste IZQUIERDA")
                PWM.set_motor_model(-2000,-2000,2000,2000)     #Left 
                time.sleep(1)
            elif tecla == curses.KEY_RIGHT:
                stdscr.addstr(2, 0, "? Presionaste DERECHA  ")
                PWM.set_motor_model(2000,2000,-2000,-2000)     #Right    
                time.sleep(1)
            elif tecla == ord('q'): # Si presionas la letra 'q', sale
                PWM.set_motor_model(0,0,0,0)                   #Stop
            elif tecla == ord('w'):
                PWM.close()
                break
                
        except KeyboardInterrupt:
            break

curses.wrapper(BottomEvent)


