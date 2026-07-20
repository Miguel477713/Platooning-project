import time
import threading
from motor import Ordinary_Car
from infrared import Infrared

class MecanumPIDLineFollower:
    def __init__(self):
        self.motor = Ordinary_Car()
        self.infrared = Infrared()
        self.is_running = False
        self.thread = None
        
        # =======================================================
        # VARIABLES DE CONFIGURACIÓN FÁCIL
        # =======================================================
        # 1. Si al iniciar el coche va hacia ATRÁS, cambia este valor a -900
        self.base_speed = -700 
        
        # 2. Si el coche "huye" de la línea en lugar de buscarla, cambia esto a True
        self.invertir_correccion = False
        
        # =======================================================
        
        # Constantes PID
        self.Kp = 300
        self.Ki = 0
        self.Kd = 250
        
        self.previous_error = 0
        self.integral = 0
        self.max_strafe = 1000

    def get_error(self, sensor_value):
        # Mapeo de los sensores infrarrojos (Izquierda - Centro - Derecha)
        
        # Si la línea está a la DERECHA del coche (001 o 011)
        if sensor_value == 1:   
            return -2 
        elif sensor_value == 3: 
            return -1 
            
        # Si la línea está en el CENTRO exacto (010)
        elif sensor_value == 2: 
            return 0  
            
        # Si la línea está a la IZQUIERDA del coche (100 o 110)
        elif sensor_value == 4: 
            return 2  
        elif sensor_value == 6: 
            return 1  
            
        # Si perdió la línea por completo (000)
        elif sensor_value == 0: 
            if self.previous_error > 0: return 3
            elif self.previous_error < 0: return -3
            else: return 0
            
        # Cruce o ruido (111 u otros)
        else:
            return 0

    def start(self):
        self.is_running = True
        self.thread = threading.Thread(target=self.track_line)
        self.thread.daemon = True
        self.thread.start()
        print("PID Mecanum Iniciado.")

    def track_line(self):
        try:
            while self.is_running:
                sensor_value = self.infrared.read_all_infrared()
                error = self.get_error(sensor_value)
                
                # Cálculo Matemático del PID
                P = self.Kp * error
                self.integral += error
                I = self.Ki * self.integral
                D = self.Kd * (error - self.previous_error)
                
                PID_value = P + I + D
                self.previous_error = error
                
                if PID_value > self.max_strafe: PID_value = self.max_strafe
                elif PID_value < -self.max_strafe: PID_value = -self.max_strafe
                
                # --- MATRIZ MECANUM ---
                LY = self.base_speed
                
                # Deslizamiento lateral principal
                if self.invertir_correccion:
                    LX = -int(PID_value)
                else:
                    LX = int(PID_value)
                
                # ROTACIÓN DINÁMICA PARA CURVAS
                # Si el error es máximo (2 o -2), significa que la línea está en el extremo del sensor.
                # En lugar de solo deslizarse, inyectamos rotación para que el chasis apunte hacia la curva.
                if abs(error) >= 2:
                    # Multiplicador de rotación. Un valor de 0.5 significa que usará
                    # la mitad de la fuerza del PID para rotar el chasis.
                    factor_rotacion = 0.1
                    
                    if self.invertir_correccion:
                        RX = -int(PID_value * factor_rotacion)
                    else:
                        RX = int(PID_value * factor_rotacion)
                else:
                    # Si el error es pequeño (micro-correcciones en recta), no rota, solo se desliza.
                    RX = 0 
                
                # Ecuaciones de la matriz de mezcla para ruedas Mecanum
                FR = LY - LX + RX
                FL = LY + LX - RX
                BL = LY - LX - RX
                BR = LY + LX + RX
                
                self.motor.set_motor_model(FL, BL, FR, BR)
                
                print(f"Sensores (Izq-Cen-Der): {sensor_value:03b} | Error: {error} | PID: {PID_value}")
                time.sleep(0.02)
                
        except Exception as e:
            print(f"Error en PID: {e}")
        finally:
            self.motor.set_motor_model(0, 0, 0, 0)

    def stop(self):
        self.is_running = False
        if self.thread is not None:
            self.thread.join()
        self.motor.close()
        self.infrared.close()
        print("Finalizado.")

if __name__ == '__main__':
    robot = MecanumPIDLineFollower()
    try:
        robot.start()
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        robot.stop()