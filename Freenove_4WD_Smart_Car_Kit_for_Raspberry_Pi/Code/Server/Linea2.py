import time
import threading
from motor import Ordinary_Car
from infrared import Infrared

class SeguidorMecanumPID:
    def __init__(self):
        # Constantes del PID 
        self.Kp = 260.0  
        self.Kd = 450.0  
        self.Ki = 0.0 
        
        self.velocidad_base = -1000 
        self.invertir_correccion = True 
        
        # Variables de memoria para el PID
        self.proporcional_pasado = 0
        self.integral = 0
        self.max_integral = 1000 
        self.ultimo_error_conocido = 0
        
    def leer_sensores_y_error(self, izq, cen, der):
        """Traduce los 3 sensores a un error numérico"""
        if izq == 0 and cen == 1 and der == 0: error = 0   # Centro perfecto
        elif izq == 0 and cen == 1 and der == 1: error = 1 # Ligeramente a la derecha
        elif izq == 0 and cen == 0 and der == 1: error = 2 # Muy a la derecha
        elif izq == 1 and cen == 1 and der == 0: error = -1 # Ligeramente a la izquierda
        elif izq == 1 and cen == 0 and der == 0: error = -2 # Muy a la izquierda
        elif izq == 1 and cen == 1 and der == 1: error = 0  # Cruce o parada
        elif izq == 0 and cen == 0 and der == 0:
            # Línea perdida: recuperamos el último lado por donde vimos la línea
            error = 3 if self.ultimo_error_conocido > 0 else -3
        else:
            error = 0
            
        if abs(error) <= 2:
            self.ultimo_error_conocido = error
            
        return error

    def frenos_contorno(self, error):
        """Rutina de recuperación para curvas de 90 grados"""        
        LY = 0 
        LX = 0 

        fuerza_giro = 900
        if error > 0:
            # Se salió por la izquierda, debe rotar a la derecha para recuperarla
            RX = fuerza_giro if self.invertir_correccion else -fuerza_giro
        else:
            # Se salió por la derecha, rotar a la izquierda
            RX = -fuerza_giro if self.invertir_correccion else fuerza_giro
            
        return LY, LX, RX

    def calcular_pid(self, error):
        """Algoritmo PID clásico"""
        proporcional = error
        
        self.integral += proporcional
        if self.integral > self.max_integral: self.integral = self.max_integral
        elif self.integral < -self.max_integral: self.integral = -self.max_integral
        
        derivativo = proporcional - self.proporcional_pasado
        
        salida_pid = (self.Kp * proporcional) + (self.Ki * self.integral) + (self.Kd * derivativo)
        
        self.proporcional_pasado = proporcional
        return salida_pid

    def actualizar_cinematica(self, izq, cen, der):
        """Bucle principal a llamar constantemente"""
        error = self.leer_sensores_y_error(izq, cen, der)
        
        if error != 0 and abs(error) != 3: 
            if error == self.error_anterior:
                # Si lleva 250ms atorado en el mismo estado, inyectamos potencia
                if time.time() - self.tiempo_ultimo_error > 0.25:
                    self.impulso_extra += 50  # Rampa de aceleración
                    if self.impulso_extra > 800: # Límite máximo de seguridad
                        self.impulso_extra = 800
            else:
                # Se movió exitosamente. Reseteamos reloj e impulso.
                self.tiempo_ultimo_error = time.time()
                self.error_anterior = error
                self.impulso_extra = 0
        else:
            self.tiempo_ultimo_error = time.time()
            self.error_anterior = error
            self.impulso_extra = 0
        
        if abs(error) == 3:
            LY = 0
            LX = 0
            
            # Reducimos la fuerza a 650-700 para evitar sobrecompensar y 
            # saltarnos la línea provocando el giro de 180 grados.
            fuerza_giro = 650 
            
            if error > 0:
                RX = fuerza_giro if self.invertir_correccion else -fuerza_giro
            else:
                RX = -fuerza_giro if self.invertir_correccion else fuerza_giro
        else:
            salida_pid = self.calcular_pid(error)
            
            LY = self.velocidad_base
            if abs(error) == 2: 
                LY = int(self.velocidad_base * 0.8)
                
            LX_base = -int(salida_pid) if self.invertir_correccion else int(salida_pid)
            
            # Aplicar el empuje extra Anti-Atasco en la dirección correcta
            if LX_base > 0:
                LX = LX_base + self.impulso_extra
            elif LX_base < 0:
                LX = LX_base - self.impulso_extra
            else:
                LX = 0
                
            RX = 0 
            
        # Matriz Mecanum
        FR = LY - LX + RX
        FL = LY + LX - RX
        BL = LY - LX - RX
        BR = LY + LX + RX
        
        return FR, FL, BL, BR

def main():
    motor = Ordinary_Car()
    infrared = Infrared()
    pid_controller = SeguidorMecanumPID()
    num_muestras = 5

    print("Iniciando rutina de control...")
    
    try:
        while True:
            # Leer sensores 1=Izq, 2=Cen, 3=Der
            # Muestreo de sensores
            sum_izq = 0
            sum_cen = 0
            sum_der = 0

            for _ in range(num_muestras):
                sum_izq += infrared.read_one_infrared(1) #[cite: 1]
                sum_cen += infrared.read_one_infrared(2) #[cite: 1]
                sum_der += infrared.read_one_infrared(3) #[cite: 1]
                # Una pausa minúscula permite que el sensor capte cambios físicos reales
                time.sleep(0.002)
            

            val_izq = 1 if (sum_izq / num_muestras) >= 0.5 else 0
            val_cen = 1 if (sum_cen / num_muestras) >= 0.5 else 0
            val_der = 1 if (sum_der / num_muestras) >= 0.5 else 0
            
            FR, FL, BL, BR = pid_controller.actualizar_cinematica(val_izq, val_cen, val_der)
            print(f"{FR},{FL},{BR},{BR}")            
            motor.set_motor_model(FL, BL, BL, BR)
            
            # frecuencia constante para que el PID . 0.01 = 100 Hz.
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\nDeteniendo robot...")
    finally:
        motor.set_motor_model(0, 0, 0, 0)
        motor.close()
        infrared.close()

if __name__ == "__main__":
    main()