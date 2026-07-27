import time
import threading
from motor import Ordinary_Car
from infrared import Infrared

class SeguidorDiferencialPID:
    def __init__(self):
        # --- CONSTANTES PID (Llantas Normales) ---
        # Las llantas normales tienen mucha más fricción al girar que las Mecanum.
        self.Kp = 300.0  
        self.Kd = 150.0  
        self.Ki = 0.0 
        
        # Velocidad muy baja y controlada
        self.velocidad_base = -800 
        self.invertir_correccion = True 
        
        # ========================================================
        # NUEVO: VARIABLES DE SINCRONIZACIÓN (PARO PERIÓDICO)
        # ========================================================
        self.habilitar_sincronizacion = True
        self.tiempo_marcha = 2.0  # Segundos que avanza antes de detenerse
        self.tiempo_espera = 8.0  # Segundos que se detiene a esperar al otro robot
        
        self.en_espera = False
        self.reloj_sincronizacion = time.time()
        # ========================================================
        
        # Variables de memoria para el PID
        self.proporcional_pasado = 0
        self.integral = 0
        self.max_integral = 1000 
        self.ultimo_error_conocido = 0
        
        # Sistema Anti-Atasco (Esencial a bajas velocidades con llantas normales)
        self.tiempo_ultimo_error = time.time()
        self.error_anterior = 0
        self.impulso_extra = 0
        
    def leer_sensores_y_error(self, izq, cen, der):
        """Traduce los 3 sensores a un error numérico"""
        if izq == 0 and cen == 1 and der == 0: error = 0   
        elif izq == 0 and cen == 1 and der == 1: error = 1 
        elif izq == 0 and cen == 0 and der == 1: error = 2 
        elif izq == 1 and cen == 1 and der == 0: error = -1 
        elif izq == 1 and cen == 0 and der == 0: error = -2 
        elif izq == 1 and cen == 1 and der == 1: error = 0  
        elif izq == 0 and cen == 0 and der == 0:
            error = 3 if self.ultimo_error_conocido > 0 else -3
        else:
            error = 0
            
        if abs(error) <= 2:
            self.ultimo_error_conocido = error
            
        return error

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
        """Calcula la velocidad de cada rueda usando Control Diferencial"""
        error = self.leer_sensores_y_error(izq, cen, der)
        tiempo_actual = time.time()
        
        # ========================================================
        # LÓGICA DE SINCRONIZACIÓN (MÁQUINA DE ESTADOS)
        # ========================================================
        if self.habilitar_sincronizacion:
            if not self.en_espera:
                # Si ya pasó el tiempo de marcha y está centrado en una recta (error 0)
                if (tiempo_actual - self.reloj_sincronizacion) > self.tiempo_marcha and error == 0:
                    self.en_espera = True
                    self.reloj_sincronizacion = tiempo_actual
                    print("--- PARO DE SINCRONIZACIÓN: ESPERANDO AL SEGUNDO ROBOT ---")
                    return 0, 0, 0, 0 # Detiene los motores instantáneamente
            else:
                # Si está detenido esperando...
                if (tiempo_actual - self.reloj_sincronizacion) > self.tiempo_espera:
                    self.en_espera = False
                    self.reloj_sincronizacion = tiempo_actual
                    print("--- REANUDANDO MARCHA ---")
                    
                    # Resetear variables Anti-Atasco para un arranque limpio
                    self.tiempo_ultimo_error = tiempo_actual
                    self.error_anterior = error
                    self.impulso_extra = 0
                else:
                    return 0, 0, 0, 0 # Continúa detenido
        # ========================================================

        # 1. SISTEMA ANTI-ATASCO
        if error != 0 and abs(error) != 3: 
            if error == self.error_anterior:
                if tiempo_actual - self.tiempo_ultimo_error > 0.25:
                    self.impulso_extra += 50  
                    if self.impulso_extra > 800: 
                        self.impulso_extra = 800
            else:
                self.tiempo_ultimo_error = tiempo_actual
                self.error_anterior = error
                self.impulso_extra = 0
        else:
            self.tiempo_ultimo_error = tiempo_actual
            self.error_anterior = error
            self.impulso_extra = 0
        
        # 2. CONTROL DE MOVIMIENTO
        if abs(error) == 3:
            # Estado de Emergencia: Línea perdida (Giro de tanque sobre su eje)
            LY = 0
            fuerza_giro = 800 
            
            if error > 0:
                # Perdió la línea por la derecha, rotar fuertemente a la derecha
                RX = -fuerza_giro if self.invertir_correccion else fuerza_giro
            else:
                # Perdió la línea por la izquierda, rotar fuertemente a la izquierda
                RX = fuerza_giro if self.invertir_correccion else -fuerza_giro
        else:
            # Estado Normal: Seguir línea usando el PID
            salida_pid = self.calcular_pid(error)
            
            LY = self.velocidad_base
            if abs(error) == 2: 
                # Frena drásticamente el avance (50% menos) para facilitar giros
                LY = int(self.velocidad_base * 0.5) 
                
            # El PID ahora controla RX (Rotación)
            RX_base = -int(salida_pid) if self.invertir_correccion else int(salida_pid)
            
            if RX_base > 0:
                RX = RX_base + self.impulso_extra
            elif RX_base < 0:
                RX = RX_base - self.impulso_extra
            else:
                RX = 0 
            
        # 3. MATRIZ DIFERENCIAL (Skid-Steer)
        FL = LY + RX
        BL = LY + RX
        FR = LY - RX
        BR = LY - RX
        
        return FR, FL, BL, BR


def main():
    motor = Ordinary_Car()
    infrared = Infrared()
    pid_controller = SeguidorDiferencialPID()
    num_muestras = 5

    print("Iniciando control de precisión Diferencial con PARO PERIÓDICO...")
    
    try:
        while True:
            # Muestreo de sensores para eliminar ruido
            sum_izq = 0
            sum_cen = 0
            sum_der = 0

            for _ in range(num_muestras):
                sum_izq += infrared.read_one_infrared(1) 
                sum_cen += infrared.read_one_infrared(2) 
                sum_der += infrared.read_one_infrared(3) 
                time.sleep(0.002)

            val_izq = 1 if (sum_izq / num_muestras) >= 0.5 else 0
            val_cen = 1 if (sum_cen / num_muestras) >= 0.5 else 0
            val_der = 1 if (sum_der / num_muestras) >= 0.5 else 0
            
            # Obtener velocidades matemáticas
            FR, FL, BL, BR = pid_controller.actualizar_cinematica(val_izq, val_cen, val_der)
            
            # Imprimir para debug de forma clara
            print(f"FR:{FR} | FL:{FL} | BL:{BL} | BR:{BR}")            
            
            # ENVIAR A MOTORES (Orden corregido: Front-Left, Back-Left, Front-Right, Back-Right)
            motor.set_motor_model(FL, BL, FR, BR)
            
            # Frecuencia de ciclo PID constante
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\nDeteniendo robot...")
    finally:
        motor.set_motor_model(0, 0, 0, 0)
        motor.close()
        infrared.close()

if __name__ == "__main__":
    main()