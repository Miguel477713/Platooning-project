import time
from motor import Ordinary_Car

def test_mecanum():
    # Inicializamos los motores
    motor = Ordinary_Car()
    
    # Velocidad lenta y segura para la prueba
    speed = 700

    try:
        print("--- PRUEBA DE CALIBRACIÓN MECANUM ---")
        print("Iniciando en 3 segundos. Coloca el coche en el suelo...\n")
        time.sleep(3)

        # 1. Movimiento Frontal
        print("1. Avanzando Hacia ADELANTE")
        print("   Visual: Las 4 ruedas deben girar hacia adelante.")
        motor.set_motor_model(-speed, -speed, -speed, -speed)
        time.sleep(2)
        motor.set_motor_model(0, 0, 0, 0) # Freno
        time.sleep(1)

        # 2. Movimiento Trasero
        print("2. Yendo hacia ATRÁS")
        print("   Visual: Las 4 ruedas deben girar hacia atrás.")
        motor.set_motor_model(speed, speed, speed, speed)
        time.sleep(2)
        motor.set_motor_model(0, 0, 0, 0)
        time.sleep(1)

        # 3. Desplazamiento Lateral Izquierdo
        print("3. Desplazamiento lateral a la IZQUIERDA (Strafe Left)")
        print("   Visual: El coche debe deslizarse de lado sin girar el chasis.")
        print("   - Ruedas Frontal-Der y Trasera-Izq -> Adelante")
        print("   - Ruedas Frontal-Izq y Trasera-Der -> Atrás")
        motor.set_motor_model(speed, -speed, -speed, speed)
        time.sleep(2.5)
        motor.set_motor_model(0, 0, 0, 0)
        time.sleep(1)

        # 4. Desplazamiento Lateral Derecho
        print("4. Desplazamiento lateral a la DERECHA (Strafe Right)")
        print("   Visual: El coche debe deslizarse de lado hacia la derecha.")
        print("   - Ruedas Frontal-Izq y Trasera-Der -> Adelante")
        print("   - Ruedas Frontal-Der y Trasera-Izq -> Atrás")
        motor.set_motor_model(-speed, speed, speed, -speed)
        time.sleep(2.5)
        motor.set_motor_model(0, 0, 0, 0)
        time.sleep(1)

        # 5. Rotación sobre su propio eje (Izquierda)
        print("5. Rotando sobre su eje a la IZQUIERDA")
        print("   Visual: Las ruedas izquierdas van hacia atrás, las derechas hacia adelante.")
        motor.set_motor_model(speed, speed, -speed, -speed)
        time.sleep(2)
        motor.set_motor_model(0, 0, 0, 0)
        time.sleep(1)

        # 6. Rotación sobre su propio eje (Derecha)
        print("6. Rotando sobre su eje a la DERECHA")
        print("   Visual: Las ruedas izquierdas van hacia adelante, las derechas hacia atrás.")
        motor.set_motor_model(-speed, -speed, speed, speed)
        time.sleep(2)
        
        print("\n--- PRUEBA FINALIZADA ---")
        print("Si todos los movimientos fueron correctos, tu hardware está listo.")

    except KeyboardInterrupt:
        print("\nPrueba cancelada manualmente por el usuario.")
    
    finally:
        # Freno de seguridad final
        motor.set_motor_model(0, 0, 0, 0)
        motor.close()

if __name__ == '__main__':
    test_mecanum()