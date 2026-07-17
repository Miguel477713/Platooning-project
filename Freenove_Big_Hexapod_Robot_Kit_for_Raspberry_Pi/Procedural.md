# FollowObjectProcedural.py: Referencia Procedural

Este documento describe el comportamiento real implementado en
`Code/Client/FollowObjectProcedural.py`. La lógica sigue siendo procedural:
recibe video, detecta un objeto por color, centra la cámara, consulta el
ultrasonido y envía comandos directos al servidor Freenove.

No introduce agentes BDI. El control se resuelve con estado simple, funciones
pequeñas y reglas `if/elif/else`.

---

## 1. Archivo Implementado

```text
Code/Client/FollowObjectProcedural.py
```

El script se ejecuta desde `Code/Client/` y usa:

| Necesidad | Implementación |
| :--- | :--- |
| Conexión de comandos | `Client.client_socket1` hacia puerto TCP `5002` |
| Video | `Client.receiving_video()` hacia puerto `8002` |
| Detección visual | OpenCV, conversión BGR a HSV y contornos |
| Movimiento de cámara | `CMD_CAMERA#<x>#<y>` |
| Movimiento de patas | `CMD_MOVE#1#0#<y>#<speed>#<angle>` |
| Posición / altura corporal | `CMD_POSITION#<x>#<y>#<z>` |
| Ultrasonido | `CMD_SONIC` |
| Postura inicial | `CMD_SERVOPOWER#1` y `CMD_POSITION#0#0#<body_z>` |

---

## 2. Constantes Actuales

Estos son los valores definidos en el script:

```python
CENTER_DEADZONE_X = 0.10
CENTER_DEADZONE_Y = 0.10

TARGET_AREA_MIN = 0.05
TARGET_AREA_MAX = 0.20

OBSTACLE_MIN_CM = 25.0
LOOP_DELAY = 0.05
SONIC_INTERVAL = 0.20
FRAME_TIMEOUT = 1.0
CAMERA_COMMAND_INTERVAL = 0.12

PAN_STEP_GAIN = 6
TILT_STEP_GAIN = 3
MAX_PAN_STEP = 4
MAX_TILT_STEP = 1

MOVE_SPEED = 6
SPEED_MIN = 2
SPEED_MAX = 10
MOVE_FORWARD_Y = 12
MOVE_BACKWARD_Y = -10

PAN_MIN = -35
PAN_MAX = 35
TILT_MIN = -35
TILT_MAX = 35

CAMERA_CENTER_X = 90
CAMERA_CENTER_Y = 75
TURN_DEADZONE_PAN = 10
TURN_GAIN = 0.15
ROTATE_TARGET_X = 0.20
BODY_Z = 0
OCCLUDED_TARGET_AREA_MAX = 0.025
OCCLUSION_DARK_RATIO = 0.30
OCCLUSION_MARGIN = 0.35
```

Algunos valores se pueden cambiar por argumentos de línea de comandos:
`--speed`, `--target-area`, `--pan-max`, `--tilt-max`,
`--camera-center-y`, `--ignore-lower-frame`, `--body-z`,
`--occluded-target-area-max` y `--occlusion-dark-ratio`.

---

## 3. Colores Detectados

El script detecta objetos por rangos HSV. `--color` acepta:

| Color | Rango HSV |
| :--- | :--- |
| `red` | `(0,100,80)-(10,255,255)` y `(170,100,80)-(180,255,255)` |
| `green` | `(35,70,60)-(85,255,255)` |
| `blue` | `(90,70,60)-(130,255,255)` |
| `yellow` | `(20,80,80)-(35,255,255)` |

La detección ignora contornos con área relativa menor a `0.002`. Además, por
defecto ignora el `25%` inferior del video para reducir detecciones de patas.

Después de encontrar el contorno de color, el script revisa si la detección es
parcial u ocluida. Esto evita que una pequeña franja verde visible detrás de una
pata sea usada como si el objeto completo estuviera cerca.

La detección se marca como parcial cuando:

1. El área del objetivo es pequeña (`relative_area <= --occluded-target-area-max`).
2. Hay suficiente zona oscura alrededor del rectángulo del objetivo
   (`--occlusion-dark-ratio`).
3. Los píxeles del propio objetivo se excluyen de esa medición para no castigar
   al color detectado.

Cuando aparece `PARTIAL/OCCLUDED TARGET: no distance move` en la ventana, el
robot no usa esa detección para avanzar o retroceder por distancia.

---

## 4. Estado Interno

`ProceduralObjectFollower` mantiene este estado principal:

```python
pan_angle = 0
tilt_angle = 0
target_detected = False
target_x = 0.0
target_y = 0.0
target_area = 0.0
target_occluded = False
action_status = "idle"
last_seen_time = 0.0
search_direction = 1
last_sonic_request = 0.0
obstacle_distance = None
running = False
last_camera_command = 0.0
last_camera_x = None
last_camera_y = None
```

`target_x` y `target_y` son errores normalizados respecto al centro del frame:
valores negativos indican izquierda/arriba y positivos derecha/abajo.

---

## 5. Flujo Principal

El método `run()` hace esta secuencia:

1. Activa el receptor de respuestas TCP.
2. Si `wake_robot` está activo, envía `CMD_SERVOPOWER#1` y `CMD_POSITION#0#0#<body_z>`.
3. Reinicia la cámara a `pan_angle = 0`, `tilt_angle = 0`.
4. Envía stop inicial con `CMD_MOVE#1#0#0#<speed>#0`.
5. En cada ciclo toma el frame más reciente.
6. Si no hay frame durante más de `FRAME_TIMEOUT`, envía stop.
7. Si detecta objetivo, actualiza estado, revisa oclusión, centra cámara y controla movimiento.
8. Si no detecta objetivo, barre la cámara y mantiene patas detenidas.
9. Si `show_video` está activo, muestra ventana OpenCV con estado.
10. Al salir, detiene el robot y reinicia la cámara.

---

## 6. Centrado de Cámara

La cámara usa un centro físico base:

```python
servo_x = CAMERA_CENTER_X + pan_angle
servo_y = camera_center_y + tilt_angle
```

Por defecto:

```text
CAMERA_CENTER_X = 90
camera_center_y = 75
```

El comando enviado es:

```text
CMD_CAMERA#<servo_x>#<servo_y>
```

La corrección horizontal se aplica si `abs(target_x) > 0.10`. La corrección
vertical se aplica si `abs(target_y) > 0.10`, salvo que se use `--lock-tilt`.

Los pasos están limitados por ciclo:

```text
pan:  máximo 4 grados por ciclo
tilt: máximo 1 grado por ciclo
```

El rango de cámara se limita con `--pan-max` y `--tilt-max`. Internamente esos
argumentos se acotan entre `10` y `60`.

---

## 7. Movimiento de Patas

Todos los movimientos salen por:

```text
CMD_MOVE#1#0#<y>#<speed>#<angle>
```

`y` se limita entre `-35` y `35`. `angle` se limita entre `-10` y `10`.

Reglas reales de `control_movement()`:

1. Si `--camera-only` está activo, envía stop.
2. Si `CMD_SONIC` reporta menos de `25 cm`, retrocede con `y = -10`.
3. Si `target_occluded` está activo, no avanza ni retrocede por área; si hay giro calculado, gira en sitio con `y = 0`, si no, envía stop.
4. Si hay giro calculado y el objetivo no está centrado, gira en sitio con `y = 0`.
5. Si el robot debe esperar a que el objetivo esté centrado y todavía no lo está, envía stop.
6. Si `target_area` es menor que `target_area_min`, avanza con `y = 12` y giro calculado.
7. Si `target_area` es mayor que `target_area_max`, retrocede con `y = -10`.
8. Si la distancia es adecuada pero el giro calculado no es cero, gira en sitio con `y = 0`.
9. Si nada de lo anterior aplica, envía stop.

`target_area_min` viene de `--target-area` y por defecto vale `0.05`.
`target_area_max` se calcula como:

```python
target_area_max = target_area_min + 0.15
```

---

## 8. Giro del Chasis

El giro está activo por defecto porque `main()` pasa:

```python
enable_turning = not args.disable_turning
```

`--enable-turning` existe como argumento, pero en la versión actual no cambia el
resultado porque el giro ya está activo salvo que se use `--disable-turning`.

La función real combina pan de cámara y error horizontal del objetivo:

```python
if not enable_turning:
    return 0
if abs(pan_angle) < TURN_DEADZONE_PAN and abs(target_x) < ROTATE_TARGET_X:
    return 0
turn_source = pan_angle if abs(pan_angle) >= TURN_DEADZONE_PAN else -target_x * 20
turn = clamp(-turn_source * TURN_GAIN, -5, 5)
if abs(turn) < 1:
    turn = 1 if turn > 0 else -1
return int(round(turn))
```

Con los valores actuales:

```text
TURN_DEADZONE_PAN = 10
TURN_GAIN = 0.15
ROTATE_TARGET_X = 0.20
rango final = -5 a 5
```

Esto corrige el caso donde la cámara ya está muy girada o el objeto está muy
descentrado, pero el robot se quedaba detenido por la regla de “esperar a que
esté centrado”. Ahora primero puede rotar en sitio con `CMD_MOVE#1#0#0#<speed>#<angle>`.

Para caminar sin giro:

```bash
sudo python3 FollowObjectProcedural.py --ip 10.0.7.200 --color green --lock-tilt --disable-turning
```

No hay lógica de `CMD_ATTITUDE` en este script. La prueba anterior de “twist”
se retiró porque el repositorio no tiene un comando de twist vertical real:
`CMD_ATTITUDE` puede inclinar con roll/pitch y rotar horizontalmente con yaw,
pero no resuelve la oclusión vertical de una pata sobre la cámara.

---

## 9. Posición Z / Altura Corporal

El comando para desplazar la postura del cuerpo, incluyendo altura en `z`, es:

```text
CMD_POSITION#<x>#<y>#<z>
```

Está implementado en el servidor en `Code/Server/control.py`:

```python
x = restrict_value(command[1], -40, 40)
y = restrict_value(command[2], -40, 40)
z = restrict_value(command[3], -20, 20)
move_position(x, y, z)
```

Dentro de `move_position(x, y, z)`, el servidor calcula la altura así:

```python
points[i][2] = -30 - z
body_height = points[i][2]
```

Por eso `z` afecta la altura corporal usada para recalcular las posiciones de
las seis patas. En esta versión de `FollowObjectProcedural.py`, el comando se
usa al iniciar dentro de `stand_up()`:

```text
CMD_POSITION#0#0#<body_z>
```

`--body-z` se acota internamente al rango del servidor (`-20` a `20`) y solo se
envía una vez durante el arranque. El seguidor no ajusta la altura durante el
bucle de seguimiento.

---

## 10. Búsqueda del Objetivo

Cuando no hay objetivo detectado:

1. Si se vio hace menos de `0.5` segundos, solo envía stop.
2. Después barre `pan_angle` en pasos de `3` grados.
3. El barrido rebota entre `-pan_max` y `pan_max`.
4. Siempre envía stop mientras busca.

La búsqueda mueve solo la cámara; no hace caminar al robot.

---

## 11. Argumentos Disponibles

Esta sección refleja todos los argumentos definidos en `parse_args()`.

| Argumento | Obligatorio | Default | Efecto |
| :--- | :---: | :--- | :--- |
| `--ip` | Sí | Sin default | IP de la Raspberry Pi del robot. Se usa para conectar a `5002` y `8002`. |
| `--color` | No | `red` | Color HSV a seguir. Opciones: `red`, `green`, `blue`, `yellow`. |
| `--speed` | No | `6` | Velocidad usada en `CMD_MOVE`. Se acota internamente de `2` a `10`. |
| `--target-area` | No | `0.05` | Área relativa mínima para considerar que el robot ya se acercó. Se acota de `0.01` a `0.90`; el límite superior de retroceso será este valor más `0.15`. |
| `--pan-max` | No | `35` | Límite horizontal absoluto de `pan_angle`. Se acota de `10` a `60`. |
| `--tilt-max` | No | `35` | Límite vertical absoluto de `tilt_angle`. Se acota de `10` a `60`. |
| `--no-video` | No | `False` | Ejecuta sin abrir ventana OpenCV. Internamente llama `run(show_video=False)`. |
| `--dry-run` | No | `False` | No conecta al robot; imprime comandos en terminal en vez de enviarlos por TCP. |
| `--camera-only` | No | `False` | Centra la cámara pero mantiene las patas detenidas con comandos stop. |
| `--invert-pan` | No | `False` | Invierte la dirección de corrección horizontal de la cámara. |
| `--invert-tilt` | No | `False` | Invierte la dirección de corrección vertical de la cámara. |
| `--lock-tilt` | No | `False` | Bloquea la corrección vertical; solo corrige pan horizontal. |
| `--move-before-centered` | No | `False` | Permite caminar aunque el objetivo todavía no esté dentro de la zona muerta de centrado. |
| `--enable-turning` | No | `False` como flag | Se conserva por compatibilidad; el giro ya está activo por defecto. |
| `--disable-turning` | No | `False` | Desactiva el giro del chasis. Hace que `turn_from_pan()` devuelva `0`. |
| `--camera-center-y` | No | `75` | Ángulo base vertical de la cámara. Valores menores suelen mirar más alto. |
| `--ignore-lower-frame` | No | `0.25` | Fracción inferior del frame que se ignora. Se acota de `0.0` a `0.8`. |
| `--body-z` | No | `0` | Altura corporal inicial enviada como `CMD_POSITION#0#0#<z>`. Se acota de `-20` a `20`. Solo se aplica al arranque. |
| `--occluded-target-area-max` | No | `0.025` | Área máxima para aplicar la prueba de objetivo parcial u ocluido. Se acota de `0.001` a `0.20`. |
| `--occlusion-dark-ratio` | No | `0.30` | Fracción oscura alrededor del objetivo que marca la detección como parcialmente tapada. Se acota de `0.05` a `0.90`. |
| `--no-wake` | No | `False` | Evita enviar `CMD_SERVOPOWER#1` y `CMD_POSITION#0#0#<body_z>` al iniciar. |

Si se usa `--no-wake`, `--body-z` no tendrá efecto porque se omite el envío de
`CMD_POSITION`.

Comando de ayuda:

```bash
python3 FollowObjectProcedural.py --help
```

---

## 12. Comandos de Uso

Arrancar servidor en la Raspberry Pi:

```bash
cd ~/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi/Code/Server
sudo python3 main.py -t -n
```

Ejecutar seguidor recomendado para objeto verde:

```bash
cd ~/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi/Code/Client
sudo python3 FollowObjectProcedural.py --ip 10.0.7.200 --color green --lock-tilt
```

Probar solo cámara, sin mover patas:

```bash
sudo python3 FollowObjectProcedural.py --ip 10.0.7.200 --color green --camera-only --lock-tilt
```

Ejecutar sin enviar comandos reales:

```bash
sudo python3 FollowObjectProcedural.py --ip 10.0.7.200 --color green --dry-run
```

Caminar sin giro de chasis:

```bash
sudo python3 FollowObjectProcedural.py --ip 10.0.7.200 --color green --lock-tilt --disable-turning
```

Aumentar velocidad dentro del rango permitido:

```bash
sudo python3 FollowObjectProcedural.py --ip 10.0.7.200 --color green --lock-tilt --speed 8
```

Acercarse más antes de detenerse:

```bash
sudo python3 FollowObjectProcedural.py --ip 10.0.7.200 --color green --lock-tilt --target-area 0.15
```

Ajustar altura base de cámara:

```bash
sudo python3 FollowObjectProcedural.py --ip 10.0.7.200 --color green --lock-tilt --camera-center-y 65
```

Ajustar altura corporal inicial:

```bash
sudo python3 FollowObjectProcedural.py --ip 10.0.7.200 --color green --lock-tilt --body-z 10
```

Ignorar más zona inferior del video:

```bash
sudo python3 FollowObjectProcedural.py --ip 10.0.7.200 --color green --lock-tilt --ignore-lower-frame 0.35
```

Si la pata tapa el objeto y aparece una franja pequeña de color, hacer más
estricta la detección de objetivo parcial:

```bash
sudo python3 FollowObjectProcedural.py --ip 10.0.7.200 --color green --lock-tilt --occlusion-dark-ratio 0.20
```

Si marca demasiados objetivos como parciales aunque no estén tapados, hacerla
menos sensible:

```bash
sudo python3 FollowObjectProcedural.py --ip 10.0.7.200 --color green --lock-tilt --occlusion-dark-ratio 0.45
```

---

## 13. Seguridad

Antes de permitir movimiento autónomo:

1. Probar primero con `--camera-only --lock-tilt`.
2. Mantener el robot en un espacio libre.
3. Empezar con velocidad por defecto `--speed 6` o menor.
4. Usar `--disable-turning` si el giro del chasis causa oscilación.
5. Usar `--no-wake` solo si la postura ya fue preparada manualmente.
6. Detener con `Ctrl+C`; el bloque `finally` envía stop y reinicia la cámara.

La calibración de patas sigue dependiendo del servidor y su `point.txt`; este
script solo decide cuándo enviar `CMD_MOVE`.

---

## 14. Diagnóstico

Si aparece `Connection refused`, revisar que el servidor esté escuchando en
`5002`, que no haya otro cliente conectado y que la IP sea correcta.

En la Raspberry Pi:

```bash
sudo ss -ltnp | grep 5002
hostname -I
```

Si se está ejecutando en la misma Raspberry Pi por VNC, el mensaje de error del
script sugiere probar `--ip 127.0.0.1` solo si el servidor escucha en localhost.

`Client.py` no es la GUI. Para abrir el cliente gráfico normal se usa:

```bash
sudo python3 Main.py
```

Si el cuadro verde aparece como una franja pequeña encima de una pata y el robot
cree que está cerca del objeto, revisar la superposición de texto en la ventana:

```text
PARTIAL/OCCLUDED TARGET: no distance move
```

Si ese mensaje no aparece, bajar `--occlusion-dark-ratio`, por ejemplo `0.20`.
Si aparece cuando el objeto está limpio, subirlo, por ejemplo `0.45`.

---

## 15. Resultado Esperado

Con el servidor activo y un color visible:

1. El cliente se conecta al robot.
2. Recibe video.
3. Detecta el objeto por color.
4. Mueve la cámara para centrarlo.
5. Consulta el ultrasonido periódicamente.
6. Si el objetivo está parcialmente tapado por una pata, no avanza ni retrocede por área; gira en sitio si necesita recentrar.
7. Si el objetivo está libre, avanza, retrocede, gira o se detiene según las reglas anteriores.
8. Si pierde el objetivo, barre la cámara y mantiene las patas detenidas.
