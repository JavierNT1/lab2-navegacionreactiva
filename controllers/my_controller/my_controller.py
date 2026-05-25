from controller import Robot

# =========================
# INICIALIZACION ROBOT
# =========================

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# =========================
# PARAMETROS ROBOT
# =========================

WHEEL_RADIUS = 0.02

# =========================
# PARAMETROS DEL FILTRO SIMPLE
# =========================

alpha = 0.3

# =========================
# PARAMETROS DEL FILTRO KALMAN
# =========================

Q = 1.0          # ruido del modelo
R = 25.0         # ruido de medicion
P = 100.0        # incertidumbre inicial

# =========================
# MODOS DE OPERACION
# =========================

USE_RAW = False
USE_FILTER = False
USE_KALMAN = True

# =========================
# FRECUENCIA DE MUESTREO
# =========================

Ts = timestep / 1000.0
fs = 1 / Ts

print(f"Tiempo de muestreo Ts = {Ts:.3f} s")
print(f"Frecuencia de muestreo fs = {fs:.2f} Hz")

# =========================
# MOTORES
# =========================

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# =========================
# ENCODERS
# =========================

left_encoder = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')

left_encoder.enable(timestep)
right_encoder.enable(timestep)

# =========================
# SENSORES DE DISTANCIA
# =========================

ps = []

for i in range(8):
    sensor = robot.getDevice(f'ps{i}')
    sensor.enable(timestep)
    ps.append(sensor)

# =========================
# ALMACENAMIENTO DE SEÑALES
# =========================

tiempo_datos = []

sensor_izq_crudo_datos = []
sensor_der_crudo_datos = []

sensor_izq_filtrado_datos = []
sensor_der_filtrado_datos = []

sensor_fusion_datos = []
kalman_datos = []

encoder_izq_datos = []
encoder_der_datos = []

delta_s_datos = []
distancia_datos = []

K_datos = []

# =========================
# CONTADOR DE MUESTRAS
# =========================

k = 0

encoder_izq = 0.0
encoder_der = 0.0

# =========================
# VARIABLES DE ODOMETRIA
# =========================

encoder_izq_prev = None
encoder_der_prev = None

delta_theta_izq = 0.0
delta_theta_der = 0.0

delta_s = 0.0
distancia_total = 0.0

# =========================
# VARIABLES DE CONTROL
# =========================

direccion_giro = 0
contador_giro = 0

# =========================
# VARIABLES DEL FILTRO SIMPLE
# =========================

sensor_filtrado_izq = 0.0
sensor_filtrado_der = 0.0

# =========================
# VARIABLES DEL FILTRO KALMAN
# =========================

d_est = None
d_pred = 0.0
K = 0.0

# =========================
# LOOP PRINCIPAL
# =========================

while robot.step(timestep) != -1:
    if k > 1000:
        break
    tiempo_actual = k * Ts

    # =========================
    # LEER SENSORES
    # =========================

    valores = []

    for sensor in ps:
        valores.append(sensor.getValue())
    
    encoder_izq = left_encoder.getValue()
    encoder_der = right_encoder.getValue()
    
    if encoder_izq_prev is None:
    
        encoder_izq_prev = encoder_izq
        encoder_der_prev = encoder_der
        k += 1
        continue

    # Cambio angular
    
    delta_theta_izq = encoder_izq - encoder_izq_prev
    delta_theta_der = encoder_der - encoder_der_prev
    
    # Distancia recorrida por cada rueda
    
    delta_s_izq = WHEEL_RADIUS * delta_theta_izq
    delta_s_der = WHEEL_RADIUS * delta_theta_der

    # Avance promedio del robot
    delta_s = (delta_s_izq + delta_s_der) / 2.0
    
    # Distancia acumulada
    
    distancia_total += delta_s    
    encoder_izq_prev = encoder_izq
    encoder_der_prev = encoder_der
    
    # =========================
    # SENSORES CRUDOS
    # =========================
    
    # Lado izquierdo
    izq_frontal_crudo = valores[7]   # ps7
    izq_diagonal_crudo = valores[6]  # ps6
    
    # Lado derecho
    der_frontal_crudo = valores[0]   # ps0
    der_diagonal_crudo = valores[1]  # ps1
    
    # Combinar sensores
    frontal_izq_crudo = izq_frontal_crudo + 0.5 * izq_diagonal_crudo
    frontal_der_crudo = der_frontal_crudo + 0.5 * der_diagonal_crudo
    
    # =========================
    # FILTRO SIMPLE
    # =========================
    
    sensor_filtrado_izq = (
        alpha * frontal_izq_crudo
        + (1 - alpha) * sensor_filtrado_izq
    )
    
    sensor_filtrado_der = (
        alpha * frontal_der_crudo
        + (1 - alpha) * sensor_filtrado_der
    )

    # =========================
    # MEDICION DEL SENSOR
    # =========================
    
    z = sensor_filtrado_izq + sensor_filtrado_der
    
    # Inicializacion Kalman
    
    if d_est is None:
    
        d_est = z
    
    else:
    
        # Prediccion usando encoders
    
        d_pred = d_est - delta_s * 1000
    
        P_pred = P + Q
    
        # Ganancia de Kalman
    
        K = P_pred / (P_pred + R)
    
        # Correccion
    
        d_est = d_pred + K * (z - d_pred)
    
        P = (1 - K) * P_pred
    
    kalman_datos.append(d_est)
    sensor_fusion_datos.append(z)
    K_datos.append(K)
    # =========================
    # SELECCION DE SENAL
    # =========================
    
    if USE_RAW:
    
        frontal_izq = frontal_izq_crudo
        frontal_der = frontal_der_crudo
    
    elif USE_FILTER:
    
        frontal_izq = sensor_filtrado_izq
        frontal_der = sensor_filtrado_der
    
    elif USE_KALMAN:
    # Se mantienen para almacenamiento y comparacion
        frontal_izq = sensor_filtrado_izq
        frontal_der = sensor_filtrado_der
        
    
    tiempo_datos.append(tiempo_actual)
    
    sensor_izq_crudo_datos.append(frontal_izq_crudo)
    sensor_der_crudo_datos.append(frontal_der_crudo)
    
    sensor_izq_filtrado_datos.append(sensor_filtrado_izq)
    sensor_der_filtrado_datos.append(sensor_filtrado_der)
    
    encoder_izq_datos.append(encoder_izq)
    encoder_der_datos.append(encoder_der)
    
    distancia_datos.append(distancia_total)
    delta_s_datos.append(delta_s)
    
    # =========================
    # NAVEGACION REACTIVA
    # =========================
    
    if USE_KALMAN:
    
        frontal_kalman = d_est
    
        if frontal_kalman > 400:
        
            # Sensores laterales
            lateral_izq = valores[5]
            lateral_der = valores[2]
            
            # Elegir direccion SOLO una vez
            if contador_giro == 0:
        
                if lateral_izq > lateral_der:
        
                    direccion_giro = 1      # derecha
        
                else:
        
                    direccion_giro = -1     # izquierda
        
                contador_giro = 20
        
            # Mantener la direccion elegida
            if direccion_giro == 1:
        
                left_motor.setVelocity(3.0)
                right_motor.setVelocity(-3.0)
        
            else:
        
                left_motor.setVelocity(-3.0)
                right_motor.setVelocity(3.0)
        
            contador_giro -= 1
        
        else:
        
            left_motor.setVelocity(3.0)
            right_motor.setVelocity(3.0)
        
            contador_giro = 0
    
    else:
    
        frontal = frontal_izq + frontal_der
    
        if frontal > 400:
    
            if frontal_izq > frontal_der:
    
                left_motor.setVelocity(3.0)
                right_motor.setVelocity(-3.0)
    
            else:
    
                left_motor.setVelocity(-3.0)
                right_motor.setVelocity(3.0)
    
        else:
    
            left_motor.setVelocity(3.0)
            right_motor.setVelocity(3.0)
        
    k += 1
    
    if k % 100 == 0:
        
        print(
            f"Muestras:{len(tiempo_datos)} "
            f"Tiempo:{tiempo_datos[-1]:.2f}s "
            f"Dist:{distancia_total:.3f}m "
            f"K:{K:.3f} "
            f"z:{z:.1f} "
            f"est:{d_est:.1f}"
        )
"""
import pandas as pd

df = pd.DataFrame({
    "tiempo": tiempo_datos,
    "crudo": sensor_izq_crudo_datos,
    "filtrado": sensor_izq_filtrado_datos,
    "fusion": sensor_fusion_datos,
    "kalman": kalman_datos,
    "distancia": distancia_datos
})

df.to_csv("datos_robot.csv", index=False)
"""