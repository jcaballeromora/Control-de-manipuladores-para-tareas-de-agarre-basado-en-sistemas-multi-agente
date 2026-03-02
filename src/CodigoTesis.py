import urx
import math
import time
import sys
import os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd  

if sys.version_info[0] < 3:
    input = raw_input

def clear_console():
    """Limpia la consola de la terminal."""
    os_name = os.name
    if os_name == 'nt':  # Para Windows
        os.system('cls')
        
# Definimos un método de espera para activar movimiento
def wait():
    if do_wait:
        print("Click enter para continuar")
        input()

def r2d(ang_grad):
    """Convierte radianes a grados."""
    return ang_grad * 180 / math.pi

"""
# Variables de estado del controlador (se inicializan una vez)
integral = 0.0
e_prev = 0.0
Ts=0.008

def control_F(F_med):
    global integral, e_prev
    
    # Parámetros del controlador
    Kp,Ki= 0.012,0.02
    #Kp = 0.01  # Ganancia proporcional
    #Ki = 0.07  # Ganancia integral
    Kd = 0.0    # Ganancia derivativa
    Ts = 0.008  # Intervalo de muestreo (segundos)
    
    # Referencia o setpoint
    reference = 20.0
    
    # ------------------ Lógica del controlador PID ------------------
    
    # Calcular el error
    error = reference - F_med    
    # Actualizar la parte integral
    integral += error * Ts
    
    # Calcular la derivada
    derivative = (error - e_prev) / Ts
    
    # Calcular la salida del controlador
    u = Kp * error + Ki * integral + Kd * derivative
    
    # Actualizar el error para el próximo ciclo
    e_prev = error
    
    # Calcular y devolver la variable de control zr
    zr = u * 0.008
    return zr

def Filtro(Fz):
    Fz_filtro=0
    fs=1000
    fc=10
    dt=Ts
    RC=1/(2*3.1416*fc)
    alfa=dt/(dt+RC)
    Fz_filtro=alfa*Fz + (1-alfa)*Fz_filtro
    return Fz_filtro
"""

# --- Variables de estado del controlador (Separadas por robot) ---
integral_r1 = 0.0
e_prev_r1 = 0.0

integral_r2 = 0.0
e_prev_r2 = 0.0

Ts = 0.008

def control_F(F_med, robot_id):
    # Traemos las variables globales de ambos robots
    global integral_r1, e_prev_r1, integral_r2, e_prev_r2
    
    # Parámetros del controlador
    Kp, Ki = 0.012, 0.02
    Kd = 0.0
    
    # Referencia o setpoint
    reference = 20.0
    
    # Calcular el error
    error = reference - F_med
    
    # --- Selección de Memoria según el Robot ---
    if robot_id == 1:
        # Usar memoria del Robot 1
        integral_r1 += error * Ts
        derivative = (error - e_prev_r1) / Ts
        
        # Calcular salida PID
        u = Kp * error + Ki * integral_r1 + Kd * derivative
        
        # Guardar error actual
        e_prev_r1 = error
        
    else:
        # Usar memoria del Robot 2
        integral_r2 += error * Ts
        derivative = (error - e_prev_r2) / Ts
        
        # Calcular salida PID
        u = Kp * error + Ki * integral_r2 + Kd * derivative
        
        # Guardar error actual
        e_prev_r2 = error
    
    # Calcular y devolver la variable de control zr
    zr = u * 0.008
    return zr

fz_pasada_r1 = 0.0
fz_pasada_r2 = 0.0

def Filtro(Fz_actual, robot_id):
    global fz_pasada_r1, fz_pasada_r2
    
    # Parámetros 
    fc = 5  #5Hz 
    dt = 0.008 
    RC = 1 / (2 * 3.1416 * fc)
    alfa = dt / (dt + RC)
    
    # Seleccionamos la memoria según el robot
    if robot_id == 1:
        fz_filtrada = alfa * Fz_actual + (1 - alfa) * fz_pasada_r1
        fz_pasada_r1 = fz_filtrada # Actualizamos memoria
    else:
        fz_filtrada = alfa * Fz_actual + (1 - alfa) * fz_pasada_r2
        fz_pasada_r2 = fz_filtrada # Actualizamos memoria
        
    return fz_filtrada
def cin_inv_r1(px, py, pz):
    # Parámetros constantes del robot (DH-Parameters)
    d1 = 0.15185
    d2, d3 = 0, 0
    d4 = 0.13105
    d5 = 0.08535
    d6 = 0.0921
    a0, a1 = 0, 0
    a2 = -0.24355
    a3 = -0.2132
    a4, a5, a5 = 0, 0, 0

    # Orientación de la herramienta (Matriz de rotación fija)
    nx, ox, ax = 0, 0, 1
    ny, oy, ay = -1, 0, 0
    nz, oz, az = 0, -1, 0
    
    # ------------------ Cálculos de cinemática inversa ------------------
    
    # Cálculo de q1
    q1 = math.atan2(py - ay * d6, px - ax * d6) - math.atan2(-d4, math.sqrt(math.pow(px - ax * d6, 2) + math.pow(py - ay * d6, 2) - math.pow(-d4, 2)))
    
    # Cálculos para q5 y q6
    S5 = math.sqrt(math.pow(-math.sin(q1) * nx + math.cos(q1) * ny, 2) + math.pow((-math.sin(q1) * ox + math.cos(q1) * oy), 2))
    S6 = (-math.sin(q1) * ox + math.cos(q1) * oy) / S5
    C6 = (math.sin(q1) * nx - math.cos(q1) * ny) / S5
    q5 = math.atan2(S5, math.sin(q1) * ax - math.cos(q1) * ay)
    q6 = math.atan2(S6, C6)

    # Cálculos para q2, q3 y q4
    q234 = math.atan2((-az / S5), (-(math.cos(q1) * ax + math.sin(q1) * ay) / S5))
    B1 = (math.cos(q1) * px + math.sin(q1) * py - d5 * math.sin(q234) + d6 * math.cos(q234) * S5)
    B2 = (pz - d1 + d5 * math.cos(q234) + d6 * math.sin(q234) * S5)
    A = -2 * B2 * a2
    B = 2 * B1 * a2
    C = math.pow(B1, 2) + math.pow(B2, 2) + math.pow(a2, 2) - math.pow(a3, 2)

    q2 = math.atan2(B, A) - math.atan2(C, math.sqrt(math.pow(A, 2) + math.pow(B, 2) - math.pow(C, 2)))
    q23 = math.atan2((B2 - a2 * math.sin(q2)) / a3, (B1 - a2 * math.cos(q2)) / a3)
    q3 = q23 - q2 - 2 * math.pi
    q4 = q234 - q23 #+ 2 * math.pi

    # Conversión a grados antes de retornar
    q1_grados = r2d(q1)
    q2_grados = r2d(q2)
    q3_grados = r2d(q3)
    q4_grados = r2d(q4)
    q5_grados = r2d(q5)
    q6_grados = r2d(q6)
    return (q1,q2, q3, q4, q5, 0)

def cin_inv_r2(px, py, pz):
    # Parámetros constantes del robot (DH-Parameters)
    d1 = 0.15185
    d2, d3 = 0, 0
    d4 = 0.13105
    d5 = 0.08535
    d6 = 0.0921
    a0, a1 = 0, 0
    a2 = -0.24355
    a3 = -0.2132
    a4, a5, a5 = 0, 0, 0

    # Orientación de la herramienta (Matriz de rotación fija)
    nx, ox, ax = 0, 0, -1
    ny, oy, ay = 1, 0, 0
    nz, oz, az = 0, -1, 0
    
    # ------------------ Cálculos de cinemática inversa ------------------
    
    # Cálculo de q1
    q1 = math.atan2(py - ay * d6, px - ax * d6) - math.atan2(-d4, math.sqrt(math.pow(px - ax * d6, 2) + math.pow(py - ay * d6, 2) - math.pow(-d4, 2)))
    
    # Cálculos para q5 y q6
    S5 = math.sqrt(math.pow(-math.sin(q1) * nx + math.cos(q1) * ny, 2) + math.pow((-math.sin(q1) * ox + math.cos(q1) * oy), 2))
    S6 = (-math.sin(q1) * ox + math.cos(q1) * oy) / S5
    C6 = (math.sin(q1) * nx - math.cos(q1) * ny) / S5
    q5 = math.atan2(S5, math.sin(q1) * ax - math.cos(q1) * ay)
    q6 = math.atan2(S6, C6)

    # Cálculos para q2, q3 y q4
    q234 = math.atan2((-az / S5), (-(math.cos(q1) * ax + math.sin(q1) * ay) / S5))
    B1 = (math.cos(q1) * px + math.sin(q1) * py - d5 * math.sin(q234) + d6 * math.cos(q234) * S5)
    B2 = (pz - d1 + d5 * math.cos(q234) + d6 * math.sin(q234) * S5)
    A = -2 * B2 * a2
    B = 2 * B1 * a2
    C = math.pow(B1, 2) + math.pow(B2, 2) + math.pow(a2, 2) - math.pow(a3, 2)

    q2 = math.atan2(B, A) - math.atan2(C, math.sqrt(math.pow(A, 2) + math.pow(B, 2) - math.pow(C, 2)))
    q23 = math.atan2((B2 - a2 * math.sin(q2)) / a3, (B1 - a2 * math.cos(q2)) / a3)
    q3 = q23 - q2 - 2 * math.pi
    q4 = q234 - q23 #+ 2 * math.pi

    # Conversión a grados antes de retornar
    q1_grados = r2d(q1)
    q2_grados = r2d(q2)
    q3_grados = r2d(q3)
    q4_grados = r2d(q4)
    q5_grados = r2d(q5)
    q6_grados = r2d(q6)
    return (q1,q2, q3, q4, q5, 0) #solucion en radianes

def mars_agarre(r, t):
    objx, objy, objz = r[0], r[1], r[2]
    r3x, r3y, r3z = r[3], r[4], r[5]
    r5x, r5y, r5z = r[6], r[7], r[8]

    k, kp, kd = 1.0, 1.0, 0.5
    r1xd, r1yd, r1zd = 0.0, -0.05, 0.381
    dr1xd, dr1yd, dr1zd = 0.0, 0.0, 0.0
    
    if t < 20:
        C31x, C51x = 0.414, -0.414
    elif 20 <= t <= 30:
        C31x, C51x = 0.42, -0.42 #test
    elif 30 <= t <= 60:
        C31x, C51x = 0.42, -0.42 #test
    elif 60 <= t <= 90:
        C31x, C51x = 0.42, -0.42 #test
        r1zd = 0.43
    elif 90 <= t <= 120:
        C31x, C51x = 0.42, -0.42 #test
        r1yd, r1zd = -0.15, 0.43
    elif 120 <= t <= 150:
        C31x, C51x = 0.42, -0.42 #test
        r1yd, r1zd = -0.15, 0.381
    else:
        C31x, C51x = 0.35, -0.35 #test
        r1yd, r1zd = -0.15, 0.381

    dz1x = kp * (r1xd - objx) + kd * (dr1xd - 0)
    dz1y = kp * (r1yd - objy) + kd * (dr1yd - 0)
    dz1z = kp * (r1zd - objz) + kd * (dr1zd - 0)

    dz3x = -k * (r3x - objx - C31x)
    dz3y = -k * (r3y - objy - 0)
    dz3z = -k * (r3z - objz - 0)

    dz5x = -k * (r5x - objx - C51x)
    dz5y = -k * (r5y - objy - 0)
    dz5z = -k * (r5z - objz - 0)

    return np.array([dz1x, dz1y, dz1z, dz3x, dz3y, dz3z, dz5x, dz5y, dz5z])


# Programa principal
if __name__ == '__main__':
    do_wait = True
    if len(sys.argv) > 1:
        do_wait = False
    
    rob = None
    rob2 = None 
    
    # ------------------ Configuración de recolección de datos ------------------
    datos_excel = [] # <--- Lista para guardar cada fila del experimento
    """
    plt.ion()  
    fig, ax = plt.subplots()
    ax.set_title("Control de Fuerza")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Fuerza en X (N)")
    ax.grid(True)
    
    line1, = ax.plot([], [], 'b-', label='Fuerza Fx_Robot 1') 
    line2, = ax.plot([], [], 'g-', label='Fuerza Fx_Robot 2') 
    line_ref, = ax.plot([], [], 'r--', label='Referencia')
    
    ax.legend()
    
    tiempo_data = []
    fuerza_data1 = [] 
    fuerza_data2 = [] 

    ax.set_xlim(0, 10)
    ax.set_ylim(-10, 30)
    """
    r = np.array([0, -0.05, 0.381, 0.35, -0.05, 0.381, -0.35, -0.05, 0.381])
    tf = 180.0
    t=0
    try:
        clear_console()
        print("Conectandose a robot 1...")
        rob = urx.Robot("192.168.3.71")
        rob.set_tcp((0, 0, 0, 0, 0, 0))
        rob.set_payload(0.1, (0,0,0))
        rob.get_realtime_monitor()
        print("**Robot 1 conectado**")

        print("Conectandose a robot 2...")
        rob2 = urx.Robot("192.168.3.76")
        rob2.set_tcp((0, 0, 0.0, 0, 0, 0))
        rob2.set_payload(0.1, (0,0,0))
        rob2.get_realtime_monitor()
        print("**Robot 2 conectado**")
        
        time.sleep(1) 
        
        h1 = [math.radians(-3),math.radians(-122),math.radians(-74),math.radians(16),math.radians(93),math.radians(0)]
        h2 = [math.radians(-138),math.radians(-119),math.radians(-79),math.radians(18),math.radians(48),math.radians(0)]
        a=1.4/2
        v=3.14/2
        time.sleep(2)
        
        print("Moviéndose")
        rob.movej(h1,a,v,wait=False) 
        rob2.movej(h2,a,v,wait=False)
        print("Esperando 15 seg para experimento...\n")
        time.sleep(15) 
        
        start_time = time.time()
        last_time = start_time

        print("Iniciando simulación en tiempo real...")
    
        while (time.time() - start_time) < tf:
            a,v =0.8,0.15
            #a=1.4
            #v=3.14
            current_real_time = time.time() - start_time
            dt = (time.time() - start_time) - (last_time - start_time)
            last_time = time.time()
            t = current_real_time
            
            force1 = rob.get_tcp_force()
            force2 = rob2.get_tcp_force()
            
            Fx1 = - force1[0]
            Fx_Fil1 = Filtro(Fx1,1)

            Fx2 =  force2[0]
            Fx_Fil2 = Filtro(Fx2,2)

            xr1=control_F(Fx_Fil1,1)
            xr2=control_F(Fx_Fil2,2)

            dr = mars_agarre(r, current_real_time)
            r = r + dr * dt
            px1,py1,pz1=r[3],r[4],r[5]
            px2,py2,pz2=r[6],r[7],r[8]
            
            # --- Guardado de datos en la lista ---
            datos_excel.append({
                'Fx_Fil1': Fx_Fil1, 'Fx_Fil2': Fx_Fil2,
                'xr1': xr1, 'xr2': xr2,
                'px1': px1, 'py1': py1, 'pz1': pz1,
                'px2': px2, 'py2': py2, 'pz2': pz2,
                'dt': dt, 'current_real_time': current_real_time
            })

            
            if 20 <= t <= 150:
                # Aplicamos la corrección del PID
                target_px1 = px1 + xr1
                target_px2 = px2 - xr2
                target_py2 = py2 + 0.032
            else:
                # Trayectoria pura sin fuerza
                target_px1 = px1
                target_px2 = px2
                target_py2 = py2 + 0.032

            qd1_r1 = cin_inv_r1(target_px1, py1, pz1)
            qd1_r2 = cin_inv_r2(target_px2, target_py2, pz2)
            rob.movej(qd1_r1, a, v, wait=False)
            rob2.movej(qd1_r2, a, v, wait=False)

            elapsed = time.time() - last_time
            if elapsed < Ts:
                time.sleep(Ts - elapsed)
            """
            tiempo_data.append(t)
            fuerza_data1.append(Fx_Fil1) 
            fuerza_data2.append(Fx_Fil2) 

            line1.set_data(tiempo_data, fuerza_data1)
            line2.set_data(tiempo_data, fuerza_data2) 
            line_ref.set_data(tiempo_data, [20.0] * len(tiempo_data))

            ax.set_xlim(0, tiempo_data[-1] + 1)
            max_fuerza = max(max(fuerza_data1), max(fuerza_data2))
            min_fuerza = min(min(fuerza_data1), min(fuerza_data2))
            ax.set_ylim(min_fuerza - 5, max_fuerza + 5)

            plt.pause(Ts)
            """
            #time.sleep(Ts)

    except KeyboardInterrupt:
        print("\nPrograma detenido por el usuario.")
    except Exception as e:
        print(f"\nOcurrió un error: {e}")
    finally:
        # --- Lógica de exportación a Excel ---
        if datos_excel:
            print("\nGuardando datos en Excel...")
            df = pd.DataFrame(datos_excel)
            df.to_excel("13datos_experimento.xlsx", index=False)
            print("Datos guardados exitosamente en 'datos_experimento.xlsx'.")
        
        #plt.savefig("grafica_fuerza.png")
        #print("Gráfica guardada como 'grafica_fuerza.png'.")
        #plt.ioff()
        #plt.show()
        
        if rob:
            rob.close()
        if rob2: 
            rob2.close()
        print("\nConexión con robots cerrada.")
        sys.exit()
