import numpy as np
import matplotlib.pyplot as plt

class QuinticPolinomial():
    def __init__(self):
        pass

    def compute_polynomial(self,initial_pos,initial_vel,initial_acc,
                           final_pos,final_vel,final_acc,
                           initial_time, end_time):
        """
        Calcula un polinomio de grado 5 para interpolar el mov. de una articulacion.

        Parametros:
            * initial_pos: posicion inicial
            * initial_vel: velocidad inicial
            * initial_acc: aceleracion inicial
            * final_pos: posicion final
            * final_vel: velocidad final
            * final_acc: aceleracion final
            * initial_time: tiempo de inicio de la trayectoria
            * end_time: tiempo de finalizacion de la trayectoria

        Retorna:
            pos_trajectory, vel_trajectory, acc_trajectory
        """
        t = np.linspace(initial_time, end_time, int(100 * (end_time - initial_time)))
        c = np.ones_like(t)

        M = np.array([
            [1, initial_time, initial_time**2, initial_time**3, initial_time**4, initial_time**5],
            [0, 1, 2*initial_time, 3*initial_time**2, 4*initial_time**3, 5*initial_time**4],
            [0, 0, 2, 6*initial_time, 12*initial_time**2, 20*initial_time**3],
            [1, end_time, end_time**2, end_time**3, end_time**4, end_time**5],
            [0, 1, 2*end_time, 3*end_time**2, 4*end_time**3, 5*end_time**4],
            [0, 0, 2, 6*end_time, 12*end_time**2, 20*end_time**3]
        ])

        b = np.array([initial_pos, initial_vel, initial_acc,
                      final_pos, final_vel, final_acc])

        # Solve for 'a' using numpy's linear algebra solve function (more efficient and stable than inverse)
        a = np.linalg.solve(M, b)

        qd = a[0]*c + a[1]*t + a[2]*t**2 + a[3]*t**3 + a[4]*t**4 + a[5]*t**5
        vd = a[1]*c + 2*a[2]*t + 3*a[3]*t**2 + 4*a[4]*t**3 + 5*a[5]*t**4
        ad = 2*a[2]*c + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3

        return t, qd, vd, ad
    

    def interpolation(self, times: np.ndarray, positions: np.ndarray):
        """
        Interpola el spline Quintic a partir con continuidad C2
        entre multiples puntos

        Argumentos:
            * times: vector de tiempos (n elementos)
            * positions: vector de posiciones (n elementos)

        Retorna:
            Matriz de coeficientes (n-1 x 6) por segmento
        """

        n = len(positions)
        if n != len(times):
            raise ValueError("Los vectores de tiempo y posición deben tener el mismo tamaño")
        if n < 2:
            raise ValueError("Se requieren al menos 2 puntos para interpolación")
        
        num_segments = n - 1
        total_equations = 6 * num_segments

        A = np.zeros((total_equations, total_equations))
        b = np.zeros(total_equations)
        
        # Construir sistema de ecuaciones
        for i in range(n-1):
            t0 = times[i]
            tf = times[i+1]
            dt = tf - t0
            
                        # Condiciones de frontera del segmento
            A = np.array([
                [1, t0, t0**2, t0**3, t0**4, t0**5],    # Posición inicial
                [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4], # Velocidad inicial
                [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],     # Aceleración inicial
                [1, tf, tf**2, tf**3, tf**4, tf**5],     # Posición final
                [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4], # Velocidad final
                [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]      # Aceleración final
            ])

            b = np.array([
                positions[i],  # Posición inicial
                0,             # Velocidad inicial = 0
                0,             # Aceleración inicial = 0
                positions[i+1],# Posición final
                0,             # Velocidad final = 0
                0              # Aceleración final = 0
            ])

        # Resolver sistema lineal
        try:
            coeffs = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            raise RuntimeError("Sistema singular: Verifique los datos de entrada")

        return coeffs


if __name__ == "__main__":

    t0 = 0.0
    tf = 1.0
    q0 = 0.0
    v0 = 0.0
    ac0 = 0.0
    q1 = 2.0
    v1 = 4.0
    ac1 = 0.0

    quintic = QuinticPolinomial()
    
    t_traj, qd_traj, vd_traj, ad_traj = quintic.compute_polynomial(q0,v0,ac0,q1,v1,ac1,t0,tf)

    coef=quintic.interpolation([0,1], [0,2])
    print(coef)

    # --- Graficar las trayectorias utilizando matplotlib ---
    fig, ax = plt.subplots(3, 1, figsize=(8, 10)) # Crear una figura con 3 subplots en vertical

    # Grafica de Posición
    ax[0].plot(t_traj, qd_traj, label='Posición (qd)', color='blue')
    ax[0].set_xlabel('Tiempo (t)')
    ax[0].set_ylabel('Posición')
    ax[0].set_title('Trayectoria de Posición')
    ax[0].grid(True)
    ax[0].legend()

    # Grafica de Velocidad
    ax[1].plot(t_traj, vd_traj, label='Velocidad (vd)', color='red')
    ax[1].set_xlabel('Tiempo (t)')
    ax[1].set_ylabel('Velocidad')
    ax[1].set_title('Trayectoria de Velocidad')
    ax[1].grid(True)
    ax[1].legend()

    # Grafica de Aceleración
    ax[2].plot(t_traj, ad_traj, label='Aceleración (ad)', color='green')
    ax[2].set_xlabel('Tiempo (t)')
    ax[2].set_ylabel('Aceleración')
    ax[2].set_title('Trayectoria de Aceleración')
    ax[2].grid(True)
    ax[2].legend()

    plt.tight_layout() # Ajustar el diseño para evitar superposición de etiquetas
    plt.show() # Mostrar la figura
        