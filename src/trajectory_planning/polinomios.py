import numpy as np
import matplotlib.pyplot as plt

class QuinticPolinomial():
    def __init__(self, max_vel: float, max_acc: float, max_jerk: float):
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.max_jerk = max_jerk

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

if __name__ == "__main__":
    max_vel = 1
    max_acc = 0.8
    max_jerk = 0.4

    t0 = 0.0
    tf = 1.0
    q0 = 0.0
    v0 = 0.0
    ac0 = 0.0
    q1 = 2.0
    v1 = 4.0
    ac1 = 0.0

    quintic = QuinticPolinomial(max_vel=max_vel,
                                max_acc=max_acc,
                                max_jerk=max_jerk)
    
    t_traj, qd_traj, vd_traj, ad_traj = quintic.compute_polynomial(q0,v0,ac0,q1,v1,ac1,t0,tf)

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
        