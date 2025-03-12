import matplotlib.pyplot as plt
import numpy as np

class TrajectoryPlotter:
    def __init__(self):
        self.joint_trajectories = {}  # Diccionario para almacenar trayectorias por articulación

    def add_joint_trajectory(self, joint_id, segments):
        """
        Agrega una trayectoria de una articulación compuesta por múltiples segmentos.
        
        Args:
            joint_id (int): Identificador de la articulación
            segments (list[SegmentProfile]): Lista de perfiles de segmentos de la trayectoria
        """
        # Verificar si ya existe una trayectoria para esta articulación
        if joint_id in self.joint_trajectories:
            # Si existe, extender con los nuevos segmentos
            self.joint_trajectories[joint_id].extend(segments)
        else:
            # Si no existe, crear una nueva entrada
            self.joint_trajectories[joint_id] = segments

    def plot(self, joint_id, sample_rate=0.01):
        """
        Grafica la trayectoria completa de una articulación.
        
        Args:
            joint_id (int): Identificador de la articulación
            sample_rate (float): Tasa de muestreo para generar los perfiles
        """
        if joint_id not in self.joint_trajectories:
            print(f"No se encontraron trayectorias para la articulación {joint_id}")
            return

        segments = self.joint_trajectories[joint_id]
        
        # Verificar si hay segmentos
        if not segments:
            print(f"No hay segmentos para la articulación {joint_id}")
            return

        # Calcular el tiempo acumulado y concatenar los perfiles
        total_time = 0
        time_vector = []
        position_profile = []
        velocity_profile = []
        acceleration_profile = []

        for segment in segments:
            # Muestrear el segmento
            segment.sample_trajectory(sample_rate)

            # Calcular el tiempo inicial del segmento
            start_time = total_time
            
            # Calcular el tiempo final del segmento
            end_time = start_time + segment.total_time
            print(f"Total Time: {segment.total_time}")
            # Generar el vector de tiempo para este segmento
            segment_time = np.linspace(start_time, end_time, len(segment._time_vector))
            
            # Concatenar los perfiles
            time_vector.extend(segment_time)
            position_profile.extend(segment.position_profile)
            velocity_profile.extend(segment.velocity_profile)
            acceleration_profile.extend(segment.acceleration_profile)
            
            # Actualizar el tiempo total
            total_time = end_time

        # Crear la figura y los subplots
        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        
        # Graficar posición
        axs[0].plot(time_vector, position_profile)
        axs[0].set_ylabel('Posición (rad)')
        axs[0].grid(True)
        
        # Graficar velocidad
        axs[1].plot(time_vector, velocity_profile)
        axs[1].set_ylabel('Velocidad (rad/s)')
        axs[1].grid(True)
        
        # Graficar aceleración
        axs[2].plot(time_vector, acceleration_profile)
        axs[2].set_ylabel('Aceleración (rad/s²)')
        axs[2].set_xlabel('Tiempo (s)')
        axs[2].grid(True)
        
        # Ajustar el espaciado
        plt.tight_layout()
        
        # Mostrar los límites de velocidad, aceleración y jerk si están disponibles
        if hasattr(segments[0], 'limit_velocity'):
            axs[1].axhline(segments[0].limit_velocity, color='r', linestyle='--', label='Velocidad máxima')
            axs[1].axhline(-segments[0].limit_velocity, color='r', linestyle='--')
            
        if hasattr(segments[0], 'max_acceleration'):
            axs[2].axhline(segments[0].max_acceleration, color='g', linestyle='--', label='Aceleración máxima')
            axs[2].axhline(-segments[0].max_acceleration, color='g', linestyle='--')
        
        # Mostrar leyendas
        axs[1].legend()
        axs[2].legend()
        
        # Título de la figura
        fig.suptitle(f'Trayectoria de la Articulación {joint_id}')
        
        # Guardar la figura para mostrarla posteriormente
        self.current_figure = fig

    def show(self):
        """Muestra todas las figuras generadas."""
        plt.show() 