import logging
import os
import sys
import traceback


class LoggerSetup:
    """
    Clase para configurar y gestionar los loggers del proyecto de planificación
    de trayectorias S-Curve con soporte para excepciones personalizadas.
    """
    
    def __init__(self, debug_mode=False, log_dir=None):
        """
        Inicializa el sistema de logging
        
        Args:
            debug_mode: Booleano que indica si se debe usar modo debug (True) o producción (False)
            log_dir: Directorio donde guardar los logs (opcional, por defecto usa directorio actual)
        """
        self.debug_mode = debug_mode
        
        # Determinar directorio de logs
        if log_dir is None:
            # Usar un directorio 'logs' en la raíz del proyecto
            project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            self.log_dir = os.path.join(project_root, 'logs')
        else:
            self.log_dir = log_dir
            
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Crear loggers para diferentes componentes
        self.planning_logger = logging.getLogger("planning_logger")
        self.trajectory_logger = logging.getLogger("trajectory_logger")
        self.constraints_logger = logging.getLogger("constraints_logger")
        self.error_logger = logging.getLogger("error_logger")
        
        # Configurar los loggers
        self._setup_loggers()
    
    def _setup_loggers(self):
        """
        Configura los loggers según el modo (debug o producción)
        """
        if self.debug_mode:
            self._setup_debug_logging()
        else:
            self._setup_production_logging()
    
    def _setup_debug_logging(self):
        """
        Configura logging detallado para desarrollo y depuración
        """
        # Archivos de log
        planning_log = os.path.join(self.log_dir, "planning.log")
        trajectory_log = os.path.join(self.log_dir, "trajectory.log")
        constraints_log = os.path.join(self.log_dir, "constraints.log")
        error_log = os.path.join(self.log_dir, "errors.log")
        
        # 1. Configurar loggers principales (similar a tu implementación original)
        self._configure_component_logger(
            self.planning_logger, planning_log, 
            "%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)s] - %(message)s"
        )
        
        self._configure_component_logger(
            self.trajectory_logger, trajectory_log, 
            "%(message)s", console_output=False
        )
        
        self._configure_component_logger(
            self.constraints_logger, constraints_log, 
            "%(asctime)s - %(levelname)s - %(message)s"
        )
        
        # 2. Configurar logger específico para errores
        self._configure_component_logger(
            self.error_logger, error_log,
            "%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)s] - %(message)s\n%(exc_info)s",
            level=logging.ERROR
        )
    
    def _configure_component_logger(self, logger, log_file, format_str, level=logging.DEBUG, console_output=True):
        """
        Configura un logger específico del componente
        
        Args:
            logger: Objeto logger a configurar
            log_file: Ruta al archivo de log
            format_str: Formato de las entradas de log
            level: Nivel de logging
            console_output: Si debe enviar salida también a la consola
        """
        logger.setLevel(level)
        
        # File handler
        file_handler = logging.FileHandler(log_file, 'w')
        file_handler.setLevel(level)
        file_formatter = logging.Formatter(format_str)
        file_handler.setFormatter(file_formatter)
        logger.addHandler(file_handler)
        
        # Console handler (opcional)
        if console_output:
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(logging.INFO if level == logging.DEBUG else level)
            console_formatter = logging.Formatter(format_str)
            console_handler.setFormatter(console_formatter)
            logger.addHandler(console_handler)
    
    def _setup_production_logging(self):
        """
        Configuración mínima para entorno de producción
        """
        # Solo configurar para errores
        error_log = os.path.join(self.log_dir, "errors.log")
        
        for logger in [self.planning_logger, self.trajectory_logger, 
                      self.constraints_logger, self.error_logger]:
            logger.setLevel(logging.ERROR)
            
            # File handler para errores
            file_handler = logging.FileHandler(error_log, 'a')
            file_handler.setLevel(logging.ERROR)
            file_formatter = logging.Formatter(
                "%(asctime)s - %(name)s - %(levelname)s - [%(filename)s:%(lineno)s] - %(message)s"
            )
            file_handler.setFormatter(file_formatter)
            logger.addHandler(file_handler)
            
            # Console handler para errores críticos
            console_handler = logging.StreamHandler(sys.stderr)
            console_handler.setLevel(logging.CRITICAL)
            console_formatter = logging.Formatter(
                "CRITICAL: %(message)s"
            )
            console_handler.setFormatter(console_formatter)
            logger.addHandler(console_handler)
    
    def get_planning_logger(self):
        """Devuelve el logger para planificación de trayectorias"""
        return self.planning_logger
    
    def get_trajectory_logger(self):
        """Devuelve el logger para datos de trayectorias"""
        return self.trajectory_logger
    
    def get_constraints_logger(self):
        """Devuelve el logger para restricciones cinemáticas"""
        return self.constraints_logger
    
    def get_error_logger(self):
        """Devuelve el logger específico para errores"""
        return self.error_logger
    
    def log_exception(self, exception, logger=None):
        """
        Registra una excepción con detalles completos
        
        Args:
            exception: La excepción a registrar
            logger: Logger específico a usar (por defecto usa error_logger)
        """
        if logger is None:
            logger = self.error_logger
            
        error_details = {
            'message': str(exception),
            'type': exception.__class__.__name__,
            'traceback': traceback.format_exc()
        }
        
        if hasattr(exception, 'error_code') and exception.error_code:
            error_details['code'] = exception.error_code
            
        logger.error(f"Exception occurred: {error_details['message']}", 
                    exc_info=self.debug_mode)
        
        if self.debug_mode:
            logger.debug(f"Error details: {error_details}")