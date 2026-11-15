"""
Módulo de Comunicación Serial Bidireccional de Alto Rendimiento (serial_controller.py)
Implementa un QThread secundario para lectura no bloqueante, cola segura de comandos
y protección atómica contra bucles de error por desconexión física de USB.
"""

import math
import time
import queue
from PyQt6 import QtCore
import serial
import serial.tools.list_ports


SIMULATOR_PORT = "[SIMULADOR] ESP32 Virtual"


class SerialController(QtCore.QThread):
    """
    Controlador de comunicación serie asíncrono y bidireccional.
    Gestiona la recepción continua de puntos y el envío no bloqueante de comandos de calibración.
    """
    punto_recibido = QtCore.pyqtSignal(float, float, float, float, float, float, float)
    telemetria_calibracion = QtCore.pyqtSignal(float, float, float, float)
    posicion_escaneo = QtCore.pyqtSignal(float, float, float)
    evento_escaneo = QtCore.pyqtSignal(str)
    latencia_actualizada = QtCore.pyqtSignal(float)
    comando_enviado = QtCore.pyqtSignal(str)
    error_conexion = QtCore.pyqtSignal(str)
    estado_conexion = QtCore.pyqtSignal(bool, str)

    def __init__(self, port: str, baudrate: int, config_manager=None, initial_pan: float = 90.0, initial_tilt: float = 90.0):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.config_manager = config_manager

        # Offsets y ajustes
        self.pan_offset = 0.0
        self.tilt_offset = 0.0
        self.dist_offset = 0.0
        self.invert_pan = False
        self.invert_tilt = False

        # Límites mecánicos seguros
        self.pan_min = 0.0
        self.pan_max = 150.0
        self.tilt_min = 0.0
        self.tilt_max = 180.0

        # Altura física del sensor sobre la superficie de apoyo (mesa/suelo Z=0)
        self.sensor_height_mm = 65.0

        # Resolución de escaneo
        self.scan_resolution = 1.0

        if self.config_manager:
            self.cargar_configuracion()

        # Control de hilo y cola de comandos
        self.running = True
        self.scanning_active = False
        self.scanning_paused = False
        self.cmd_queue = queue.Queue()
        self._last_cmd_timestamp = None

        # Bandera atómica para evitar cascadas y bucles infinitos de diálogos de error
        self._notificando_error = False

        # Estado del simulador (si aplica)
        self.is_simulator = (self.port == SIMULATOR_PORT)
        self._sim_pan = float(initial_pan)
        self._sim_tilt = float(initial_tilt)
        self._sim_scan_dir = 1

    def cargar_configuracion(self):
        """Carga parámetros iniciales desde el ConfigManager."""
        if not self.config_manager:
            return
        cfg = self.config_manager
        self.pan_offset = float(cfg.get("offsets.pan_offset", 0.0))
        self.tilt_offset = float(cfg.get("offsets.tilt_offset", 0.0))
        self.dist_offset = float(cfg.get("offsets.dist_offset_mm", 0.0))
        self.invert_pan = bool(cfg.get("servos.invert_pan", False))
        self.invert_tilt = bool(cfg.get("servos.invert_tilt", False))
        self.pan_min = float(cfg.get("servos.pan_min", 0.0))
        self.pan_max = float(cfg.get("servos.pan_max", 150.0))
        self.tilt_min = float(cfg.get("servos.tilt_min", 0.0))
        self.tilt_max = float(cfg.get("servos.tilt_max", 180.0))
        self.sensor_height_mm = float(cfg.get("geometry.sensor_height_mm", 65.0))
        self.scan_resolution = float(cfg.get("scan.resolution", 1.0))

    def set_sensor_height(self, height_mm: float):
        self.sensor_height_mm = float(height_mm)

    def set_offsets(self, pan_offset: float, tilt_offset: float, dist_offset: float):
        self.pan_offset = pan_offset
        self.tilt_offset = tilt_offset
        self.dist_offset = dist_offset

    def set_inversion(self, invert_pan: bool, invert_tilt: bool):
        self.invert_pan = invert_pan
        self.invert_tilt = invert_tilt

    def set_limits(self, pan_min: float, pan_max: float, tilt_min: float, tilt_max: float):
        self.pan_min = float(pan_min)
        self.pan_max = float(pan_max)
        self.tilt_min = float(tilt_min)
        self.tilt_max = float(tilt_max)

    def set_scan_resolution(self, res: float):
        self.scan_resolution = max(0.1, float(res))

    def enviar_comando(self, cmd: str):
        """Encola un comando para ser transmitido al microcontrolador de forma segura."""
        cmd_clean = cmd.strip()
        if cmd_clean and self.running:
            self.cmd_queue.put(cmd_clean)

    def run(self):
        """Bucle principal de ejecución del hilo."""
        if self.is_simulator:
            self._run_simulator()
        else:
            self._run_hardware()

    def _run_hardware(self):
        """Manejo de conexión física por puerto serial con desconexión atómica segura."""
        ser = None
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=0.04)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            self.estado_conexion.emit(True, f"Conectado a {self.port} @ {self.baudrate} bps")
        except Exception as e:
            if not self._notificando_error:
                self._notificando_error = True
                self.error_conexion.emit(f"Error al abrir {self.port}: {str(e)}")
                self.estado_conexion.emit(False, f"Fallo al abrir {self.port}")
            return

        while self.running:
            # 1. Enviar comandos pendientes en la cola
            while not self.cmd_queue.empty() and self.running:
                try:
                    cmd = self.cmd_queue.get_nowait()
                    line_to_send = (cmd + "\n").encode('utf-8')
                    ser.write(line_to_send)
                    ser.flush()
                    self._last_cmd_timestamp = time.perf_counter()
                    self.comando_enviado.emit(cmd)
                except (serial.SerialException, OSError, Exception) as e:
                    if not self._notificando_error:
                        self._notificando_error = True
                        self.running = False
                        self.error_conexion.emit(f"Pérdida de conexión al transmitir: {str(e)}")
                    break

            if not self.running:
                break

            # 2. Leer datos recibidos desde el ESP32
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    if self._last_cmd_timestamp is not None:
                        latencia = (time.perf_counter() - self._last_cmd_timestamp) * 1000.0
                        self.latencia_actualizada.emit(round(latencia, 1))
                        self._last_cmd_timestamp = None

                    self._procesar_linea(line)
            except (serial.SerialException, OSError, Exception) as e:
                if not self._notificando_error:
                    self._notificando_error = True
                    self.running = False
                    self.error_conexion.emit(f"Pérdida física de conexión con {self.port}: {str(e)}")
                break

            QtCore.QThread.msleep(2)

        if ser:
            try:
                if ser.is_open:
                    ser.close()
            except Exception:
                pass

        self.estado_conexion.emit(False, "Desconectado")

    def _run_simulator(self):
        """Simulador de ESP32 para pruebas de integración sin hardware físico."""
        self.estado_conexion.emit(True, "Conectado al Simulador ESP32 Virtual")
        tick = 0

        while self.running:
            while not self.cmd_queue.empty():
                cmd = self.cmd_queue.get_nowait()
                t_recv = time.perf_counter()
                if self._last_cmd_timestamp:
                    lat = (t_recv - self._last_cmd_timestamp) * 1000.0 + 8.5
                    self.latencia_actualizada.emit(round(lat, 1))
                else:
                    self.latencia_actualizada.emit(9.2)
                self._last_cmd_timestamp = None

                self.comando_enviado.emit(cmd)

                if cmd.startswith("CAL:P,"):
                    try:
                        self._sim_pan = float(cmd.split(",")[1])
                    except ValueError:
                        pass
                elif cmd.startswith("CAL:T,"):
                    try:
                        self._sim_tilt = float(cmd.split(",")[1])
                    except ValueError:
                        pass
                elif cmd.startswith("CMD:RES,"):
                    try:
                        self.scan_resolution = float(cmd.split(",")[1])
                    except ValueError:
                        pass
                elif cmd == "CMD:CENTER":
                    self._sim_pan = 90.0
                    self._sim_tilt = 90.0
                elif cmd.startswith("CMD:START_SCAN"):
                    partes = cmd.split(",")
                    if len(partes) >= 6:
                        try:
                            self.pan_min = float(partes[1])
                            self.pan_max = float(partes[2])
                            self.tilt_min = float(partes[3])
                            self.tilt_max = float(partes[4])
                            self.scan_resolution = float(partes[5])
                        except ValueError:
                            pass
                    # Clampear posición actual estrictamente a los límites seguros recién recibidos
                    self._sim_pan = max(self.pan_min, min(self.pan_max, self._sim_pan))
                    self._sim_tilt = max(self.tilt_min, min(self.tilt_max, self._sim_tilt))
                    self._sim_scan_dir = 1
                    self.scanning_active = True
                    self.scanning_paused = False
                    self._procesar_linea("OK:SCAN_STARTED")
                elif cmd == "CMD:PAUSE_SCAN":
                    if self.scanning_active:
                        self.scanning_paused = True
                        self._procesar_linea("OK:SCAN_PAUSED")
                elif cmd == "CMD:RESUME_SCAN":
                    if self.scanning_active and self.scanning_paused:
                        self.scanning_paused = False
                        self._procesar_linea("OK:SCAN_RESUMED")
                elif cmd == "CMD:STOP_SCAN":
                    self.scanning_active = False
                    self.scanning_paused = False
                    self._sim_pan = 90.0
                    self._sim_tilt = 90.0
                    self._procesar_linea("OK:SCAN_STOPPED")

                # Telemetría inmediata al procesar comando
                dist_sim = self._calcular_distancia_sintetica(self._sim_pan, self._sim_tilt)
                line_sim = f"{self._sim_pan:.1f},{self._sim_tilt:.1f},{dist_sim:.1f}"
                self._procesar_linea(line_sim)

            if self.scanning_active and not self.scanning_paused:
                step = self.scan_resolution
                self._sim_pan += step * self._sim_scan_dir
                if self._sim_pan >= self.pan_max:
                    self._sim_pan = self.pan_max
                    self._sim_scan_dir = -1
                    self._sim_tilt += step
                elif self._sim_pan <= self.pan_min:
                    self._sim_pan = self.pan_min
                    self._sim_scan_dir = 1
                    self._sim_tilt += step

                if self._sim_tilt > self.tilt_max:
                    self.scanning_active = False
                    self._sim_pan = 90.0
                    self._sim_tilt = 90.0
                    self._procesar_linea("OK:SCAN_FINISHED")
                else:
                    dist_sim = self._calcular_distancia_sintetica(self._sim_pan, self._sim_tilt)
                    line_sim = f"{self._sim_pan:.1f},{self._sim_tilt:.1f},{dist_sim:.1f}"
                    self._procesar_linea(line_sim)
                QtCore.QThread.msleep(15)
            else:
                tick += 1
                if tick % 5 == 0:
                    dist_sim = self._calcular_distancia_sintetica(self._sim_pan, self._sim_tilt)
                    line_sim = f"{self._sim_pan:.1f},{self._sim_tilt:.1f},{dist_sim:.1f}"
                    self._procesar_linea(line_sim)
                QtCore.QThread.msleep(20)

        self.estado_conexion.emit(False, "Simulador Desconectado")

    def _calcular_distancia_sintetica(self, pan, tilt):
        rad_p = math.radians(pan)
        rad_elev = math.radians(tilt - 90.0)
        base_dist = 450.0 + 120.0 * math.sin(rad_p * 2.0) + 80.0 * math.cos(rad_elev * 3.0)
        ruido = ((hash(f"{pan}_{tilt}") % 100) / 100.0 - 0.5) * 6.0
        return max(40.0, min(1400.0, base_dist + ruido))

    def _procesar_linea(self, line: str):
        """Parsea una trama recibida PAN,TILT,DIST_MM, aplica offsets y realiza cinemática con horizonte en 90°."""
        line_clean = line.strip()
        if not line_clean:
            return

        # Notificar eventos y respuestas de control del microcontrolador
        if line_clean.startswith("OK:") or line_clean.startswith("CMD:") or line_clean.startswith("ERR:"):
            self.evento_escaneo.emit(line_clean)
            return

        partes = line_clean.replace(';', ',').split(',')
        if len(partes) >= 3:
            try:
                pan_raw = float(partes[0].strip())
                tilt_raw = float(partes[1].strip())
                dist_raw = float(partes[2].strip())

                # Notificar avance angular para barra de progreso continua y fluida
                self.posicion_escaneo.emit(pan_raw, tilt_raw, dist_raw)

                # Compensación con offsets
                pan_eff = pan_raw + self.pan_offset
                tilt_eff = tilt_raw + self.tilt_offset
                dist_eff = max(0.0, dist_raw + self.dist_offset) if dist_raw > 0 else dist_raw

                # Inversión de signos configurable
                if self.invert_pan:
                    pan_eff = 180.0 - pan_eff
                if self.invert_tilt:
                    tilt_eff = 180.0 - tilt_eff

                self.telemetria_calibracion.emit(pan_eff, tilt_eff, dist_raw, dist_eff)

                # Solo procesar punto 3D si la lectura física es válida (30 a 2000 mm)
                if 30.0 <= dist_raw <= 2000.0:
                    elev_grados = tilt_eff - 90.0
                    elev_rad = math.radians(elev_grados)
                    pan_rad = math.radians(pan_eff)

                    # Coordenadas cartesianas reales en mm con elevación física sobre la superficie (Z=0)
                    x = dist_eff * math.cos(elev_rad) * math.cos(pan_rad)
                    y = dist_eff * math.cos(elev_rad) * math.sin(pan_rad)
                    z = (dist_eff * math.sin(elev_rad)) + self.sensor_height_mm

                    self.punto_recibido.emit(x, y, z, pan_eff, tilt_eff, dist_raw, dist_eff)
            except ValueError:
                pass

    def stop(self):
        """Detiene el hilo de comunicación de forma ordenada y garantizada."""
        self.running = False
        if self.isRunning():
            self.wait(800)
            if self.isRunning():
                self.terminate()
                self.wait(200)
