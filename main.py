"""
Visualizador y Calibrador 3D LiDAR ESP32 - Aplicación de Escritorio
Desarrollado con PyQt6, OpenGL (pyqtgraph) y NumPy.

Soporta:
- Escáner 3D en tiempo real con filtrado SOR y exportación PLY/CSV.
- Pestaña de Calibración Mecánica y Sensores (MG996R en GPIO 18/19 y VL53L0X).
- Jogging seguro validado solo con ENTER (evita tirones mecánicos).
- Selector de resolución angular (0.5°, 1.0°, 2.0°).
- Reconexión segura sin pérdida de posición ni auto-reset.
- Prevención atómica de bucle infinito por desconexión física USB.
- Persistencia atómica de configuración en config.json.
- Modo Simulación con ESP32 virtual para pruebas sin hardware conectado.
"""

import sys
import os
import ctypes
import math
import time
import numpy as np
import serial.tools.list_ports
from PyQt6 import QtWidgets, QtCore, QtGui
import pyqtgraph.opengl as gl

# Protección para ejecución silenciosa con pythonw.exe en Windows (donde sys.stdout/stderr son None)
class _NullStream:
    def write(self, *args, **kwargs):
        pass
    def flush(self, *args, **kwargs):
        pass

if sys.stdout is None:
    try:
        sys.stdout = open(os.devnull, "w", encoding="utf-8")
    except Exception:
        sys.stdout = _NullStream()

if sys.stderr is None:
    try:
        sys.stderr = open(os.devnull, "w", encoding="utf-8")
    except Exception:
        sys.stderr = _NullStream()

# Módulos del proyecto
from config_manager import ConfigManager
from serial_controller import SerialController, SIMULATOR_PORT
from calibration_widget import CalibrationWidget

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ICON_PATH = os.path.join(BASE_DIR, "assets", "lidar_icon.ico")


def statistical_outlier_removal(points, k_neighbors=16, std_ratio=1.5, batch_size=500):
    """
    Filtro estadístico de valores atípicos (Statistical Outlier Removal - SOR)
    implementado puramente con NumPy vectorizado por lotes.
    """
    n_points = len(points)
    if n_points <= k_neighbors:
        return np.ones(n_points, dtype=bool)

    mean_distances = np.empty(n_points, dtype=np.float32)

    for start_idx in range(0, n_points, batch_size):
        end_idx = min(start_idx + batch_size, n_points)
        batch = points[start_idx:end_idx]

        diff = batch[:, np.newaxis, :] - points[np.newaxis, :, :]
        dist_sq = np.sum(diff ** 2, axis=-1)

        k_plus_1 = min(k_neighbors + 1, n_points)
        part_sq = np.partition(dist_sq, kth=k_plus_1 - 1, axis=1)[:, 1:k_plus_1]
        part_dist = np.sqrt(part_sq)
        mean_distances[start_idx:end_idx] = np.mean(part_dist, axis=1)

    mean_dist = np.mean(mean_distances)
    std_dist = np.std(mean_distances)
    threshold = mean_dist + std_ratio * std_dist

    return mean_distances <= threshold


class FPSGLViewWidget(gl.GLViewWidget):
    """Widget de visualización 3D OpenGL con monitoreo de FPS de renderizado."""
    fps_updated = QtCore.pyqtSignal(float)

    def __init__(self, parent=None, **kwargs):
        super().__init__(parent=parent, **kwargs)
        self._last_time = time.perf_counter()
        self._frame_count = 0
        self._current_fps = 0.0

    def paintGL(self, *args, **kwargs):
        super().paintGL(*args, **kwargs)
        self._frame_count += 1
        now = time.perf_counter()
        dt = now - self._last_time
        if dt >= 0.4:
            self._current_fps = self._frame_count / dt
            self._frame_count = 0
            self._last_time = now
            self.fps_updated.emit(self._current_fps)


class LidarScannerGUI(QtWidgets.QMainWindow):
    """Ventana principal del software de escaneo y calibración LiDAR 3D."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Escáner 3D LiDAR ESP32 - Visualizador & Calibrador de Hardware")
        self.resize(1340, 880)
        self.setMinimumSize(1000, 700)

        # Asignación de icono a la ventana principal
        if os.path.exists(ICON_PATH):
            self.setWindowIcon(QtGui.QIcon(ICON_PATH))

        # Cargar configuración persistente
        self.config_manager = ConfigManager()

        # Parámetros del escaneo en curso
        self.scan_p_min = float(self.config_manager.get("servos.pan_min", 0))
        self.scan_p_max = float(self.config_manager.get("servos.pan_max", 150))
        self.scan_t_min = float(self.config_manager.get("servos.tilt_min", 0))
        self.scan_t_max = float(self.config_manager.get("servos.tilt_max", 180))
        self.scan_step = float(self.config_manager.get("scan.resolution", 1.0))

        # Posición angular conservada en memoria (para reconexión segura sin auto-reset)
        self.last_known_pan = 90.0
        self.last_known_tilt = 90.0

        # Altura física del sensor sobre la superficie de apoyo (mesa/suelo Z=0)
        self.sensor_height_mm = float(self.config_manager.get("geometry.sensor_height_mm", 65.0))

        # Bandera atómica para evitar cascada de ventanas modales de error
        self._dialogo_error_activo = False

        # Datos de la nube de puntos
        self.points = []
        self.colors = []
        self.raw_data = []
        self.capturando = True
        self.serial_controller = None

        self._setup_theme()
        self.init_ui()
        self.init_status_bar()

    def _setup_theme(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #17171d; }
            QWidget { color: #f0f0f0; font-family: 'Segoe UI', Arial, sans-serif; }
            QLabel { color: #e0e0e0; font-size: 12px; }
            QTabWidget::pane { border: 1px solid #333344; background: #1f1f28; border-radius: 4px; }
            QTabBar::tab {
                background: #252532;
                color: #b0bec5;
                padding: 8px 16px;
                margin-right: 2px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                font-weight: bold;
                font-size: 12px;
            }
            QTabBar::tab:selected {
                background: #1f1f28;
                color: #00e5ff;
                border-bottom: 2px solid #00e5ff;
            }
            QTabBar::tab:hover:!selected {
                background: #2e2e3e;
                color: #ffffff;
            }
            QComboBox, QLineEdit, QSpinBox, QDoubleSpinBox { 
                background-color: #2b2b38; 
                color: #ffffff; 
                padding: 5px; 
                border: 1px solid #3d3d52; 
                border-radius: 4px; 
                font-size: 12px;
            }
            QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus {
                border: 1px solid #00bcd4;
            }
            QCheckBox { color: #f0f0f0; font-size: 12px; spacing: 8px; }
            QCheckBox::indicator { width: 16px; height: 16px; border: 1px solid #5a5a70; border-radius: 3px; background-color: #2b2b36; }
            QCheckBox::indicator:checked { background-color: #00bcd4; border-color: #00e5ff; }
            QSlider::groove:horizontal {
                border: 1px solid #3a3a4c;
                height: 6px;
                background: #262633;
                border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #00acc1;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #ffffff;
                border: 2px solid #00bcd4;
                width: 14px;
                margin-top: -5px;
                margin-bottom: -5px;
                border-radius: 7px;
            }
            QPushButton { 
                background-color: #0288d1; 
                color: white; 
                font-weight: bold; 
                padding: 7px 12px; 
                border-radius: 5px; 
                border: none;
                font-size: 12px;
            }
            QPushButton:hover { background-color: #039be5; }
            QPushButton:pressed { background-color: #0277bd; }
            QPushButton#btn_stop { background-color: #d32f2f; }
            QPushButton#btn_stop:hover { background-color: #e53935; }
            QPushButton#btn_filter { background-color: #7b1fa2; }
            QPushButton#btn_filter:hover { background-color: #9c27b0; }
            QPushButton#btn_scan_start { background-color: #2e7d32; }
            QPushButton#btn_scan_start:hover { background-color: #388e3c; }
            QGroupBox { 
                color: #00e5ff; 
                font-weight: bold; 
                border: 1px solid #333346; 
                border-radius: 6px; 
                margin-top: 10px; 
                padding-top: 14px; 
                font-size: 12px;
            }
            QStatusBar { 
                background-color: #121217; 
                color: #cfd8dc; 
                border-top: 1px solid #282836; 
                font-size: 12px; 
            }
            QToolTip { background-color: #262635; color: #ffffff; border: 1px solid #00bcd4; padding: 4px; }
            QSplitter::handle { background-color: #292938; }
        """)

    def init_ui(self):
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout(central_widget)
        main_layout.setContentsMargins(6, 6, 6, 4)
        main_layout.setSpacing(6)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        splitter.setHandleWidth(5)

        # ================= PANEL LATERAL =================
        panel_left = QtWidgets.QWidget()
        layout_panel = QtWidgets.QVBoxLayout(panel_left)
        layout_panel.setContentsMargins(2, 2, 2, 2)
        layout_panel.setSpacing(6)

        # 1. BARRA SUPERIOR DE CONEXIÓN UNIFICADA
        gb_conn = QtWidgets.QGroupBox("🔌 Enlace de Comunicación (ESP32)")
        layout_conn = QtWidgets.QGridLayout()
        layout_conn.setContentsMargins(8, 12, 8, 8)
        layout_conn.setHorizontalSpacing(6)
        layout_conn.setVerticalSpacing(6)

        layout_conn.addWidget(QtWidgets.QLabel("Puerto:"), 0, 0)
        self.cb_ports = QtWidgets.QComboBox()
        layout_conn.addWidget(self.cb_ports, 0, 1)

        self.btn_refresh = QtWidgets.QPushButton("🔄")
        self.btn_refresh.setFixedWidth(36)
        self.btn_refresh.setToolTip("Actualizar lista de puertos COM disponibles")
        self.btn_refresh.clicked.connect(self.actualizar_puertos)
        layout_conn.addWidget(self.btn_refresh, 0, 2)

        layout_conn.addWidget(QtWidgets.QLabel("Baudios:"), 1, 0)
        self.cb_baud = QtWidgets.QComboBox()
        self.cb_baud.addItems(["115200", "921600", "57600", "9600"])
        saved_baud = str(self.config_manager.get("connection", "baudrate", "115200"))
        idx = self.cb_baud.findText(saved_baud)
        if idx >= 0:
            self.cb_baud.setCurrentIndex(idx)
        layout_conn.addWidget(self.cb_baud, 1, 1, 1, 2)

        self.btn_conectar = QtWidgets.QPushButton("Conectar Dispositivo")
        self.btn_conectar.clicked.connect(self.toggle_conexion)
        layout_conn.addWidget(self.btn_conectar, 2, 0, 1, 3)

        gb_conn.setLayout(layout_conn)
        layout_panel.addWidget(gb_conn)

        # 2. PESTAÑAS: ESCANEO vs CALIBRACIÓN
        self.tabs = QtWidgets.QTabWidget()

        # PESTAÑA 1: ESCANEO 3D
        tab_escaneo = self._crear_tab_escaneo()
        self.tabs.addTab(tab_escaneo, "🛰️ Escaneo 3D")

        # PESTAÑA 2: CALIBRACIÓN DE HARDWARE
        self.tab_calibracion = CalibrationWidget(self.config_manager)
        self.tab_calibracion.comando_solicitado.connect(self.enviar_comando_serial)
        self.tab_calibracion.offsets_modificados.connect(self.aplicar_nuevos_offsets)
        self.tab_calibracion.limites_modificados.connect(self.aplicar_nuevos_limites)
        self.tab_calibracion.altura_sensor_modificada.connect(self.actualizar_altura_sensor)
        self.tab_calibracion.posicion_jog_cambiada.connect(self.actualizar_rayo_apuntador_jog)
        self.tab_calibracion.inversion_modificada.connect(self.on_inversion_desde_calibracion)
        self.tab_calibracion.mensaje_estado.connect(lambda msg, t: self.statusBar().showMessage(msg, t))
        self.tabs.addTab(self.tab_calibracion, "🔧 Calibración de Hardware")

        layout_panel.addWidget(self.tabs)
        panel_left.setMinimumWidth(390)
        splitter.addWidget(panel_left)

        # ================= VISOR 3D (OPENGL) =================
        container_3d = QtWidgets.QWidget()
        layout_3d = QtWidgets.QVBoxLayout(container_3d)
        layout_3d.setContentsMargins(0, 0, 0, 0)
        layout_3d.setSpacing(4)

        bar_view = QtWidgets.QHBoxLayout()
        bar_view.setContentsMargins(6, 2, 6, 2)

        lbl_title_view = QtWidgets.QLabel("<b>Espacio Tridimensional OpenGL</b>")
        lbl_title_view.setStyleSheet("color: #00e5ff; font-size: 13px;")
        bar_view.addWidget(lbl_title_view)
        bar_view.addStretch()

        btn_cam_iso = QtWidgets.QPushButton("📐 Isométrica")
        btn_cam_iso.setStyleSheet("background-color: #2b2b36; font-size: 11px; padding: 4px 8px;")
        btn_cam_iso.clicked.connect(lambda: self.view3d.setCameraPosition(distance=900, elevation=30, azimuth=45))

        btn_cam_top = QtWidgets.QPushButton("🔝 Planta")
        btn_cam_top.setStyleSheet("background-color: #2b2b36; font-size: 11px; padding: 4px 8px;")
        btn_cam_top.clicked.connect(lambda: self.view3d.setCameraPosition(distance=900, elevation=90, azimuth=0))

        btn_cam_front = QtWidgets.QPushButton("👁️ Frontal")
        btn_cam_front.setStyleSheet("background-color: #2b2b36; font-size: 11px; padding: 4px 8px;")
        btn_cam_front.clicked.connect(lambda: self.view3d.setCameraPosition(distance=900, elevation=0, azimuth=90))

        btn_cam_reset = QtWidgets.QPushButton("🔄 Centrar")
        btn_cam_reset.setStyleSheet("background-color: #2b2b36; font-size: 11px; padding: 4px 8px;")
        btn_cam_reset.clicked.connect(lambda: self.view3d.setCameraPosition(distance=800, elevation=25, azimuth=45))

        bar_view.addWidget(btn_cam_iso)
        bar_view.addWidget(btn_cam_top)
        bar_view.addWidget(btn_cam_front)
        bar_view.addWidget(btn_cam_reset)
        layout_3d.addLayout(bar_view)

        self.view3d = FPSGLViewWidget()
        self.view3d.setCameraPosition(distance=800, elevation=30, azimuth=45)
        self.view3d.fps_updated.connect(self.actualizar_fps)

        grid = gl.GLGridItem()
        grid.setSize(1200, 1200, 10)
        grid.setSpacing(50, 50, 1)
        self.view3d.addItem(grid)

        axis = gl.GLAxisItem()
        axis.setSize(150, 150, 150)
        self.view3d.addItem(axis)

        self.scatter = gl.GLScatterPlotItem()
        self.view3d.addItem(self.scatter)

        # Soporte / Pedestal físico del escáner en OpenGL (desde el suelo Z=0 hasta el sensor Z=+65mm)
        self.pedestal_item = gl.GLLinePlotItem(
            pos=np.array([[0.0, 0.0, 0.0], [0.0, 0.0, self.sensor_height_mm]], dtype=np.float32),
            color=(0.2, 0.7, 1.0, 0.8),
            width=3.5,
            antialias=True
        )
        self.view3d.addItem(self.pedestal_item)

        self.laser_ray = gl.GLLinePlotItem(
            pos=np.array([[0, 0, self.sensor_height_mm], [300, 0, self.sensor_height_mm]], dtype=np.float32),
            color=(0.0, 0.9, 1.0, 0.8),
            width=2.5,
            antialias=True
        )
        self.view3d.addItem(self.laser_ray)

        self.laser_target = gl.GLScatterPlotItem(
            pos=np.array([[300, 0, self.sensor_height_mm]], dtype=np.float32),
            color=np.array([[1.0, 0.2, 0.1, 1.0]], dtype=np.float32),
            size=9,
            pxMode=True
        )
        self.view3d.addItem(self.laser_target)

        layout_3d.addWidget(self.view3d)
        splitter.addWidget(container_3d)

        splitter.setSizes([430, 910])
        main_layout.addWidget(splitter)

        self.actualizar_puertos()

    def _crear_tab_escaneo(self):
        """Crea el contenido de la pestaña de Escaneo 3D."""
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(10)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        scroll.setStyleSheet("QScrollArea { background-color: transparent; border: none; }")

        container = QtWidgets.QWidget()
        c_layout = QtWidgets.QVBoxLayout(container)
        c_layout.setContentsMargins(4, 4, 4, 4)
        c_layout.setSpacing(10)

        # 1. Control del Ciclo de Escaneo y Resolución Angular
        gb_scan = QtWidgets.QGroupBox("🎮 Control de Barrido Automático")
        l_scan = QtWidgets.QVBoxLayout()

        # Selector de Resolución Angular
        layout_res = QtWidgets.QHBoxLayout()
        layout_res.addWidget(QtWidgets.QLabel("Resolución Angular:"))
        self.cb_resolucion = QtWidgets.QComboBox()
        self.cb_resolucion.addItem("0.5° (Alta Densidad)", 0.5)
        self.cb_resolucion.addItem("1.0° (Equilibrado)", 1.0)
        self.cb_resolucion.addItem("2.0° (Rápido)", 2.0)

        # Cargar resolución previa
        saved_res = float(self.config_manager.get("scan", "resolution", 1.0))
        for i in range(self.cb_resolucion.count()):
            if abs(self.cb_resolucion.itemData(i) - saved_res) < 0.05:
                self.cb_resolucion.setCurrentIndex(i)
                break
        self.cb_resolucion.currentIndexChanged.connect(self._on_resolucion_cambiada)
        layout_res.addWidget(self.cb_resolucion)
        l_scan.addLayout(layout_res)

        # Fila 1: Iniciar y Detener
        h_scan_btns = QtWidgets.QHBoxLayout()
        self.btn_iniciar_escaneo = QtWidgets.QPushButton("▶️ Iniciar Escaneo")
        self.btn_iniciar_escaneo.setObjectName("btn_scan_start")
        self.btn_iniciar_escaneo.clicked.connect(self.iniciar_escaneo_cmd)

        self.btn_detener_escaneo = QtWidgets.QPushButton("⏹️ Detener")
        self.btn_detener_escaneo.setObjectName("btn_stop")
        self.btn_detener_escaneo.setEnabled(False)
        self.btn_detener_escaneo.clicked.connect(self.detener_escaneo_cmd)

        h_scan_btns.addWidget(self.btn_iniciar_escaneo)
        h_scan_btns.addWidget(self.btn_detener_escaneo)
        l_scan.addLayout(h_scan_btns)

        # Fila 2: Pausar y Reanudar
        h_ctrl_btns = QtWidgets.QHBoxLayout()
        self.btn_pausa = QtWidgets.QPushButton("⏸️ Pausar")
        self.btn_pausa.setEnabled(False)
        self.btn_pausa.clicked.connect(self.pausar_escaneo_cmd)

        self.btn_reanudar = QtWidgets.QPushButton("▶️ Reanudar")
        self.btn_reanudar.setEnabled(False)
        self.btn_reanudar.clicked.connect(self.reanudar_escaneo_cmd)

        h_ctrl_btns.addWidget(self.btn_pausa)
        h_ctrl_btns.addWidget(self.btn_reanudar)
        l_scan.addLayout(h_ctrl_btns)

        # Barra de Progreso de Escaneo
        self.progress_bar = QtWidgets.QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("Progreso de escaneo: %p%")
        self.progress_bar.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #333344;
                border-radius: 4px;
                background-color: #1a1a24;
                color: #ffffff;
                text-align: center;
                font-weight: bold;
                height: 22px;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #007acc, stop:0.5 #00b4d8, stop:1 #00f0ff);
                border-radius: 3px;
            }
        """)
        l_scan.addWidget(self.progress_bar)

        self.btn_limpiar = QtWidgets.QPushButton("🧹 Limpiar Nube")
        self.btn_limpiar.clicked.connect(self.limpiar_puntos)
        l_scan.addWidget(self.btn_limpiar)

        gb_scan.setLayout(l_scan)
        c_layout.addWidget(gb_scan)

        # 2. Inversión Mecánica de Servos
        gb_servos = QtWidgets.QGroupBox("🔄 Sentido de Rotación (Pan / Tilt)")
        layout_servos = QtWidgets.QVBoxLayout()

        self.chk_inv_pan = QtWidgets.QCheckBox("Invertir Pan (180° - Horario/Antihorario)")
        self.chk_inv_pan.setChecked(bool(self.config_manager.get("servos", "invert_pan", False)))
        self.chk_inv_pan.clicked.connect(self._on_chk_inv_pan_clicked)
        layout_servos.addWidget(self.chk_inv_pan)

        self.chk_inv_tilt = QtWidgets.QCheckBox("Invertir Tilt (180° - Inclinación vertical)")
        self.chk_inv_tilt.setChecked(bool(self.config_manager.get("servos", "invert_tilt", False)))
        self.chk_inv_tilt.clicked.connect(self._on_chk_inv_tilt_clicked)
        layout_servos.addWidget(self.chk_inv_tilt)

        gb_servos.setLayout(layout_servos)
        c_layout.addWidget(gb_servos)

        # 3. Filtro SOR
        gb_sor = QtWidgets.QGroupBox("🧹 Filtro Estadístico de Ruido (SOR)")
        layout_sor = QtWidgets.QVBoxLayout()

        self.chk_sor_export = QtWidgets.QCheckBox("Filtrar atípicos (SOR) al exportar PLY")
        self.chk_sor_export.setChecked(bool(self.config_manager.get("sor_filter", "enabled_export", True)))
        layout_sor.addWidget(self.chk_sor_export)

        layout_sor_params = QtWidgets.QHBoxLayout()
        layout_sor_params.addWidget(QtWidgets.QLabel("k vecinos:"))
        self.spin_sor_k = QtWidgets.QSpinBox()
        self.spin_sor_k.setRange(3, 100)
        self.spin_sor_k.setValue(int(self.config_manager.get("sor_filter", "k_neighbors", 16)))
        layout_sor_params.addWidget(self.spin_sor_k)

        layout_sor_params.addWidget(QtWidgets.QLabel("Std (σ):"))
        self.spin_sor_std = QtWidgets.QDoubleSpinBox()
        self.spin_sor_std.setRange(0.1, 5.0)
        self.spin_sor_std.setSingleStep(0.1)
        self.spin_sor_std.setValue(float(self.config_manager.get("sor_filter", "std_ratio", 1.5)))
        layout_sor_params.addWidget(self.spin_sor_std)
        layout_sor.addLayout(layout_sor_params)

        self.btn_aplicar_sor_live = QtWidgets.QPushButton("Filtrar Outliers en Pantalla")
        self.btn_aplicar_sor_live.setObjectName("btn_filter")
        self.btn_aplicar_sor_live.clicked.connect(self.aplicar_sor_en_pantalla)
        layout_sor.addWidget(self.btn_aplicar_sor_live)

        gb_sor.setLayout(layout_sor)
        c_layout.addWidget(gb_sor)

        # 4. Exportación de Archivos
        gb_export = QtWidgets.QGroupBox("💾 Exportación de Nube 3D")
        l_exp = QtWidgets.QVBoxLayout()

        self.btn_ply = QtWidgets.QPushButton("📦 Exportar a .PLY (Blender / MeshLab)")
        self.btn_ply.clicked.connect(self.exportar_ply)
        l_exp.addWidget(self.btn_ply)

        self.btn_csv = QtWidgets.QPushButton("📄 Exportar a .CSV (Cartesiano + Crudo)")
        self.btn_csv.clicked.connect(self.exportar_csv)
        l_exp.addWidget(self.btn_csv)

        gb_export.setLayout(l_exp)
        c_layout.addWidget(gb_export)

        # 5. Telemetría de Escaneo
        gb_stats = QtWidgets.QGroupBox("📊 Telemetría del Escaneo")
        l_stats = QtWidgets.QVBoxLayout()

        self.lbl_puntos = QtWidgets.QLabel("Puntos capturados: 0")
        self.lbl_telemetria = QtWidgets.QLabel("Pan: --° | Tilt: --° | Dist: -- mm")
        self.lbl_fps_panel = QtWidgets.QLabel("FPS Render: --")
        self.lbl_fps_panel.setStyleSheet("color: #00e5ff; font-weight: bold;")
        self.lbl_estado = QtWidgets.QLabel("Estado: Desconectado")
        self.lbl_estado.setStyleSheet("color: #ffaa00; font-weight: bold;")

        l_stats.addWidget(self.lbl_puntos)
        l_stats.addWidget(self.lbl_telemetria)
        l_stats.addWidget(self.lbl_fps_panel)
        l_stats.addWidget(self.lbl_estado)

        gb_stats.setLayout(l_stats)
        c_layout.addWidget(gb_stats)

        c_layout.addStretch()
        scroll.setWidget(container)
        layout.addWidget(scroll)

        return widget

    def init_status_bar(self):
        """Inicializa la barra inferior de información."""
        bar = self.statusBar()

        self.sb_estado = QtWidgets.QLabel("  ● Desconectado  ")
        self.sb_estado.setStyleSheet("color: #ffaa00; font-weight: bold;")

        self.sb_latencia = QtWidgets.QLabel("  Ping: -- ms  ")
        self.sb_latencia.setStyleSheet("color: #81c784; padding: 2px 6px;")

        self.sb_fps = QtWidgets.QLabel("  FPS: 0.0  ")
        self.sb_fps.setStyleSheet("color: #00e5ff; font-weight: bold; background-color: #23232e; border-radius: 3px; padding: 2px 6px;")

        self.sb_puntos = QtWidgets.QLabel("  Puntos: 0  ")
        self.sb_puntos.setStyleSheet("color: #ffffff; padding: 2px 6px;")

        self.sb_telemetria = QtWidgets.QLabel("  Pan: --° | Tilt: --° | Dist: -- mm  ")
        self.sb_telemetria.setStyleSheet("color: #b0bec5; padding: 2px 6px;")

        bar.addWidget(self.sb_estado)
        bar.addWidget(self.sb_latencia)
        bar.addWidget(self.sb_fps)
        bar.addWidget(self.sb_puntos)
        bar.addWidget(self.sb_telemetria)

    def _on_resolucion_cambiada(self):
        res = float(self.cb_resolucion.currentData())
        self.config_manager.set("scan", "resolution", res)
        self.config_manager.save()
        if self.serial_controller and self.serial_controller.isRunning():
            self.serial_controller.set_scan_resolution(res)
            self.enviar_comando_serial(f"CMD:RES,{res}")

    def actualizar_puertos(self):
        """Escanea puertos serie físicos e incluye el simulador virtual para desarrollo."""
        self.cb_ports.clear()
        self.cb_ports.addItem(SIMULATOR_PORT)

        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.cb_ports.addItem(f"{p.device} - {p.description}")

    def _actualizar_ui_desconectado(self):
        """Restaura la interfaz al estado de desconexión sin bloquear el hilo principal."""
        self.btn_conectar.setText("Conectar Dispositivo")
        self.btn_conectar.setObjectName("")
        self.btn_conectar.setStyle(self.btn_conectar.style())
        self.lbl_estado.setText("Estado: Desconectado")
        self.lbl_estado.setStyleSheet("color: #ffaa00; font-weight: bold;")
        self.sb_estado.setText("  ● Desconectado  ")
        self.sb_estado.setStyleSheet("color: #ffaa00; font-weight: bold;")
        self.sb_latencia.setText("  Ping: -- ms  ")
        self.btn_iniciar_escaneo.setEnabled(True)
        self.btn_detener_escaneo.setEnabled(False)
        self.btn_pausa.setEnabled(False)
        self.btn_reanudar.setEnabled(False)

    def toggle_conexion(self):
        """Conecta o desconecta el hilo de comunicación serial preservando la posición física."""
        if self.serial_controller and self.serial_controller.isRunning():
            ctrl = self.serial_controller
            self.serial_controller = None
            ctrl.stop()
            self._actualizar_ui_desconectado()
        else:
            puerto_sel = self.cb_ports.currentText()
            if not puerto_sel:
                QtWidgets.QMessageBox.warning(self, "Aviso", "Selecciona un puerto COM o el Simulador.")
                return

            puerto_str = SIMULATOR_PORT if SIMULATOR_PORT in puerto_sel else puerto_sel.split(' ')[0]
            baud = int(self.cb_baud.currentText())

            self.config_manager.set("connection", "baudrate", baud)
            self.config_manager.set("connection", "last_port", puerto_str)
            self.config_manager.save()

            # RECONEXIÓN SEGURA: Pasar la última posición angular conocida para NO auto-centrar ni enviar 0°
            self.serial_controller = SerialController(
                port=puerto_str,
                baudrate=baud,
                config_manager=self.config_manager,
                initial_pan=self.last_known_pan,
                initial_tilt=self.last_known_tilt
            )

            # Sincronizar sliders y spinboxes con la última posición SIN emitir comandos
            self.tab_calibracion.sincronizar_posicion_sin_envio(self.last_known_pan, self.last_known_tilt)

            self.serial_controller.punto_recibido.connect(self.agregar_punto)
            self.serial_controller.telemetria_calibracion.connect(self.on_telemetria_recibida)
            self.serial_controller.posicion_escaneo.connect(self.on_posicion_escaneo)
            self.serial_controller.evento_escaneo.connect(self.on_evento_escaneo)
            self.serial_controller.latencia_actualizada.connect(self.on_latencia_actualizada)
            self.serial_controller.error_conexion.connect(self.mostrar_error_serial)
            self.serial_controller.estado_conexion.connect(self.on_estado_conexion)

            self.serial_controller.start()

            self.btn_conectar.setText("Desconectar")
            self.btn_conectar.setObjectName("btn_stop")
            self.btn_conectar.setStyle(self.btn_conectar.style())

    def on_estado_conexion(self, conectado: bool, info: str):
        if conectado:
            self.lbl_estado.setText(f"Estado: {info}")
            self.lbl_estado.setStyleSheet("color: #00ff66; font-weight: bold;")
            self.sb_estado.setText("  ● Conectado  ")
            self.sb_estado.setStyleSheet("color: #00ff66; font-weight: bold;")
        else:
            self._actualizar_ui_desconectado()

    def enviar_comando_serial(self, cmd: str):
        """Envía comandos seguros hacia el microcontrolador a través del hilo serial."""
        if self.serial_controller and self.serial_controller.isRunning():
            self.serial_controller.enviar_comando(cmd)

    def iniciar_escaneo_cmd(self):
        """Inicia el barrido seguro extrayendo los límites vigentes y enviando CMD:START_SCAN parametrizado."""
        self.capturando = True
        self.progress_bar.setValue(0)

        # Extraer límites seguros vigentes desde config_manager
        p_min = int(self.config_manager.get("servos.pan_min", 0))
        p_max = int(self.config_manager.get("servos.pan_max", 150))
        t_min = int(self.config_manager.get("servos.tilt_min", 0))
        t_max = int(self.config_manager.get("servos.tilt_max", 180))
        step = float(self.cb_resolucion.currentData())

        self.scan_p_min = float(p_min)
        self.scan_p_max = float(p_max)
        self.scan_t_min = float(t_min)
        self.scan_t_max = float(t_max)
        self.scan_step = step

        if self.serial_controller:
            self.serial_controller.set_limits(p_min, p_max, t_min, t_max)
            self.serial_controller.set_scan_resolution(step)

        cmd_scan = f"CMD:START_SCAN,{p_min},{p_max},{t_min},{t_max},{step}"
        self.enviar_comando_serial(cmd_scan)

        self.btn_iniciar_escaneo.setEnabled(False)
        self.btn_detener_escaneo.setEnabled(True)
        self.btn_pausa.setEnabled(True)
        self.btn_reanudar.setEnabled(False)

        self.lbl_estado.setText(f"Estado: Escaneando ({p_min}°- {p_max}°, {t_min}°- {t_max}°, paso {step}°)...")
        self.lbl_estado.setStyleSheet("color: #00e5ff; font-weight: bold;")
        self.statusBar().showMessage(f"Escaneo iniciado: Pan [{p_min}°, {p_max}°], Tilt [{t_min}°, {t_max}°], Paso {step}°", 6000)

    def pausar_escaneo_cmd(self):
        """Pausa el barrido y congela los servomotores en su ángulo actual."""
        self.capturando = False
        self.enviar_comando_serial("CMD:PAUSE_SCAN")
        self.btn_pausa.setEnabled(False)
        self.btn_reanudar.setEnabled(True)
        self.lbl_estado.setText("Estado: ⏸️ Escaneo pausado (motores congelados)")
        self.lbl_estado.setStyleSheet("color: #ffb74d; font-weight: bold;")
        self.statusBar().showMessage("Pausa enviada: motores congelados en ángulo exacto", 5000)

    def reanudar_escaneo_cmd(self):
        """Reanuda el barrido exactamente desde donde fue pausado sin borrar puntos ni reiniciar progreso."""
        self.capturando = True
        self.enviar_comando_serial("CMD:RESUME_SCAN")
        self.btn_pausa.setEnabled(True)
        self.btn_reanudar.setEnabled(False)
        self.lbl_estado.setText("Estado: ▶️ Escaneando...")
        self.lbl_estado.setStyleSheet("color: #00e5ff; font-weight: bold;")
        self.statusBar().showMessage("Escaneo reanudado desde la posición previa", 5000)

    def detener_escaneo_cmd(self):
        """Detiene el barrido enviando CMD:STOP_SCAN y restablece botones."""
        self.capturando = False
        self.enviar_comando_serial("CMD:STOP_SCAN")
        self.btn_iniciar_escaneo.setEnabled(True)
        self.btn_detener_escaneo.setEnabled(False)
        self.btn_pausa.setEnabled(False)
        self.btn_reanudar.setEnabled(False)
        self.lbl_estado.setText("Estado: Escaneo Detenido")
        self.lbl_estado.setStyleSheet("color: #ffb74d; font-weight: bold;")
        self.statusBar().showMessage("Comando de parada enviado: CMD:STOP_SCAN", 4000)

    def limpiar_puntos(self):
        self.points.clear()
        self.colors.clear()
        self.raw_data.clear()
        self.scatter.setData(pos=np.empty((0, 3)), color=np.empty((0, 4)))
        self.lbl_puntos.setText("Puntos capturados: 0")
        self.sb_puntos.setText("  Puntos: 0  ")
        self.sb_telemetria.setText("  Pan: --° | Tilt: --° | Dist: -- mm  ")

    def on_posicion_escaneo(self, pan_raw: float, tilt_raw: float, dist_raw: float):
        """Calcula el porcentaje fluido sin saltos basado en pan y tilt reales recibidos."""
        p_min = self.scan_p_min
        p_max = self.scan_p_max
        t_min = self.scan_t_min
        t_max = self.scan_t_max
        step = max(0.1, self.scan_step)

        total_tilt_steps = max(1.0, math.floor((t_max - t_min) / step) + 1.0)
        total_pan_steps = max(1.0, math.floor((p_max - p_min) / step) + 1.0)
        total_points = total_tilt_steps * total_pan_steps

        t_clamped = max(t_min, min(t_max, tilt_raw))
        p_clamped = max(p_min, min(p_max, pan_raw))

        t_idx = math.floor((t_clamped - t_min) / step)
        if int(t_idx) % 2 == 0:
            p_idx = (p_clamped - p_min) / step
        else:
            p_idx = (p_max - p_clamped) / step

        pts_done = (t_idx * total_pan_steps) + p_idx
        porcentaje = max(0.0, min(100.0, (pts_done / total_points) * 100.0))
        self.progress_bar.setValue(int(round(porcentaje)))

    def on_evento_escaneo(self, evento_str: str):
        """Maneja las notificaciones de estado emitidas por el firmware."""
        if evento_str == "OK:SCAN_FINISHED":
            self.progress_bar.setValue(100)
            self.lbl_estado.setText("Estado: ✅ Escaneo topográfico finalizado con éxito")
            self.lbl_estado.setStyleSheet("color: #00ff66; font-weight: bold;")
            self.statusBar().showMessage("✅ Escaneo topográfico finalizado con éxito", 10000)

            # Restablecer botones
            self.btn_iniciar_escaneo.setEnabled(True)
            self.btn_detener_escaneo.setEnabled(False)
            self.btn_pausa.setEnabled(False)
            self.btn_reanudar.setEnabled(False)

            # Actualizar rayo 3D al centro (90°, 90°)
            self._actualizar_rayo_3d(90.0, 90.0, 450.0)

            QtWidgets.QMessageBox.information(
                self,
                "Escaneo Finalizado",
                "✅ Escaneo topográfico finalizado con éxito.\n\n"
                f"• Puntos capturados: {len(self.points)}\n"
                "• Los servomotores han regresado al centro mecánico (90°, 90°).\n"
                "• Puedes exportar la nube de puntos a PLY o CSV."
            )
        elif evento_str == "OK:SCAN_PAUSED":
            self.lbl_estado.setText("Estado: ⏸️ Escaneo pausado (motores congelados)")
            self.lbl_estado.setStyleSheet("color: #ffb74d; font-weight: bold;")
            self.statusBar().showMessage("⏸️ Escaneo pausado: motores congelados en ángulo exacto", 5000)
        elif evento_str == "OK:SCAN_RESUMED":
            self.lbl_estado.setText("Estado: ▶️ Escaneando...")
            self.lbl_estado.setStyleSheet("color: #00e5ff; font-weight: bold;")
            self.statusBar().showMessage("▶️ Escaneo reanudado desde la posición previa", 5000)

    def _on_chk_inv_tilt_clicked(self, checked):
        if checked:
            msg_box = QtWidgets.QMessageBox(self)
            msg_box.setIcon(QtWidgets.QMessageBox.Icon.Warning)
            msg_box.setWindowTitle("⚠️ Advertencia de Inversión Vertical (Tilt)")
            msg_box.setText(
                "Si activas 'Invertir Tilt', el eje vertical se volteará y la topografía "
                "del objeto se dibujará al revés (de cabeza) en el visor 3D.\n\n"
                "¿Estás seguro de que deseas escanear al revés?"
            )
            btn_aceptar = msg_box.addButton("Aceptar", QtWidgets.QMessageBox.ButtonRole.AcceptRole)
            btn_cancelar = msg_box.addButton("Cancelar", QtWidgets.QMessageBox.ButtonRole.RejectRole)
            msg_box.setDefaultButton(btn_cancelar)
            msg_box.exec()

            if msg_box.clickedButton() != btn_aceptar:
                self.chk_inv_tilt.blockSignals(True)
                self.chk_inv_tilt.setChecked(False)
                self.chk_inv_tilt.blockSignals(False)
                return

        self.config_manager.set("servos", "invert_tilt", checked)
        self.config_manager.save()
        if self.serial_controller and self.serial_controller.isRunning():
            self.serial_controller.set_inversion(self.chk_inv_pan.isChecked(), checked)
        if hasattr(self, 'tab_calibracion'):
            self.tab_calibracion.sincronizar_inversion(self.chk_inv_pan.isChecked(), checked)

    def _on_chk_inv_pan_clicked(self, checked):
        if checked:
            msg_box = QtWidgets.QMessageBox(self)
            msg_box.setIcon(QtWidgets.QMessageBox.Icon.Warning)
            msg_box.setWindowTitle("⚠️ Advertencia de Inversión Horizontal (Pan)")
            msg_box.setText(
                "Si activas 'Invertir Pan', el eje horizontal se reflejará en modo espejo "
                "(izquierda y derecha invertidas) en el visor 3D.\n\n"
                "¿Estás seguro de que deseas escanear invertido?"
            )
            btn_aceptar = msg_box.addButton("Aceptar", QtWidgets.QMessageBox.ButtonRole.AcceptRole)
            btn_cancelar = msg_box.addButton("Cancelar", QtWidgets.QMessageBox.ButtonRole.RejectRole)
            msg_box.setDefaultButton(btn_cancelar)
            msg_box.exec()

            if msg_box.clickedButton() != btn_aceptar:
                self.chk_inv_pan.blockSignals(True)
                self.chk_inv_pan.setChecked(False)
                self.chk_inv_pan.blockSignals(False)
                return

        self.config_manager.set("servos", "invert_pan", checked)
        self.config_manager.save()
        if self.serial_controller and self.serial_controller.isRunning():
            self.serial_controller.set_inversion(checked, self.chk_inv_tilt.isChecked())
        if hasattr(self, 'tab_calibracion'):
            self.tab_calibracion.sincronizar_inversion(checked, self.chk_inv_tilt.isChecked())

    def on_inversion_desde_calibracion(self, inv_p, inv_t):
        self.chk_inv_pan.blockSignals(True)
        self.chk_inv_pan.setChecked(inv_p)
        self.chk_inv_pan.blockSignals(False)

        self.chk_inv_tilt.blockSignals(True)
        self.chk_inv_tilt.setChecked(inv_t)
        self.chk_inv_tilt.blockSignals(False)

        if self.serial_controller and self.serial_controller.isRunning():
            self.serial_controller.set_inversion(inv_p, inv_t)

    def aplicar_nuevos_offsets(self, p_off, t_off, d_off):
        if self.serial_controller and self.serial_controller.isRunning():
            self.serial_controller.set_offsets(p_off, t_off, d_off)

    def aplicar_nuevos_limites(self, p_min, p_max, t_min, t_max):
        if self.serial_controller and self.serial_controller.isRunning():
            self.serial_controller.set_limits(p_min, p_max, t_min, t_max)

    def actualizar_fps(self, fps_val: float):
        fps_str = f"{fps_val:.1f} FPS"
        self.lbl_fps_panel.setText(f"FPS Render: {fps_str}")
        self.sb_fps.setText(f"  FPS: {fps_str}  ")

    def on_latencia_actualizada(self, lat_ms: float):
        self.tab_calibracion.actualizar_latencia(lat_ms)
        self.sb_latencia.setText(f"  Ping: {lat_ms:.1f} ms  ")

    def on_telemetria_recibida(self, pan, tilt, dist_raw, dist_eff):
        """Actualiza la telemetría y guarda la última posición angular conocida."""
        self.last_known_pan = pan
        self.last_known_tilt = tilt

        self.tab_calibracion.actualizar_telemetria(pan, tilt, dist_raw, dist_eff)
        self.lbl_telemetria.setText(f"Pan: {pan:.1f}° | Tilt: {tilt:.1f}° | Dist: {dist_eff:.1f} mm")
        self.sb_telemetria.setText(f"  Pan: {pan:.1f}° | Tilt: {tilt:.1f}° | Dist: {dist_eff:.1f} mm  ")

        self._actualizar_rayo_3d(pan, tilt, dist_eff)

    def actualizar_rayo_apuntador_jog(self, pan, tilt):
        """Actualiza la orientación del rayo 3D durante el jogging manual considerando horizonte en 90°."""
        d_off = float(self.config_manager.get("offsets.dist_offset_mm", 0.0))
        p_off = float(self.config_manager.get("offsets.pan_offset", 0.0))
        t_off = float(self.config_manager.get("offsets.tilt_offset", 0.0))
        inv_p = bool(self.config_manager.get("servos.invert_pan", False))
        inv_t = bool(self.config_manager.get("servos.invert_tilt", False))

        p_eff = pan + p_off
        t_eff = tilt + t_off
        if inv_p:
            p_eff = 180.0 - p_eff
        if inv_t:
            t_eff = 180.0 - t_eff

        dist_est = 450.0 + d_off
        self._actualizar_rayo_3d(p_eff, t_eff, dist_est)

    def actualizar_altura_sensor(self, h_mm: float):
        """Actualiza la altura física del sensor en la cinemática y la escena OpenGL."""
        self.sensor_height_mm = float(h_mm)
        if self.serial_controller and self.serial_controller.isRunning():
            self.serial_controller.set_sensor_height(self.sensor_height_mm)
        if hasattr(self, 'pedestal_item'):
            self.pedestal_item.setData(pos=np.array([[0.0, 0.0, 0.0], [0.0, 0.0, self.sensor_height_mm]], dtype=np.float32))

    def _actualizar_rayo_3d(self, pan, tilt, dist):
        """Dibuja en OpenGL el vector hacia donde apunta el sensor con el origen en la altura física Z=+sensor_height_mm."""
        elev_grados = tilt - 90.0
        elev_rad = math.radians(elev_grados)
        pan_rad = math.radians(pan)

        x = dist * math.cos(elev_rad) * math.cos(pan_rad)
        y = dist * math.cos(elev_rad) * math.sin(pan_rad)
        z = (dist * math.sin(elev_rad)) + self.sensor_height_mm

        line_pts = np.array([[0.0, 0.0, self.sensor_height_mm], [x, y, z]], dtype=np.float32)
        target_pts = np.array([[x, y, z]], dtype=np.float32)

        self.laser_ray.setData(pos=line_pts)
        self.laser_target.setData(pos=target_pts)

    def agregar_punto(self, x, y, z, pan, tilt, dist_raw, dist_eff):
        if not self.capturando:
            return

        self.points.append([x, y, z])
        self.raw_data.append([x, y, z, pan, tilt, dist_raw, dist_eff])

        # Color según elevación Z sobre la superficie de apoyo (mesa/suelo Z=0)
        norm_z = float(np.clip(z / 350.0, 0.0, 1.0))
        r = norm_z
        g = 1.0 - abs(norm_z - 0.5) * 2.0
        b = 1.0 - norm_z
        self.colors.append([r, g, b, 1.0])

        if len(self.points) % 5 == 0:
            pos_arr = np.array(self.points, dtype=np.float32)
            col_arr = np.array(self.colors, dtype=np.float32)
            self.scatter.setData(pos=pos_arr, color=col_arr, size=4, pxMode=True)
            self.lbl_puntos.setText(f"Puntos capturados: {len(self.points)}")
            self.sb_puntos.setText(f"  Puntos: {len(self.points)}  ")

    def aplicar_sor_en_pantalla(self):
        if len(self.points) < 5:
            QtWidgets.QMessageBox.information(self, "Aviso", "Se requieren más puntos para aplicar el filtro SOR.")
            return

        pts_arr = np.array(self.points)
        cols_arr = np.array(self.colors)
        raw_arr = np.array(self.raw_data)

        k = self.spin_sor_k.value()
        std = self.spin_sor_std.value()

        mask = statistical_outlier_removal(pts_arr, k_neighbors=k, std_ratio=std)
        total_previo = len(pts_arr)
        inliers_count = int(np.sum(mask))
        outliers_count = total_previo - inliers_count

        self.points = pts_arr[mask].tolist()
        self.colors = cols_arr[mask].tolist()
        self.raw_data = raw_arr[mask].tolist()

        if self.points:
            self.scatter.setData(pos=np.array(self.points, dtype=np.float32), color=np.array(self.colors, dtype=np.float32), size=4, pxMode=True)
        else:
            self.scatter.setData(pos=np.empty((0, 3)), color=np.empty((0, 4)))

        self.lbl_puntos.setText(f"Puntos capturados: {len(self.points)}")
        self.sb_puntos.setText(f"  Puntos: {len(self.points)}  ")

        QtWidgets.QMessageBox.information(
            self,
            "Filtro SOR Aplicado",
            f"Resultados del filtro estadístico:\n"
            f"• Puntos analizados: {total_previo}\n"
            f"• Inliers conservados: {inliers_count}\n"
            f"• Outliers removidos: {outliers_count}\n"
            f"(k={k}, σ={std})"
        )

    def exportar_ply(self):
        if not self.points:
            QtWidgets.QMessageBox.warning(self, "Aviso", "No hay puntos para exportar.")
            return

        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Guardar Archivo PLY", "escaneo.ply", "Polygon File (*.ply)"
        )
        if not path:
            return

        pts = np.array(self.points)
        cols = np.array(self.colors)
        total_original = len(pts)
        outliers_removidos = 0

        if self.chk_sor_export.isChecked() and total_original > self.spin_sor_k.value():
            k = self.spin_sor_k.value()
            std = self.spin_sor_std.value()
            mask_inliers = statistical_outlier_removal(pts, k_neighbors=k, std_ratio=std)
            pts = pts[mask_inliers]
            cols = cols[mask_inliers]
            outliers_removidos = total_original - len(pts)

        cols_byte = (cols[:, :3] * 255).astype(int)

        with open(path, 'w', encoding='utf-8') as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {len(pts)}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
            f.write("end_header\n")
            for i in range(len(pts)):
                f.write(f"{pts[i][0]:.2f} {pts[i][1]:.2f} {pts[i][2]:.2f} {cols_byte[i][0]} {cols_byte[i][1]} {cols_byte[i][2]}\n")

        msg = f"Archivo PLY exportado correctamente:\n{path}\n\n• Puntos exportados: {len(pts)}"
        if self.chk_sor_export.isChecked():
            msg += f"\n• Outliers filtrados (SOR): {outliers_removidos}"

        QtWidgets.QMessageBox.information(self, "Éxito", msg)

    def exportar_csv(self):
        if not self.raw_data:
            QtWidgets.QMessageBox.warning(self, "Aviso", "No hay datos para exportar.")
            return

        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Guardar Archivo CSV", "escaneo.csv", "CSV File (*.csv)"
        )
        if not path:
            return

        np.savetxt(
            path, 
            np.array(self.raw_data), 
            delimiter=",", 
            header="X_mm,Y_mm,Z_mm,Pan_eff,Tilt_eff,Dist_raw_mm,Dist_eff_mm", 
            comments="", 
            fmt="%.2f"
        )
        QtWidgets.QMessageBox.information(self, "Éxito", f"Datos CSV guardados en:\n{path}")

    def mostrar_error_serial(self, err_msg):
        """
        Maneja la desconexión o fallo del puerto COM de forma segura.
        Cierra el puerto, actualiza la UI y evita la cascada de ventanas modales.
        """
        self._actualizar_ui_desconectado()

        if self.serial_controller:
            ctrl = self.serial_controller
            self.serial_controller = None
            ctrl.stop()

        # Evitar abrir múltiples diálogos si se produjeron varios eventos
        if self._dialogo_error_activo:
            return
        self._dialogo_error_activo = True

        try:
            QtWidgets.QMessageBox.critical(
                self,
                "Desconexión del Dispositivo",
                f"{err_msg}\n\nEl puerto serie ha sido cerrado de forma segura.\nPuedes volver a conectar el cable USB o seleccionar otro puerto sin reiniciar la app."
            )
        finally:
            self._dialogo_error_activo = False

    def closeEvent(self, event):
        """Cierra el hilo secundario ordenadamente al salir de la aplicación."""
        if self.serial_controller:
            ctrl = self.serial_controller
            self.serial_controller = None
            ctrl.stop()
        event.accept()


if __name__ == "__main__":
    # Configuración de AppUserModelID para Windows (agrupa la app en la barra de tareas y muestra su icono)
    if sys.platform == "win32":
        try:
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("lidar.escaner3d.app.v1")
        except Exception:
            pass

    app = QtWidgets.QApplication(sys.argv)

    # Configuración de icono global para ventana principal y diálogos secundarios
    if os.path.exists(ICON_PATH):
        app.setWindowIcon(QtGui.QIcon(ICON_PATH))

    ventana = LidarScannerGUI()
    ventana.show()
    sys.exit(app.exec())
