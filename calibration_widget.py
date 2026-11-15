"""
Módulo de la Pestaña de Calibración de Hardware (calibration_widget.py)
Proporciona controles de Jogging seguros (validación solo con ENTER),
presets mecánicos, compensación de offsets con persistencia en config.json
y monitoreo de telemetría y latencia.
"""

from PyQt6 import QtWidgets, QtCore, QtGui


class EnterValidatedSpinBox(QtWidgets.QSpinBox):
    """
    QSpinBox que no emite comandos hacia el hardware mientras se escribe.
    Solo emite la señal 'valor_confirmado' al presionar explícitamente ENTER o perder el foco.
    """
    valor_confirmado = QtCore.pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setKeyboardTracking(False)  # Evita que se emitan eventos en cada dígito
        self.editingFinished.connect(self._on_editing_finished)

    def _on_editing_finished(self):
        self.valor_confirmado.emit(self.value())

    def keyPressEvent(self, event: QtGui.QKeyEvent):
        if event.key() in (QtCore.Qt.Key.Key_Return, QtCore.Qt.Key.Key_Enter):
            self.interpretText()
            self.clearFocus()
            self.valor_confirmado.emit(self.value())
            event.accept()
            return
        super().keyPressEvent(event)


class CalibrationWidget(QtWidgets.QWidget):
    """
    Panel interactivo para la calibración física, verificación de rangos y ajuste
    de offsets del escáner Pan-Tilt (MG996R en GPIO 18/19 y VL53L0X).
    """
    comando_solicitado = QtCore.pyqtSignal(str)
    offsets_modificados = QtCore.pyqtSignal(float, float, float)
    limites_modificados = QtCore.pyqtSignal(float, float, float, float)
    posicion_jog_cambiada = QtCore.pyqtSignal(float, float)
    mensaje_estado = QtCore.pyqtSignal(str, int)
    altura_sensor_modificada = QtCore.pyqtSignal(float)
    inversion_modificada = QtCore.pyqtSignal(bool, bool)

    def __init__(self, config_manager, parent=None):
        super().__init__(parent)
        self.cfg = config_manager

        # Timers para debounce en sliders (35 ms)
        self._timer_pan = QtCore.QTimer(self)
        self._timer_pan.setSingleShot(True)
        self._timer_pan.setInterval(35)
        self._timer_pan.timeout.connect(self._transmitir_pan)

        self._timer_tilt = QtCore.QTimer(self)
        self._timer_tilt.setSingleShot(True)
        self._timer_tilt.setInterval(35)
        self._timer_tilt.timeout.connect(self._transmitir_tilt)

        # Variables de estado
        self._pending_pan = 90
        self._pending_tilt = 90

        # Secuencia de prueba de límites
        self._sweep_timer = QtCore.QTimer(self)
        self._sweep_timer.setInterval(35)
        self._sweep_timer.timeout.connect(self._sweep_step)
        self._sweep_active = False
        self._sweep_sequence = []
        self._sweep_index = 0

        self.init_ui()
        self.cargar_datos_config()

    def init_ui(self):
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(10)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        scroll.setStyleSheet("QScrollArea { background-color: transparent; border: none; }")

        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(12)

        # =========================================================================
        # 1. MONITOR DE TELEMETRÍA DE CALIBRACIÓN
        # =========================================================================
        gb_telemetry = QtWidgets.QGroupBox("📡 Monitor de Sensor y Telemetría")
        grid_tel = QtWidgets.QGridLayout()
        grid_tel.setHorizontalSpacing(8)
        grid_tel.setVerticalSpacing(6)

        self.lbl_dist_raw = QtWidgets.QLabel("-- mm")
        self.lbl_dist_raw.setStyleSheet("""
            background-color: #1a1a24;
            color: #ffb74d;
            font-size: 15px;
            font-weight: bold;
            padding: 6px;
            border-radius: 4px;
            border: 1px solid #3c3c4e;
        """)
        self.lbl_dist_raw.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

        self.lbl_dist_eff = QtWidgets.QLabel("-- mm")
        self.lbl_dist_eff.setStyleSheet("""
            background-color: #1a1a24;
            color: #00e5ff;
            font-size: 15px;
            font-weight: bold;
            padding: 6px;
            border-radius: 4px;
            border: 1px solid #00bcd4;
        """)
        self.lbl_dist_eff.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

        self.lbl_latencia = QtWidgets.QLabel("-- ms")
        self.lbl_latencia.setStyleSheet("""
            background-color: #1a1a24;
            color: #69f0ae;
            font-size: 13px;
            font-weight: bold;
            padding: 6px;
            border-radius: 4px;
            border: 1px solid #2e7d32;
        """)
        self.lbl_latencia.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

        grid_tel.addWidget(QtWidgets.QLabel("Distancia Cruda (VL53L0X):"), 0, 0)
        grid_tel.addWidget(self.lbl_dist_raw, 0, 1)

        grid_tel.addWidget(QtWidgets.QLabel("Distancia Calibrada (+Offset):"), 1, 0)
        grid_tel.addWidget(self.lbl_dist_eff, 1, 1)

        grid_tel.addWidget(QtWidgets.QLabel("Latencia Serial (Ping):"), 2, 0)
        grid_tel.addWidget(self.lbl_latencia, 2, 1)

        gb_telemetry.setLayout(grid_tel)
        layout.addWidget(gb_telemetry)

        # =========================================================================
        # 2. CONTROL DE POSICIÓN EN TIEMPO REAL (JOGGING CON ENTER)
        # =========================================================================
        gb_jog = QtWidgets.QGroupBox("🕹️ Control de Posición en Tiempo Real (Jogging)")
        layout_jog = QtWidgets.QVBoxLayout()
        layout_jog.setSpacing(10)

        # --- EJE PAN (BASE - GPIO 18) ---
        layout_jog.addWidget(QtWidgets.QLabel("<b>Servo Pan (Base, GPIO 18):</b> <span style='color:#b0bec5; font-size:11px;'>(Escribir y pulsar ENTER)</span>"))
        h_pan_ctrl = QtWidgets.QHBoxLayout()
        self.slider_pan = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.slider_pan.setRange(0, 150)
        self.slider_pan.setValue(90)
        self.slider_pan.setTickPosition(QtWidgets.QSlider.TickPosition.TicksBelow)
        self.slider_pan.setTickInterval(15)

        self.spin_pan = EnterValidatedSpinBox()
        self.spin_pan.setRange(0, 150)
        self.spin_pan.setValue(90)
        self.spin_pan.setSuffix("°")
        self.spin_pan.setFixedWidth(70)
        self.spin_pan.setToolTip("Escribe el ángulo deseado y presiona ENTER para mover el servo con seguridad")

        h_pan_ctrl.addWidget(self.slider_pan)
        h_pan_ctrl.addWidget(self.spin_pan)
        layout_jog.addLayout(h_pan_ctrl)

        # Botones de ajuste fino Pan
        h_pan_fine = QtWidgets.QHBoxLayout()
        h_pan_fine.setSpacing(4)
        btn_p_m5 = QtWidgets.QPushButton("-5°")
        btn_p_m1 = QtWidgets.QPushButton("-1°")
        btn_p_p1 = QtWidgets.QPushButton("+1°")
        btn_p_p5 = QtWidgets.QPushButton("+5°")

        for b in (btn_p_m5, btn_p_m1, btn_p_p1, btn_p_p5):
            b.setStyleSheet("background-color: #2b2b36; border: 1px solid #4a4a5e; padding: 4px 8px; font-weight: normal;")

        btn_p_m5.clicked.connect(lambda: self._ajuste_fino_pan(-5))
        btn_p_m1.clicked.connect(lambda: self._ajuste_fino_pan(-1))
        btn_p_p1.clicked.connect(lambda: self._ajuste_fino_pan(1))
        btn_p_p5.clicked.connect(lambda: self._ajuste_fino_pan(5))

        h_pan_fine.addWidget(QtWidgets.QLabel("Ajuste fino:"))
        h_pan_fine.addWidget(btn_p_m5)
        h_pan_fine.addWidget(btn_p_m1)
        h_pan_fine.addWidget(btn_p_p1)
        h_pan_fine.addWidget(btn_p_p5)
        layout_jog.addLayout(h_pan_fine)

        # Conexiones Pan: slider envía con debounce; spinbox solo al confirmar con ENTER
        self.slider_pan.valueChanged.connect(self._on_slider_pan_changed)
        self.slider_pan.sliderReleased.connect(self._transmitir_pan_inmediato)
        self.spin_pan.valor_confirmado.connect(self._on_spin_pan_confirmed)

        layout_jog.addSpacing(6)

        # --- EJE TILT (ELEVACIÓN - GPIO 19) ---
        layout_jog.addWidget(QtWidgets.QLabel("<b>Servo Tilt (Elevación, GPIO 19):</b> <span style='color:#b0bec5; font-size:11px;'>(Escribir y pulsar ENTER)</span>"))
        h_tilt_ctrl = QtWidgets.QHBoxLayout()
        self.slider_tilt = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.slider_tilt.setRange(0, 180)
        self.slider_tilt.setValue(90)
        self.slider_tilt.setTickPosition(QtWidgets.QSlider.TickPosition.TicksBelow)
        self.slider_tilt.setTickInterval(15)

        self.spin_tilt = EnterValidatedSpinBox()
        self.spin_tilt.setRange(0, 180)
        self.spin_tilt.setValue(90)
        self.spin_tilt.setSuffix("°")
        self.spin_tilt.setFixedWidth(70)
        self.spin_tilt.setToolTip("Escribe el ángulo deseado y presiona ENTER para mover el servo con seguridad")

        h_tilt_ctrl.addWidget(self.slider_tilt)
        h_tilt_ctrl.addWidget(self.spin_tilt)
        layout_jog.addLayout(h_tilt_ctrl)

        # Botones de ajuste fino Tilt
        h_tilt_fine = QtWidgets.QHBoxLayout()
        h_tilt_fine.setSpacing(4)
        btn_t_m5 = QtWidgets.QPushButton("-5°")
        btn_t_m1 = QtWidgets.QPushButton("-1°")
        btn_t_p1 = QtWidgets.QPushButton("+1°")
        btn_t_p5 = QtWidgets.QPushButton("+5°")

        for b in (btn_t_m5, btn_t_m1, btn_t_p1, btn_t_p5):
            b.setStyleSheet("background-color: #2b2b36; border: 1px solid #4a4a5e; padding: 4px 8px; font-weight: normal;")

        btn_t_m5.clicked.connect(lambda: self._ajuste_fino_tilt(-5))
        btn_t_m1.clicked.connect(lambda: self._ajuste_fino_tilt(-1))
        btn_t_p1.clicked.connect(lambda: self._ajuste_fino_tilt(1))
        btn_t_p5.clicked.connect(lambda: self._ajuste_fino_tilt(5))

        h_tilt_fine.addWidget(QtWidgets.QLabel("Ajuste fino:"))
        h_tilt_fine.addWidget(btn_t_m5)
        h_tilt_fine.addWidget(btn_t_m1)
        h_tilt_fine.addWidget(btn_t_p1)
        h_tilt_fine.addWidget(btn_t_p5)
        layout_jog.addLayout(h_tilt_fine)

        # Conexiones Tilt
        self.slider_tilt.valueChanged.connect(self._on_slider_tilt_changed)
        self.slider_tilt.sliderReleased.connect(self._transmitir_tilt_inmediato)
        self.spin_tilt.valor_confirmado.connect(self._on_spin_tilt_confirmed)

        gb_jog.setLayout(layout_jog)
        layout.addWidget(gb_jog)

        # =========================================================================
        # 3. PRESETS DE MONTAJE Y SEGURIDAD
        # =========================================================================
        gb_presets = QtWidgets.QGroupBox("🛡️ Presets de Montaje y Seguridad")
        layout_presets = QtWidgets.QVBoxLayout()
        layout_presets.setSpacing(8)

        self.btn_center = QtWidgets.QPushButton("🎯 Centrar Mecánica (90° / 90°)")
        self.btn_center.setToolTip("Envía CMD:CENTER y posiciona ambos servos a 90° para montaje o calibración física.")
        self.btn_center.clicked.connect(self.centrar_mecanica)
        layout_presets.addWidget(self.btn_center)

        self.btn_sweep = QtWidgets.QPushButton("🔍 Verificar Límites Seguros (Barrido)")
        self.btn_sweep.setToolTip("Realiza un barrido suave y seguro a los límites máximos y mínimos para verificar que los cables no se tensen.")
        self.btn_sweep.setStyleSheet("background-color: #388e3c; color: white;")
        self.btn_sweep.clicked.connect(self.toggle_verificar_limites)
        layout_presets.addWidget(self.btn_sweep)

        self.progress_sweep = QtWidgets.QProgressBar()
        self.progress_sweep.setRange(0, 100)
        self.progress_sweep.setValue(0)
        self.progress_sweep.setTextVisible(True)
        self.progress_sweep.setVisible(False)
        layout_presets.addWidget(self.progress_sweep)

        self.btn_disable_torque = QtWidgets.QPushButton("⚡ Desactivar PWM / Torque")
        self.btn_disable_torque.setToolTip("Desactiva los pulsos de control PWM en el ESP32 para mover los servos manualmente sin forzar engranajes.")
        self.btn_disable_torque.setStyleSheet("background-color: #546e7a; color: white;")
        self.btn_disable_torque.clicked.connect(self.desactivar_torque)
        layout_presets.addWidget(self.btn_disable_torque)

        gb_presets.setLayout(layout_presets)
        layout.addWidget(gb_presets)

        # =========================================================================
        # 4. COMPENSACIÓN DE DESALINEACIÓN (OFFSETS Y LÍMITES)
        # =========================================================================
        gb_offsets = QtWidgets.QGroupBox("📐 Offsets y Límites Mecánicos")
        form_offsets = QtWidgets.QFormLayout()
        form_offsets.setSpacing(6)

        self.spin_offset_pan = QtWidgets.QDoubleSpinBox()
        self.spin_offset_pan.setRange(-20.0, 20.0)
        self.spin_offset_pan.setSingleStep(0.1)
        self.spin_offset_pan.setSuffix("°")
        self.spin_offset_pan.setToolTip("Compensación angular para el eje Pan si el centro no está alineado a 0°.")
        form_offsets.addRow("Offset Pan:", self.spin_offset_pan)

        self.spin_offset_tilt = QtWidgets.QDoubleSpinBox()
        self.spin_offset_tilt.setRange(-20.0, 20.0)
        self.spin_offset_tilt.setSingleStep(0.1)
        self.spin_offset_tilt.setSuffix("°")
        self.spin_offset_tilt.setToolTip("Compensación angular para el eje Tilt si la montura tiene inclinación.")
        form_offsets.addRow("Offset Tilt:", self.spin_offset_tilt)

        self.spin_offset_dist = QtWidgets.QDoubleSpinBox()
        self.spin_offset_dist.setRange(-200.0, 200.0)
        self.spin_offset_dist.setSingleStep(1.0)
        self.spin_offset_dist.setSuffix(" mm")
        self.spin_offset_dist.setToolTip("Corrección lineal: distancia_real = distancia_leída + offset_mm")
        form_offsets.addRow("Offset Distancia (mm):", self.spin_offset_dist)

        # Altura física del sensor sobre la superficie de apoyo (mesa/suelo Z=0)
        self.spin_sensor_height = QtWidgets.QDoubleSpinBox()
        self.spin_sensor_height.setRange(0.0, 1000.0)
        self.spin_sensor_height.setSingleStep(1.0)
        self.spin_sensor_height.setValue(65.0)
        self.spin_sensor_height.setSuffix(" mm")
        self.spin_sensor_height.setToolTip("Altura física del sensor respecto a la superficie de apoyo de la mesa/suelo (Z=0)")
        form_offsets.addRow("Altura Sensor (Z=0):", self.spin_sensor_height)

        # Inversión de Sentido de Rotación (Pan / Tilt)
        h_inv = QtWidgets.QHBoxLayout()
        self.chk_inv_pan = QtWidgets.QCheckBox("Invertir Pan")
        self.chk_inv_pan.setToolTip("Invierte el giro horizontal (180° - Pan)")
        self.chk_inv_tilt = QtWidgets.QCheckBox("Invertir Tilt")
        self.chk_inv_tilt.setToolTip("Invierte la inclinación vertical (180° - Tilt)")
        h_inv.addWidget(self.chk_inv_pan)
        h_inv.addWidget(self.chk_inv_tilt)
        form_offsets.addRow("Inversión de Ejes:", h_inv)

        # Controles de límites de barrido seguro
        h_lim_pan = QtWidgets.QHBoxLayout()
        self.spin_pan_min = QtWidgets.QSpinBox()
        self.spin_pan_min.setRange(0, 180)
        self.spin_pan_min.setValue(0)
        self.spin_pan_min.setSuffix("°")
        self.spin_pan_max = QtWidgets.QSpinBox()
        self.spin_pan_max.setRange(0, 180)
        self.spin_pan_max.setValue(150)
        self.spin_pan_max.setSuffix("°")
        h_lim_pan.addWidget(QtWidgets.QLabel("Mín:"))
        h_lim_pan.addWidget(self.spin_pan_min)
        h_lim_pan.addWidget(QtWidgets.QLabel("Máx:"))
        h_lim_pan.addWidget(self.spin_pan_max)
        form_offsets.addRow("Límites Pan:", h_lim_pan)

        h_lim_tilt = QtWidgets.QHBoxLayout()
        self.spin_tilt_min = QtWidgets.QSpinBox()
        self.spin_tilt_min.setRange(0, 180)
        self.spin_tilt_min.setValue(0)
        self.spin_tilt_min.setSuffix("°")
        self.spin_tilt_max = QtWidgets.QSpinBox()
        self.spin_tilt_max.setRange(0, 180)
        self.spin_tilt_max.setValue(180)
        self.spin_tilt_max.setSuffix("°")
        h_lim_tilt.addWidget(QtWidgets.QLabel("Mín:"))
        h_lim_tilt.addWidget(self.spin_tilt_min)
        h_lim_tilt.addWidget(QtWidgets.QLabel("Máx:"))
        h_lim_tilt.addWidget(self.spin_tilt_max)
        form_offsets.addRow("Límites Tilt:", h_lim_tilt)

        self.btn_guardar_cfg = QtWidgets.QPushButton("💾 Guardar Configuración (config.json)")
        self.btn_guardar_cfg.setStyleSheet("""
            QPushButton {
                background-color: #00897b;
                color: white;
                font-size: 13px;
                padding: 8px;
            }
            QPushButton:hover { background-color: #00a896; }
        """)
        self.btn_guardar_cfg.clicked.connect(self.guardar_configuracion)

        gb_offsets.setLayout(form_offsets)
        layout.addWidget(gb_offsets)
        layout.addWidget(self.btn_guardar_cfg)

        layout.addStretch()

        scroll.setWidget(container)
        main_layout.addWidget(scroll)

        # Escuchar cambios de offsets para actualización en vivo
        self.spin_offset_pan.valueChanged.connect(self._notificar_offsets_en_vivo)
        self.spin_offset_tilt.valueChanged.connect(self._notificar_offsets_en_vivo)
        self.spin_offset_dist.valueChanged.connect(self._notificar_offsets_en_vivo)
        self.spin_sensor_height.valueChanged.connect(self._notificar_altura_sensor_en_vivo)

        self.spin_pan_min.valueChanged.connect(self._on_limites_spin_changed)
        self.spin_pan_max.valueChanged.connect(self._on_limites_spin_changed)
        self.spin_tilt_min.valueChanged.connect(self._on_limites_spin_changed)
        self.spin_tilt_max.valueChanged.connect(self._on_limites_spin_changed)

        self.chk_inv_pan.clicked.connect(self._on_chk_inv_pan_clicked)
        self.chk_inv_tilt.clicked.connect(self._on_chk_inv_tilt_clicked)

    def _notificar_altura_sensor_en_vivo(self, val):
        self.altura_sensor_modificada.emit(float(val))

    def cargar_datos_config(self):
        """Carga y aplica los límites, offsets y altura guardados en config.json asegurando no rebasar rangos."""
        cfg = self.cfg
        self.spin_offset_pan.setValue(float(cfg.get("offsets.pan_offset", 0.0)))
        self.spin_offset_tilt.setValue(float(cfg.get("offsets.tilt_offset", 0.0)))
        self.spin_offset_dist.setValue(float(cfg.get("offsets.dist_offset_mm", 0.0)))
        self.spin_sensor_height.setValue(float(cfg.get("geometry.sensor_height_mm", 65.0)))

        p_min = int(cfg.get("servos.pan_min", 0))
        p_max = int(cfg.get("servos.pan_max", 150))
        t_min = int(cfg.get("servos.tilt_min", 0))
        t_max = int(cfg.get("servos.tilt_max", 180))

        # Bloquear señales para evitar llamadas recursivas o reseteos accidentales
        for w in (self.spin_pan_min, self.spin_pan_max, self.spin_tilt_min, self.spin_tilt_max):
            w.blockSignals(True)

        self.spin_pan_min.setValue(p_min)
        self.spin_pan_max.setValue(p_max)
        self.spin_tilt_min.setValue(t_min)
        self.spin_tilt_max.setValue(t_max)

        for w in (self.spin_pan_min, self.spin_pan_max, self.spin_tilt_min, self.spin_tilt_max):
            w.blockSignals(False)

        # Cargar inversión de ejes
        self.chk_inv_pan.blockSignals(True)
        self.chk_inv_pan.setChecked(bool(cfg.get("servos.invert_pan", False)))
        self.chk_inv_pan.blockSignals(False)

        self.chk_inv_tilt.blockSignals(True)
        self.chk_inv_tilt.setChecked(bool(cfg.get("servos.invert_tilt", False)))
        self.chk_inv_tilt.blockSignals(False)

        self._actualizar_rangos_gui()

    def _on_limites_spin_changed(self):
        """Manejador para cuando el usuario ajusta los spinboxes de límites mínimos/máximos."""
        self._actualizar_rangos_gui()

    def _actualizar_rangos_gui(self):
        """
        Ajusta rigurosamente las propiedades .setMinimum() y .setMaximum()
        de todos los sliders y spinboxes de la interfaz.
        """
        p_min = self.spin_pan_min.value()
        p_max = max(p_min + 1, self.spin_pan_max.value())
        if self.spin_pan_max.value() < p_max:
            self.spin_pan_max.blockSignals(True)
            self.spin_pan_max.setValue(p_max)
            self.spin_pan_max.blockSignals(False)

        t_min = self.spin_tilt_min.value()
        t_max = max(t_min + 1, self.spin_tilt_max.value())
        if self.spin_tilt_max.value() < t_max:
            self.spin_tilt_max.blockSignals(True)
            self.spin_tilt_max.setValue(t_max)
            self.spin_tilt_max.blockSignals(False)

        # Aplicar límites a los controles de Pan
        self.slider_pan.blockSignals(True)
        self.spin_pan.blockSignals(True)
        self.slider_pan.setMinimum(p_min)
        self.slider_pan.setMaximum(p_max)
        self.spin_pan.setMinimum(p_min)
        self.spin_pan.setMaximum(p_max)
        # Clampear valores actuales
        val_p = max(p_min, min(p_max, self.slider_pan.value()))
        self.slider_pan.setValue(val_p)
        self.spin_pan.setValue(val_p)
        self.slider_pan.blockSignals(False)
        self.spin_pan.blockSignals(False)

        # Aplicar límites a los controles de Tilt
        self.slider_tilt.blockSignals(True)
        self.spin_tilt.blockSignals(True)
        self.slider_tilt.setMinimum(t_min)
        self.slider_tilt.setMaximum(t_max)
        self.spin_tilt.setMinimum(t_min)
        self.spin_tilt.setMaximum(t_max)
        val_t = max(t_min, min(t_max, self.slider_tilt.value()))
        self.slider_tilt.setValue(val_t)
        self.spin_tilt.setValue(val_t)
        self.slider_tilt.blockSignals(False)
        self.spin_tilt.blockSignals(False)

        self.limites_modificados.emit(float(p_min), float(p_max), float(t_min), float(t_max))

    def sincronizar_posicion_sin_envio(self, pan: float, tilt: float):
        """
        Sincroniza visualmente los sliders y spinboxes con la última posición conocida
        SIN emitir comandos hacia el hardware (usado en reconexiones seguras).
        """
        p_val = int(round(pan))
        t_val = int(round(tilt))

        p_val = max(self.slider_pan.minimum(), min(self.slider_pan.maximum(), p_val))
        t_val = max(self.slider_tilt.minimum(), min(self.slider_tilt.maximum(), t_val))

        self.slider_pan.blockSignals(True)
        self.spin_pan.blockSignals(True)
        self.slider_tilt.blockSignals(True)
        self.spin_tilt.blockSignals(True)

        self.slider_pan.setValue(p_val)
        self.spin_pan.setValue(p_val)
        self.slider_tilt.setValue(t_val)
        self.spin_tilt.setValue(t_val)

        self._pending_pan = p_val
        self._pending_tilt = t_val

        self.slider_pan.blockSignals(False)
        self.spin_pan.blockSignals(False)
        self.slider_tilt.blockSignals(False)
        self.spin_tilt.blockSignals(False)

        self.posicion_jog_cambiada.emit(float(p_val), float(t_val))

    def _on_slider_pan_changed(self, val):
        """Manejador del slider Pan: sincroniza el spinbox visualmente e inicia el debounce."""
        self.spin_pan.blockSignals(True)
        self.spin_pan.setValue(val)
        self.spin_pan.blockSignals(False)
        self._pending_pan = val
        self.posicion_jog_cambiada.emit(float(self._pending_pan), float(self._pending_tilt))
        self._timer_pan.start()

    def _on_spin_pan_confirmed(self, val):
        """
        Manejador seguro de Pan: SOLO se invoca al pulsar ENTER en el spinbox o perder el foco.
        """
        val = max(self.slider_pan.minimum(), min(self.slider_pan.maximum(), val))
        self.slider_pan.blockSignals(True)
        self.slider_pan.setValue(val)
        self.slider_pan.blockSignals(False)

        self.spin_pan.blockSignals(True)
        self.spin_pan.setValue(val)
        self.spin_pan.blockSignals(False)

        self._pending_pan = val
        self.posicion_jog_cambiada.emit(float(self._pending_pan), float(self._pending_tilt))
        self._transmitir_pan_inmediato()

    def _on_slider_tilt_changed(self, val):
        """Manejador del slider Tilt: sincroniza el spinbox visualmente e inicia el debounce."""
        self.spin_tilt.blockSignals(True)
        self.spin_tilt.setValue(val)
        self.spin_tilt.blockSignals(False)
        self._pending_tilt = val
        self.posicion_jog_cambiada.emit(float(self._pending_pan), float(self._pending_tilt))
        self._timer_tilt.start()

    def _on_spin_tilt_confirmed(self, val):
        """
        Manejador seguro de Tilt: SOLO se invoca al pulsar ENTER en el spinbox o perder el foco.
        """
        val = max(self.slider_tilt.minimum(), min(self.slider_tilt.maximum(), val))
        self.slider_tilt.blockSignals(True)
        self.slider_tilt.setValue(val)
        self.slider_tilt.blockSignals(False)

        self.spin_tilt.blockSignals(True)
        self.spin_tilt.setValue(val)
        self.spin_tilt.blockSignals(False)

        self._pending_tilt = val
        self.posicion_jog_cambiada.emit(float(self._pending_pan), float(self._pending_tilt))
        self._transmitir_tilt_inmediato()

    def _ajuste_fino_pan(self, delta):
        nuevo = max(self.slider_pan.minimum(), min(self.slider_pan.maximum(), self.slider_pan.value() + delta))
        self.slider_pan.setValue(nuevo)

    def _ajuste_fino_tilt(self, delta):
        nuevo = max(self.slider_tilt.minimum(), min(self.slider_tilt.maximum(), self.slider_tilt.value() + delta))
        self.slider_tilt.setValue(nuevo)

    def _transmitir_pan(self):
        cmd = f"CAL:P,{self._pending_pan}"
        self.comando_solicitado.emit(cmd)

    def _transmitir_pan_inmediato(self):
        self._timer_pan.stop()
        self._transmitir_pan()

    def _transmitir_tilt(self):
        cmd = f"CAL:T,{self._pending_tilt}"
        self.comando_solicitado.emit(cmd)

    def _transmitir_tilt_inmediato(self):
        self._timer_tilt.stop()
        self._transmitir_tilt()

    def centrar_mecanica(self):
        """Posiciona ambos servos en 90° enviando CMD:CENTER."""
        if self._sweep_active:
            self._detener_sweep()

        self.slider_pan.setValue(90)
        self.slider_tilt.setValue(90)
        self.comando_solicitado.emit("CMD:CENTER")

    def desactivar_torque(self):
        """Envía comando para liberar los servos de retención activa."""
        if self._sweep_active:
            self._detener_sweep()
        self.comando_solicitado.emit("CMD:DISABLE_TORQUE")

    def toggle_verificar_limites(self):
        """Inicia o cancela la rutina de verificación lenta de límites mecánicos."""
        if self._sweep_active:
            self._detener_sweep()
        else:
            self._iniciar_sweep()

    def _iniciar_sweep(self):
        self._sweep_active = True
        self.btn_sweep.setText("⏹️ Cancelar Verificación")
        self.btn_sweep.setStyleSheet("background-color: #d32f2f; color: white; font-weight: bold;")
        self.progress_sweep.setVisible(True)
        self.progress_sweep.setValue(0)

        p_min = self.spin_pan_min.value()
        p_max = self.spin_pan_max.value()
        t_min = self.spin_tilt_min.value()
        t_max = self.spin_tilt_max.value()

        seq = []
        cur_p = self.slider_pan.value()
        cur_t = self.slider_tilt.value()

        # 1. Llevar Pan de cur_p -> p_min
        step = -2 if cur_p > p_min else 2
        for p in range(cur_p, p_min, step):
            seq.append((p, cur_t))
        # 2. Pan de p_min -> p_max
        for p in range(p_min, p_max, 2):
            seq.append((p, cur_t))
        # 3. Pan de p_max -> 90
        for p in range(p_max, 90, -2):
            seq.append((p, cur_t))

        # 4. Tilt de cur_t -> t_min
        step_t = -2 if cur_t > t_min else 2
        for t in range(cur_t, t_min, step_t):
            seq.append((90, t))
        # 5. Tilt de t_min -> t_max
        for t in range(t_min, t_max, 2):
            seq.append((90, t))
        # 6. Tilt de t_max -> 90
        for t in range(t_max, 90, -2):
            seq.append((90, t))

        seq.append((90, 90))
        self._sweep_sequence = seq
        self._sweep_index = 0
        self._sweep_timer.start()

    def _sweep_step(self):
        if self._sweep_index < len(self._sweep_sequence):
            p, t = self._sweep_sequence[self._sweep_index]
            self.slider_pan.blockSignals(True)
            self.spin_pan.blockSignals(True)
            self.slider_tilt.blockSignals(True)
            self.spin_tilt.blockSignals(True)

            self.slider_pan.setValue(p)
            self.spin_pan.setValue(p)
            self.slider_tilt.setValue(t)
            self.spin_tilt.setValue(t)

            self.slider_pan.blockSignals(False)
            self.spin_pan.blockSignals(False)
            self.slider_tilt.blockSignals(False)
            self.spin_tilt.blockSignals(False)

            self.comando_solicitado.emit(f"CAL:P,{p}")
            self.comando_solicitado.emit(f"CAL:T,{t}")
            self.posicion_jog_cambiada.emit(float(p), float(t))

            pct = int((self._sweep_index / len(self._sweep_sequence)) * 100)
            self.progress_sweep.setValue(pct)
            self._sweep_index += 1
        else:
            self._detener_sweep()
            QtWidgets.QMessageBox.information(
                self,
                "Límites Verificados",
                "Barrido completado exitosamente.\nLos cables y la mecánica alcanzaron los límites seguros sin bloqueo."
            )

    def _detener_sweep(self):
        self._sweep_active = False
        self._sweep_timer.stop()
        self.btn_sweep.setText("🔍 Verificar Límites Seguros (Barrido)")
        self.btn_sweep.setStyleSheet("background-color: #388e3c; color: white;")
        self.progress_sweep.setVisible(False)

    def _notificar_offsets_en_vivo(self):
        p_off = self.spin_offset_pan.value()
        t_off = self.spin_offset_tilt.value()
        d_off = self.spin_offset_dist.value()
        self.offsets_modificados.emit(p_off, t_off, d_off)

    def guardar_configuracion(self):
        """
        Lee directamente los valores actuales de los SpinBoxes/Sliders de límites y offsets,
        actualiza el diccionario interno en memoria y guarda en config.json de forma atómica.
        """
        # 1. Lectura directa de controles
        p_min = int(self.spin_pan_min.value())
        p_max = int(self.spin_pan_max.value())
        t_min = int(self.spin_tilt_min.value())
        t_max = int(self.spin_tilt_max.value())

        p_off = float(self.spin_offset_pan.value())
        t_off = float(self.spin_offset_tilt.value())
        d_off = float(self.spin_offset_dist.value())
        h_sensor = float(self.spin_sensor_height.value())

        # 2. Actualización del diccionario interno de configuración en memoria
        self.cfg.set("servos", "pan_min", p_min)
        self.cfg.set("servos", "pan_max", p_max)
        self.cfg.set("servos", "tilt_min", t_min)
        self.cfg.set("servos", "tilt_max", t_max)

        self.cfg.set("offsets", "pan_offset", p_off)
        self.cfg.set("offsets", "tilt_offset", t_off)
        self.cfg.set("offsets", "dist_offset_mm", d_off)
        self.cfg.set("geometry", "sensor_height_mm", h_sensor)

        inv_p = bool(self.chk_inv_pan.isChecked())
        inv_t = bool(self.chk_inv_tilt.isChecked())
        self.cfg.set("servos", "invert_pan", inv_p)
        self.cfg.set("servos", "invert_tilt", inv_t)

        # 3. Guardado en disco en config.json de forma atómica
        if self.cfg.save():
            self._actualizar_rangos_gui()
            self.offsets_modificados.emit(p_off, t_off, d_off)
            self.limites_modificados.emit(float(p_min), float(p_max), float(t_min), float(t_max))
            self.altura_sensor_modificada.emit(h_sensor)
            self.inversion_modificada.emit(inv_p, inv_t)

            msg_status = f"✅ Configuración guardada en config.json: Pan [{p_min}°, {p_max}°], Tilt [{t_min}°, {t_max}°], Altura {h_sensor:.1f} mm"
            self.mensaje_estado.emit(msg_status, 6000)

            QtWidgets.QMessageBox.information(
                self,
                "Configuración Guardada",
                f"Parámetros persistidos exitosamente en {self.cfg.filepath}:\n\n"
                f"• Límites Pan: [{p_min}°, {p_max}°]\n"
                f"• Límites Tilt: [{t_min}°, {t_max}°]\n"
                f"• Altura Sensor (Z=0): {h_sensor:.1f} mm\n"
                f"• Offset Pan: {p_off:+.1f}°\n"
                f"• Offset Tilt: {t_off:+.1f}°\n"
                f"• Offset Distancia: {d_off:+.1f} mm"
            )
        else:
            QtWidgets.QMessageBox.critical(self, "Error", "No se pudo escribir el archivo config.json")

    def actualizar_telemetria(self, pan, tilt, dist_raw, dist_eff):
        """Actualiza las tarjetas de telemetría con datos entrantes del sensor."""
        self.lbl_dist_raw.setText(f"{dist_raw:.1f} mm")
        self.lbl_dist_eff.setText(f"{dist_eff:.1f} mm")

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

        self.cfg.set("servos", "invert_tilt", checked)
        self.cfg.save()
        self.inversion_modificada.emit(self.chk_inv_pan.isChecked(), checked)

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

        self.cfg.set("servos", "invert_pan", checked)
        self.cfg.save()
        self.inversion_modificada.emit(checked, self.chk_inv_tilt.isChecked())

    def sincronizar_inversion(self, inv_p: bool, inv_t: bool):
        self.chk_inv_pan.blockSignals(True)
        self.chk_inv_pan.setChecked(inv_p)
        self.chk_inv_pan.blockSignals(False)

        self.chk_inv_tilt.blockSignals(True)
        self.chk_inv_tilt.setChecked(inv_t)
        self.chk_inv_tilt.blockSignals(False)

    def actualizar_latencia(self, latencia_ms):
        """Actualiza el indicador de ping/latencia serial con código de color dinámico."""
        self.lbl_latencia.setText(f"{latencia_ms:.1f} ms")
        if latencia_ms < 35.0:
            color = "#69f0ae"
        elif latencia_ms < 90.0:
            color = "#ffd740"
        else:
            color = "#ff5252"
        self.lbl_latencia.setStyleSheet(f"""
            background-color: #1a1a24;
            color: {color};
            font-size: 13px;
            font-weight: bold;
            padding: 6px;
            border-radius: 4px;
            border: 1px solid {color};
        """)
