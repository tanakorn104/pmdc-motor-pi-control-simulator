import sys
import numpy as np
from collections import deque
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QFormLayout, QGroupBox,
                             QPushButton, QTabWidget, QDoubleSpinBox, QGridLayout)
from PyQt5.QtCore import Qt, QTimer, QLocale
import pyqtgraph as pg

class MotorDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real-time PMDC PI Control (Realistic Delay + Auto-Tune + Load Ramp)")
        self.resize(1220, 850)

        # ----------------------------------------------------
        # 1. พารามิเตอร์มอเตอร์ (Plant Parameters)
        # ----------------------------------------------------
        self.Ts = 0.0001      # ความละเอียดการจำลอง (10 kHz)
        self.Vt_max = 6.0     # แรงดันอิ่มตัวสูงสุด [V]
        self.Ra = 8.0         # ความต้านทานอาร์เมเจอร์ [Ω]
        self.Kt = 1.1         # ค่าคงที่แรงบิด [N.m/A]
        self.Kb = 1.1         # ค่าคงที่ Back-EMF [V.s/rad]
        self.B = 0.005        # ความฝืด [N.m.s/rad]
        self.J = 0.01         # ความเฉื่อย [kg.m²]
        self.La = 0.001       # ความเหนี่ยวนำอาร์เมเจอร์ [H]

        # ----------------------------------------------------
        # 2. การจำลองความหน่วงของระบบ (Realistic Time Delay)
        # ----------------------------------------------------
        self.delay_time = 0.01
        self.delay_steps = int(self.delay_time / self.Ts) # 100 steps (10 ms)
        self.w_history = deque([0.0] * self.delay_steps, maxlen=self.delay_steps)

        # ตัวแปรสถานะทางฟิสิกส์
        self.w = 0.0
        self.ia = 0.0
        self.integral_e = 0.0
        self.time_val = 0.0

        # ตัวแปรเก็บค่ากระแสพีคและวิเคราะห์ระบบ
        self.max_positive_peak = 0.0
        self.max_negative_peak = 0.0
        self.current_overshoot = 0.0
        self.current_undershoot = 0.0
        self.max_recorded_speed = 0.0
        self.min_recorded_speed = float('inf')
        self.reached_target_once = False  # แฟล็กบอกว่าเคยไต่ถึงเป้าหมายหรือยัง เพื่อไม่ให้นับช่วงออกตัวเป็น Undershoot

        # ตัวแปรควบคุมเริ่มต้น
        self.target_rpm = 25.0
        self.target_load = 0.0       # โหลดเป้าหมายที่ตั้งจาก UI
        self.actual_load = 0.0       # โหลดจริงที่ค่อยๆ ไต่ระดับ (Ramp)
        self.load_ramp_time = 0.0    # เวลาที่ใช้ไต่ระดับโหลด (วินาที)
        self.kp = 0.5
        self.ki = 20.0

        # บัฟเฟอร์สำหรับวาดกราฟ
        self.buffer_size = 1500
        self.t_data = deque(maxlen=self.buffer_size)
        self.w_data = deque(maxlen=self.buffer_size)
        self.ia_data = deque(maxlen=self.buffer_size)
        self.target_data = deque(maxlen=self.buffer_size)

        self.init_ui()

        # รัน Auto-Tune อัตโนมัติครั้งแรกเมื่อเริ่มโปรแกรม
        self.run_auto_tune()

        # ----------------------------------------------------
        # 3. ตั้งค่า Timer สำหรับ Game Loop (อัปเดตหน้าจอทุก 16 ms)
        # ----------------------------------------------------
        self.timer = QTimer()
        self.timer.timeout.connect(self.physics_loop)
        self.timer.start(16) # ~60 FPS

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)

        main_widget.setObjectName("MainWidget")
        main_widget.setStyleSheet("""
            QWidget#MainWidget {
                background: qlineargradient(
                    x1: 0, y1: 0,
                    x2: 1, y2: 1,
                    stop: 0 #FFFFFF,
                    stop: 0.5 #DDEEDD,
                    stop: 1 #2E7D32
                );
            }
        """)

        layout = QHBoxLayout(main_widget)

        def create_spinbox(val, min_v, max_v, step, decimals, callback):
            sb = QDoubleSpinBox()
            sb.setRange(min_v, max_v)
            sb.setSingleStep(step)
            sb.setDecimals(decimals)
            sb.setValue(val)
            sb.valueChanged.connect(callback)
            sb.setStyleSheet("font-size: 14px; font-weight: bold; color: #2c3e50;")
            return sb

        # ====================================================
        # ฝั่งซ้าย: แผงควบคุม (Control Panel)
        # ====================================================
        left_panel = QWidget()
        left_panel.setFixedWidth(395)
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)

        main_controls_group = QGroupBox("Main Controls")
        main_controls_group.setStyleSheet("""
            QGroupBox { 
                background: white;
                border: 1px solid #d0d0d0;
                border-radius: 8px;
                margin-top: 5px;
                padding-top: 10px;
                font-weight: bold;
                font-size: 16px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)
        main_layout = QVBoxLayout()

        # ปุ่ม Start / Pause
        self.btn_toggle = QPushButton("Pause Simulation ▐▐")
        self.btn_toggle.setStyleSheet("background-color: lightcoral; font-weight: bold; padding: 10px; font-size: 14px; border-radius: 5px;")
        self.btn_toggle.clicked.connect(self.toggle_simulation)
        main_layout.addWidget(self.btn_toggle)

        # ฟอร์มตั้งค่า Target, Load และ Load Ramp Time
        form_ctrl = QFormLayout()
        self.sb_rpm = create_spinbox(self.target_rpm, 0.0, 99999.0, 5.0, 3, self.update_target)
        self.sb_load = create_spinbox(self.target_load, 0.0, 99999.0, 0.01, 3, self.update_load)
        self.sb_load_ramp = create_spinbox(self.load_ramp_time, 0.0, 60.0, 0.5, 3, self.update_load_ramp)

        self.lbl_max_load = QLabel("Max Load: -- N.m")
        self.lbl_max_rpm = QLabel("Max RPM: -- RPM")
        self.lbl_actual_load = QLabel("Actual Load: 0.000 N.m")
        self.lbl_max_load.setStyleSheet("color: #d35400; font-size: 11px; font-style: italic;")
        self.lbl_max_rpm.setStyleSheet("color: #d35400; font-size: 11px; font-style: italic;")
        self.lbl_actual_load.setStyleSheet("color: #2980b9; font-size: 11px; font-weight: bold;")

        form_ctrl.addRow("Target Speed (RPM):", self.sb_rpm)
        form_ctrl.addRow("", self.lbl_max_load)
        form_ctrl.addRow("Load Torque (N.m):", self.sb_load)
        form_ctrl.addRow("Load Ramp Time (s):", self.sb_load_ramp)
        form_ctrl.addRow("", self.lbl_actual_load)
        form_ctrl.addRow("", self.lbl_max_rpm)
        main_layout.addLayout(form_ctrl)

        # ----------------------------------------------------
        # ส่วนแสดงผล Real-time & Safety Monitor + Overshoot & Undershoot
        # ----------------------------------------------------
        monitor_box = QGroupBox("Real-time & Safety Monitor")
        monitor_box.setStyleSheet("font-weight: bold; font-size: 13px; color: #2c3e50; margin-top: 5px;")
        monitor_layout = QGridLayout()

        self.lbl_rt_speed = QLabel("Speed: 0.000 RPM")
        self.lbl_rt_current = QLabel("Current: 0.000 A")
        self.lbl_peak_pos = QLabel("Peak (+): 0.000 A")
        self.lbl_peak_neg = QLabel("Peak (-): 0.000 A")
        self.lbl_overshoot = QLabel("Overshoot: 0.000 %")
        self.lbl_undershoot = QLabel("Undershoot: 0.000 %")
        self.lbl_ss_error = QLabel("SS Error: 0.000 RPM")

        self.lbl_rt_speed.setStyleSheet("font-size: 13px; color: #0000CC; font-weight: bold;")
        self.lbl_rt_current.setStyleSheet("font-size: 13px; color: #008800; font-weight: bold;")
        self.lbl_peak_pos.setStyleSheet("font-size: 12px; color: #CC0000; font-weight: bold;")
        self.lbl_peak_neg.setStyleSheet("font-size: 12px; color: #880088; font-weight: bold;")
        self.lbl_overshoot.setStyleSheet("font-size: 12px; color: #D35400; font-weight: bold;")
        self.lbl_undershoot.setStyleSheet("font-size: 12px; color: #8E44AD; font-weight: bold;")
        self.lbl_ss_error.setStyleSheet("font-size: 12px; color: #27AE60; font-weight: bold;")

        self.btn_reset_peak = QPushButton("Reset Peak Stats")
        self.btn_reset_peak.setStyleSheet("background-color: #f39c12; color: white; font-weight: bold; font-size: 11px; padding: 6px; border-radius: 4px;")
        self.btn_reset_peak.clicked.connect(self.reset_peak_stats)

        monitor_layout.addWidget(self.lbl_rt_speed, 0, 0, 1, 2)
        monitor_layout.addWidget(self.lbl_rt_current, 1, 0, 1, 2)
        monitor_layout.addWidget(self.lbl_peak_pos, 2, 0, 1, 1)
        monitor_layout.addWidget(self.lbl_peak_neg, 2, 1, 1, 1)
        monitor_layout.addWidget(self.lbl_overshoot, 3, 0, 1, 1)
        monitor_layout.addWidget(self.lbl_undershoot, 3, 1, 1, 1)
        monitor_layout.addWidget(self.lbl_ss_error, 4, 0, 1, 2)
        monitor_layout.addWidget(self.btn_reset_peak, 5, 0, 1, 2)

        monitor_box.setLayout(monitor_layout)
        main_layout.addWidget(monitor_box)

        main_controls_group.setLayout(main_layout)
        left_layout.addWidget(main_controls_group)

        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
        QTabWidget { border: none; }
        QTabWidget::pane { background: white; border: 1px solid #d0d0d0; border-radius: 8px; }
        QTabBar { border: none; }
        QTabBar::tab { background: #808080; color: white; padding: 8px 15px; border: 1px solid #d0d0d0; border-radius: 8px 8px 0px 0px; }
        QTabBar::tab:selected { background: white; color: black; font-weight: bold; border-bottom-color: white; }
        QTabBar::tab:hover { background: #d0d0d0; }
        """)
        
        # [Tab 1] PI Tuning
        tab_pi = QWidget()
        layout_pi = QFormLayout(tab_pi)
        
        self.sb_kp = create_spinbox(self.kp, 0.0, 99999.0, 0.1, 3, self.update_kp)
        self.sb_ki = create_spinbox(self.ki, 0.0, 99999.0, 1.0, 3, self.update_ki)
        
        layout_pi.addRow("Kp (Proportional):", self.sb_kp)
        layout_pi.addRow("Ki (Integral):", self.sb_ki)
        
        # [Tab 2] Motor Constants
        tab_motor = QWidget()
        layout_motor = QFormLayout(tab_motor)

        self.sb_vt = create_spinbox(self.Vt_max, 1.0, 9999.0, 1.0, 3, self.update_motor_params)
        self.sb_ra = create_spinbox(self.Ra, 0.01, 9999.0, 0.1, 3, self.update_motor_params)
        self.sb_kt = create_spinbox(self.Kt, 0.001, 9999.0, 0.1, 4, self.update_motor_params)
        self.sb_kb = create_spinbox(self.Kb, 0.001, 9999.0, 0.1, 4, self.update_motor_params)
        self.sb_b = create_spinbox(self.B, 0.0001, 9999.0, 0.001, 5, self.update_motor_params)
        self.sb_j = create_spinbox(self.J, 0.0001, 9999.0, 0.01, 5, self.update_motor_params)
        self.sb_la = create_spinbox(self.La, 0.0001, 9999.0, 0.001, 5, self.update_motor_params)

        layout_motor.addRow("Max Voltage (Vt_max) [V]:", self.sb_vt)
        layout_motor.addRow("Armature Res (Ra) [Ω]:", self.sb_ra)
        layout_motor.addRow("Torque Const (Kt) [N.m/A]:", self.sb_kt)
        layout_motor.addRow("Back-EMF Const (Kb) [V.s/rad]:", self.sb_kb)
        layout_motor.addRow("Friction (B) [N.m.s/rad]:", self.sb_b)
        layout_motor.addRow("Inertia (J) [kg.m²]:", self.sb_j)
        layout_motor.addRow("Armature Ind (La) [H]:", self.sb_la)

        self.tabs.addTab(tab_pi, "PI Tuning (Auto)")
        self.tabs.addTab(tab_motor, "Motor Constants")
        
        left_layout.addWidget(self.tabs)
        layout.addWidget(left_panel)

        # ====================================================
        # ฝั่งขวา: กราฟแสดงผล (Plots)
        # ====================================================
        graph_layout = QVBoxLayout()
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        self.plot_speed = pg.PlotWidget(title="Motor Speed (RPM)")
        self.plot_speed.enableAutoRange(axis=pg.ViewBox.YAxis)
        self.plot_speed.showGrid(x=True, y=True, alpha=0.3)
        self.curve_speed = self.plot_speed.plot(pen=pg.mkPen('b', width=2), name="Actual Speed")
        self.curve_target = self.plot_speed.plot(pen=pg.mkPen('r', width=2, style=Qt.DashLine), name="Target")
        
        self.plot_current = pg.PlotWidget(title="Armature Current (A)")
        self.plot_current.enableAutoRange(axis=pg.ViewBox.YAxis)
        self.plot_current.showGrid(x=True, y=True, alpha=0.3)
        self.plot_current.setXLink(self.plot_speed)
        self.curve_current = self.plot_current.plot(pen=pg.mkPen('g', width=2))

        graph_layout.addWidget(self.plot_speed)
        graph_layout.addWidget(self.plot_current)
        layout.addLayout(graph_layout)

        self.apply_three_decimals_to_axes()
        self.refresh_dynamic_limits()

    def apply_three_decimals_to_axes(self):
        def custom_tick_strings(values, scale, spacing):
            return [f"{v:.3f}" for v in values]

        for p in [self.plot_speed, self.plot_current]:
            for pos in ['left', 'bottom']:
                axis = p.getAxis(pos)
                axis.tickStrings = custom_tick_strings

    # ====================================================
    # ศูนย์รวมสมการฟิสิกส์
    # ====================================================
    def motor_physics_step(self, Vt, current_w, current_ia, load):
        dia_dt = (Vt - self.Ra * current_ia - self.Kb * current_w) / self.La
        next_ia = current_ia + dia_dt * self.Ts
        
        Te = self.Kt * next_ia
        
        if current_w <= 0.0 and Te < load:
            dw_dt = 0.0
            next_w = 0.0
        else:
            dw_dt = (Te - self.B * current_w - load) / self.J
            next_w = current_w + dw_dt * self.Ts
            if next_w < 0.0:
                next_w = 0.0
                
        return next_w, next_ia

    def calc_max_load_at_rpm(self, rpm):
        w = rpm * (2 * np.pi) / 60.0
        ia_max = (self.Vt_max - self.Kb * w) / self.Ra
        te_max = self.Kt * ia_max
        max_load = te_max - (self.B * w)
        return max(0.0, max_load)

    def calc_max_rpm_at_load(self, load):
        numerator = (self.Kt * self.Vt_max) - (load * self.Ra)
        denominator = (self.Kt * self.Kb) + (self.B * self.Ra)
        w_max = numerator / denominator
        rpm_max = w_max * 60.0 / (2 * np.pi)
        return max(0.0, rpm_max)

    def refresh_dynamic_limits(self):
        max_load = self.calc_max_load_at_rpm(self.target_rpm)
        max_rpm = self.calc_max_rpm_at_load(self.actual_load)
        
        self.lbl_max_load.setText(f"Max Load capacity at {self.target_rpm:.3f} RPM: {max_load:.3f} N.m")
        self.lbl_max_rpm.setText(f"Max RPM capacity at {self.actual_load:.3f} N.m: {max_rpm:.3f} RPM")
        self.lbl_actual_load.setText(f"Actual Load: {self.actual_load:.3f} N.m")

    # ====================================================
    # ฟังก์ชัน Relay Feedback Auto-Tuning (Z-N ดั้งเดิม)
    # ====================================================
    def run_auto_tune(self):
        tune_target_rpm = self.target_rpm if self.target_rpm > 5.0 else 25.0
        target_rad = tune_target_rpm * (2 * np.pi) / 60.0
        
        v_ss = (self.Kb * target_rad) + ((self.B * target_rad) / self.Kt) * self.Ra
        v_ss = np.clip(v_ss, 0.0, self.Vt_max * 0.7)
        
        d = min(self.Vt_max - v_ss, v_ss + self.Vt_max) * 0.8
        if d <= 0.1:
            d = self.Vt_max * 0.5

        tune_w = target_rad
        tune_ia = (self.B * target_rad) / self.Kt
        tune_time = 0.0
        
        tune_w_history = deque([tune_w] * self.delay_steps, maxlen=self.delay_steps)
        crossings = 0
        
        last_error = 0.0
        start_time = 0.0
        max_w = -float('inf')
        min_w = float('inf')
        tc = 0.0

        for _ in range(200000): 
            delayed_w = tune_w_history[0]
            error = target_rad - delayed_w
            
            Vt = (v_ss + d) if error >= 0 else (v_ss - d)
            Vt = np.clip(Vt, -self.Vt_max, self.Vt_max)
                
            tune_w, tune_ia = self.motor_physics_step(Vt, tune_w, tune_ia, 0.0)
            tune_time += self.Ts
            tune_w_history.append(tune_w)
            
            if error * last_error < 0:
                crossings += 1
                if crossings == 4:
                    start_time = tune_time
                    max_w = tune_w
                    min_w = tune_w
                elif crossings == 6:
                    tc = tune_time - start_time
                    break
                    
            if crossings >= 4:
                if tune_w > max_w: max_w = tune_w
                if tune_w < min_w: min_w = tune_w
                
            last_error = error
            
        if tc > 0 and (max_w - min_w) > 0:
            a = (max_w - min_w) / 2.0
            Ku = (4.0 * d) / (np.pi * a) 
            
            # ใช้สูตร Modified Ziegler-Nichols 
            Kp_new = 0.35 * Ku
            Ki_new = (0.40 * Ku) / tc
            
            self.sb_kp.blockSignals(True)
            self.sb_ki.blockSignals(True)
            
            self.sb_kp.setValue(Kp_new)
            self.sb_ki.setValue(Ki_new)
            
            self.sb_kp.blockSignals(False)
            self.sb_ki.blockSignals(False)
            
            self.kp = Kp_new
            self.ki = Ki_new
            self.integral_e = 0.0  
            print(f"Auto-Tuned Success: tc={tc:.4f}s, Ku={Ku:.3f} -> Kp={Kp_new:.3f}, Ki={Ki_new:.3f}")

    # ====================================================
    # ฟังก์ชันโต้ตอบกับ UI 
    # ====================================================
    def toggle_simulation(self):
        if self.timer.isActive():
            self.timer.stop() 
            self.btn_toggle.setText("Play Simulation ►")
            self.btn_toggle.setStyleSheet("background-color: lightgreen; font-weight: bold; padding: 10px; font-size: 14px; border-radius: 5px;")
        else:
            self.timer.start(16) 
            self.btn_toggle.setText("Pause Simulation ▐▐")
            self.btn_toggle.setStyleSheet("background-color: lightcoral; font-weight: bold; padding: 10px; font-size: 14px; border-radius: 5px;")

    def update_target(self, val):
        self.target_rpm = val
        self.reset_peak_stats()
        self.refresh_dynamic_limits()

    def update_load(self, val):
        self.target_load = val
        if self.load_ramp_time <= 0.0:
            self.actual_load = val
        self.refresh_dynamic_limits()

    def update_load_ramp(self, val):
        self.load_ramp_time = val

    def update_kp(self, val):
        self.kp = val

    def update_ki(self, val):
        self.ki = val

    def update_motor_params(self):
        self.Vt_max = self.sb_vt.value()
        self.Ra = self.sb_ra.value()
        self.Kt = self.sb_kt.value()
        self.Kb = self.sb_kb.value()
        self.B = self.sb_b.value()
        self.J = self.sb_j.value()
        self.La = self.sb_la.value()
        
        self.refresh_dynamic_limits()
        self.run_auto_tune() 

    def reset_peak_stats(self):
        current_rpm = self.w * 60 / (2 * np.pi)
        self.max_positive_peak = max(0.0, self.ia)
        self.max_negative_peak = min(0.0, self.ia)
        self.max_recorded_speed = current_rpm
        self.min_recorded_speed = current_rpm
        self.current_overshoot = 0.0
        self.current_undershoot = 0.0
        self.reached_target_once = (current_rpm >= self.target_rpm * 0.98) if self.target_rpm > 0 else True
        
        self.lbl_peak_pos.setText(f"Peak (+): {self.max_positive_peak:.3f} A")
        self.lbl_peak_neg.setText(f"Peak (-): {self.max_negative_peak:.3f} A")
        self.lbl_overshoot.setText("Overshoot: 0.000 %")
        self.lbl_undershoot.setText("Undershoot: 0.000 %")

    # ====================================================
    # ลูปฟิสิกส์ (รันทุกๆ 16 ms)
    # ====================================================
    def physics_loop(self):
        steps_per_frame = 160 
        target_rad = self.target_rpm * (2 * np.pi) / 60.0

        for _ in range(steps_per_frame):
            # 0. ระบบ Load Ramp Rate (ค่อยๆ ไต่ระดับโหลดเข้าหาเป้าหมาย)
            if self.load_ramp_time > 0.0:
                max_load_step = (abs(self.target_load - self.actual_load) / self.load_ramp_time) * self.Ts
                if self.actual_load < self.target_load:
                    self.actual_load = min(self.target_load, self.actual_load + max_load_step)
                elif self.actual_load > self.target_load:
                    self.actual_load = max(self.target_load, self.actual_load - max_load_step)
            else:
                self.actual_load = self.target_load

            delayed_w = self.w_history[0]
            error = target_rad - delayed_w
            
            # 1. คำนวณแบบ Unsaturated
            p_term = self.kp * error
            new_integral = self.integral_e + (error * self.Ts)
            Vt_unsat = p_term + (self.ki * new_integral)
            
            # 2. Clamping Anti-Windup
            if Vt_unsat > self.Vt_max:
                Vt = self.Vt_max
                if error < 0:
                    self.integral_e = new_integral
            elif Vt_unsat < -self.Vt_max:
                Vt = -self.Vt_max
                if error > 0:
                    self.integral_e = new_integral
            else:
                Vt = Vt_unsat
                self.integral_e = new_integral

            # 3. อัปเดตสมการมอเตอร์ด้วย actual_load ที่ผ่าน Ramp แล้ว
            self.w, self.ia = self.motor_physics_step(Vt, self.w, self.ia, self.actual_load)
            self.w_history.append(self.w)
            self.time_val += self.Ts

            if self.ia > self.max_positive_peak:
                self.max_positive_peak = self.ia
            if self.ia < self.max_negative_peak:
                self.max_negative_peak = self.ia

        current_rpm = self.w * 60 / (2 * np.pi)

        # ตรวจสอบว่าระบบเคยไต่รอบถึงเป้าหมายแล้วหรือยัง เพื่อเริ่มจับค่า Undershoot
        if not self.reached_target_once and self.target_rpm > 0:
            if current_rpm >= self.target_rpm * 0.98:
                self.reached_target_once = True
                self.min_recorded_speed = current_rpm

        # บันทึก Peak Speed (สำหรับคำนวณ Overshoot)
        if current_rpm > self.max_recorded_speed:
            self.max_recorded_speed = current_rpm

        # บันทึก Min Speed (สำหรับคำนวณ Undershoot หลังเคยถึงเป้าหมายแล้ว)
        if self.reached_target_once and current_rpm < self.min_recorded_speed:
            self.min_recorded_speed = current_rpm

        # คำนวณ Overshoot & Undershoot (%)
        if self.target_rpm > 0:
            self.current_overshoot = max(0.0, ((self.max_recorded_speed - self.target_rpm) / self.target_rpm) * 100.0)
            if self.reached_target_once and self.min_recorded_speed != float('inf'):
                self.current_undershoot = max(0.0, ((self.target_rpm - self.min_recorded_speed) / self.target_rpm) * 100.0)
            else:
                self.current_undershoot = 0.0
        else:
            self.current_overshoot = 0.0
            self.current_undershoot = 0.0

        ss_error = self.target_rpm - current_rpm

        # แสดงผล Real-time
        self.lbl_rt_speed.setText(f"Speed: {current_rpm:.3f} RPM")
        self.lbl_rt_current.setText(f"Current: {self.ia:.3f} A")
        self.lbl_peak_pos.setText(f"Peak (+): {self.max_positive_peak:.3f} A")
        self.lbl_peak_neg.setText(f"Peak (-): {self.max_negative_peak:.3f} A")
        self.lbl_overshoot.setText(f"Overshoot: {self.current_overshoot:.3f} %")
        self.lbl_undershoot.setText(f"Undershoot: {self.current_undershoot:.3f} %")
        self.lbl_ss_error.setText(f"SS Error: {ss_error:.3f} RPM")
        self.lbl_actual_load.setText(f"Actual Load: {self.actual_load:.3f} N.m")

        self.t_data.append(self.time_val)
        self.w_data.append(current_rpm)
        self.ia_data.append(self.ia)
        self.target_data.append(self.target_rpm)

        self.curve_speed.setData(self.t_data, self.w_data)
        self.curve_target.setData(self.t_data, self.target_data)
        self.curve_current.setData(self.t_data, self.ia_data)

        window_size = 5.0 
        if self.time_val > window_size:
            self.plot_speed.setXRange(self.time_val - window_size, self.time_val, padding=0)
        else:
            self.plot_speed.setXRange(0, window_size, padding=0)

if __name__ == '__main__':
    QLocale.setDefault(QLocale(QLocale.English, QLocale.UnitedStates))
    
    if not QApplication.instance():
        app = QApplication(sys.argv)
    else:
        app = QApplication.instance()
        
    window = MotorDashboard()
    window.show()
    sys.exit(app.exec_())