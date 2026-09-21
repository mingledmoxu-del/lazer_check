import sys, time
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox, QSpinBox, QPlainTextEdit, QCheckBox, QGridLayout, QGroupBox
from serial import Serial, SerialException
from serial.tools import list_ports
from protocol import command, parse_line
from sim_device import SimDevice

class Window(QWidget):
    def __init__(self):
        super().__init__(); self.setWindowTitle("P100S 伺服监控与调试"); self.resize(980, 680)
        self.setStyleSheet("""
        QWidget { background:#f4f6f8; color:#24313d; font-size:13px; }
        QLabel#title { font-size:22px; font-weight:700; color:#172b4d; padding:4px 0 8px; }
        QLabel#status { background:#e8f1fb; border:1px solid #c8d8ea; border-radius:6px; padding:9px 12px; color:#24415f; }
        QGroupBox { background:white; border:1px solid #d9e1e8; border-radius:8px; margin-top:12px; padding:10px; font-weight:600; }
        QGroupBox::title { subcontrol-origin:margin; left:12px; padding:0 5px; color:#35658f; }
        QLabel.metricTitle { color:#6b7785; font-size:12px; font-weight:600; }
        QLabel.metricValue { color:#172b4d; font-size:19px; font-weight:700; padding:5px 0 8px; }
        QPushButton { background:#2867a5; color:white; border:0; border-radius:5px; padding:7px 12px; }
        QPushButton:hover { background:#1e568c; } QPushButton:disabled { background:#b9c4ce; }
        QComboBox,QSpinBox { background:white; border:1px solid #c5d0db; border-radius:4px; padding:5px; }
        QPlainTextEdit { background:#17212b; color:#d9e7f5; border-radius:6px; font-family:Consolas; }
        """)
        self.port=None; self.buf=b""; self.rid=0; self.connected=False; self.last_state=0; self.sim=None
        root=QVBoxLayout(self); top=QHBoxLayout(); self.ports=QComboBox(); self.refresh=QPushButton("刷新"); self.connect_btn=QPushButton("连接"); self.sim_btn=QPushButton("模拟设备"); top.addWidget(QLabel("串口")); top.addWidget(self.ports); top.addWidget(self.refresh); top.addWidget(self.connect_btn); top.addWidget(self.sim_btn); root.addLayout(top)
        title=QLabel("P100S 伺服监控与调试"); title.setObjectName("title"); root.addWidget(title)
        self.status=QLabel("串口：未连接 | STM32：状态未知 | 驱动器：状态未知 | 软件：安全锁定"); self.status.setObjectName("status"); root.addWidget(self.status)
        row=QHBoxLayout(); self.speed=QSpinBox(); self.speed.setRange(-100,100); self.speed.setValue(50); row.addWidget(QLabel("速度 rpm")); row.addWidget(self.speed)
        self.buttons={n:QPushButton(n) for n in ("参数检查","零速使能","正转试转","反转试转","停止","解除使能")}
        for n,b in self.buttons.items(): row.addWidget(b); b.clicked.connect(lambda _, x=n:self.action(x))
        root.addLayout(row); self.detail=QLabel("参数检查：未执行"); root.addWidget(self.detail)
        group=QGroupBox("实时状态"); grid=QGridLayout(group); grid.setHorizontalSpacing(28); self.metric_labels={}
        for i,(key,title) in enumerate((("rpm","速度 (rpm)"),("position","当前位置 (脉冲)"),("command_position","指令位置 (脉冲)"),("position_error","位置偏差 (脉冲)"),("inputs","输入端子"),("outputs","输出端子"),("alarm","报警码"),("drive","运行状态"))):
            label=QLabel(title); label.setProperty("class","metricTitle"); label.setStyleSheet("color:#6b7785;font-size:12px;font-weight:600;")
            value=QLabel("--"); value.setMinimumWidth(175); value.setStyleSheet("color:#172b4d;font-size:19px;font-weight:700;padding:5px 0 8px;"); self.metric_labels[key]=value; grid.addWidget(label,i//4*2,i%4); grid.addWidget(value,i//4*2+1,i%4)
        root.addWidget(group); self.log=QPlainTextEdit(); self.log.setReadOnly(True); self.log.setVisible(False); root.addWidget(self.log)
        log_row=QHBoxLayout(); self.verbose=QCheckBox("显示通信日志"); self.verbose.toggled.connect(self.log.setVisible); clear=QPushButton("清空日志"); clear.clicked.connect(self.log.clear); log_row.addWidget(self.verbose); log_row.addWidget(clear); log_row.addStretch(); root.addLayout(log_row)
        self.refresh.clicked.connect(self.refresh_ports); self.connect_btn.clicked.connect(self.toggle); self.sim_btn.clicked.connect(self.toggle_sim); self.refresh_ports(); self.update_buttons()
        self.timer=QTimer(self); self.timer.timeout.connect(self.expire); self.timer.timeout.connect(self.read); self.timer.start(250)
    def refresh_ports(self): self.ports.clear(); self.ports.addItems([p.device for p in list_ports.comports()])
    def toggle(self):
        if self.port and self.port.is_open: self.port.close(); self.connected=False; self.connect_btn.setText("连接")
        else:
            try:
                self.port=Serial(self.ports.currentText(), 115200, timeout=0); self.connected=True; self.connect_btn.setText("断开")
            except SerialException as e:
                self.log_line("串口打开失败: "+str(e)); self.connected=False
        self.log_line("串口" + ("已连接" if self.connected else "已断开")); self.update_buttons()
    def toggle_sim(self):
        if self.sim: self.sim.close(); self.sim=None; self.sim_btn.setText("模拟设备")
        else: self.sim=SimDevice(); self.sim.line.connect(self.handle); self.sim_btn.setText("关闭模拟")
        self.update_buttons()
    def action(self,n):
        if n=="停止": self.send("STOP")
        elif n=="参数检查": self.send("CHECK")
        elif n=="零速使能": self.send("ENABLE")
        elif n=="解除使能": self.send("DISABLE")
        elif n in ("正转试转","反转试转"):
            v=abs(self.speed.value())*(1 if n.startswith("正") else -1); self.send("RUN",rpm=v)
    def send(self,name,**fields):
        if not (self.connected or self.sim): return
        self.rid=(self.rid+1)&0x7fffffff; s=command(self.rid,name,**fields); self.log_line("> "+s.strip())
        if self.sim: self.sim.send(s)
        else:
            try: self.port.write(s.encode("ascii"))
            except (SerialException, OSError) as e:
                self.log_line("串口发送失败：" + str(e)); self.connected=False; self.update_buttons()
    def read(self):
        if not self.port or not self.port.is_open: return
        try:
            waiting = self.port.in_waiting
            if waiting:
                self.buf += self.port.read(waiting)
                self.consume()
        except (SerialException, OSError) as e:
            # Windows may invalidate the handle after unplug/close between
            # is_open and ClearCommError. Treat it as a disconnected port.
            self.log_line("串口通信中断：" + str(e))
            try: self.port.close()
            except (SerialException, OSError): pass
            self.connected = False
            self.status.setText("串口：已断开 | STM32：状态未知 | 驱动器：状态未知 | 软件：状态未知")
            self.update_buttons()
    def consume(self):
        while b"\n" in self.buf:
            raw,self.buf=self.buf.split(b"\n",1)
            try: self.handle(raw.decode("ascii",errors="replace"))
            except Exception as e: self.log_line("解析错误: "+str(e))
    def handle(self,line):
        r=parse_line(line)
        if not r:
            if self.verbose.isChecked() and line.strip(): self.log_line(line.strip())
            return
        # 周期状态只更新状态栏；默认不把 500 ms 心跳写入日志。
        if self.verbose.isChecked(): self.log_line("< "+line.strip())
        self.last_state=time.monotonic(); f=r.fields
        if r.kind=="R" and r.status!="OK": self.detail.setText("命令失败："+str(f));
        if "failed" in f: self.detail.setText(f"参数检查：失败地址 {f.get('failed')} 实际 {f.get('actual')} 要求 {f.get('expected')}")
        if r.kind=="S" and r.status=="OK":
            drive={'0':'主电路未充电','1':'已充电未使能','2':'报警','3':'已使能/运行'}.get(f.get('drive'),'未知')
            for key in ("rpm","position","command_position","position_error","inputs","outputs","alarm"): self.metric_labels[key].setText(f.get(key,"--"))
            self.metric_labels["drive"].setText(drive)
        drive={'0':'未上电','1':'未使能','2':'报警','3':'已使能/运行'}.get(f.get('drive'),'未知')
        self.status.setText(f"串口：{'已连接' if (self.connected or self.sim) else '未连接'} | STM32：在线 | 驱动器：{drive} | 软件状态：{f.get('state','未知')} | 更新：{time.strftime('%H:%M:%S')}"); self.update_buttons()
    def expire(self):
        if self.last_state and time.monotonic()-self.last_state>1.5: self.status.setText("串口：已连接 | STM32：状态未知 | 驱动器：状态未知 | 软件：状态未知"); self.update_buttons()
    def update_buttons(self):
        known=bool(self.last_state and time.monotonic()-self.last_state<=1.5); online=bool(self.connected or self.sim)
        for n,b in self.buttons.items(): b.setEnabled(online and known and (n in ("停止","参数检查","零速使能","解除使能") or n in ("正转试转","反转试转") and known))
        self.buttons["停止"].setEnabled(online)
    def log_line(self,s): self.log.appendPlainText(time.strftime("[%H:%M:%S] ")+s)
    def closeEvent(self,e):
        if self.sim: self.sim.close()
        if self.port and self.port.is_open: self.send("STOP"); self.port.close()
        e.accept()
app=QApplication(sys.argv); w=Window(); w.show(); sys.exit(app.exec())
