import tkinter as tk
from tkinter import ttk
import ttkbootstrap as ttkb
import pyqtgraph as pg
from PySide6 import QtWidgets
import threading
import time
import serial
import qt_material
from queue import Queue
import sqlite3
from datetime import datetime


class SerialHandler:
    def __init__(self):
        self.serial_port = None
        self.running = False
        self.data_queue = Queue()
        self.last_temp = 0
        self.start_time = None

    def open_serial(self, port, baudrate):
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            print("串口打开成功")
        except Exception as e:
            print(f"串口打开失败: {e}")

    def close_serial(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        print("串口关闭成功")

    def start_reading(self):
        if self.serial_port and self.serial_port.is_open:
            self.running = True
            self.start_time = time.time()  # 记录开始时间
            self.read_thread = threading.Thread(target=self.read_serial_and_update_plot)
            self.read_thread.start()

    def read_serial_and_update_plot(self):
        buffer = bytearray()
        while self.running:
            if self.serial_port.in_waiting:
                data = self.serial_port.read(1)
                buffer += data
                if len(buffer) >= 6 and buffer[0] == 0x55 and buffer[-1] == 0xaa:
                    if buffer[1] == 0x00 and len(buffer) == 6:
                        temp = (buffer[3] << 8) | buffer[4]
                        self.last_temp = temp / 100.0
                        current_time = time.time() - self.start_time
                        self.data_queue.put((current_time, self.last_temp))
                    elif buffer[1] == 0x06 and len(buffer) == 6:
                        if buffer[2] == 0x00 and buffer[3] == 0x00 and buffer[4] == 0x00:
                            print("收到确认帧")
                        else:
                            kp = buffer[2] / 10.0
                            ki = buffer[3] / 100.0
                            kd = buffer[4] / 10.0
                            print(f"收到确认帧: Kp={kp}, Ki={ki}, Kd={kd}")
                    buffer = bytearray()
            time.sleep(0.01)

    def set_temperature(self, temp):
        if self.serial_port and self.serial_port.is_open:
            try:
                if 0 <= temp <= 19999:
                    high_byte = (temp >> 8) & 0xFF
                    low_byte = temp & 0xFF
                    command = bytes([0x55, 0x01, 0x00, high_byte, low_byte, 0xaa])
                    self.serial_port.write(command)
                    print(f"设定温度: {temp}, 命令: {command}")
                else:
                    print("温度超出范围")
            except ValueError:
                print("无效的温度值")

    def set_pid_parameters(self, kp, ki, kd):
        if self.serial_port and self.serial_port.is_open:
            try:
                kp_byte = int(kp * 10)
                ki_byte = int(ki * 100)
                kd_byte = int(kd * 10)
                command = bytes([0x55, 0x02, kp_byte, ki_byte, kd_byte, 0xaa])
                self.serial_port.write(command)
            except ValueError:
                print("无效的PID参数")


class PlotHandler:
    def __init__(self):
        self.plot_app = QtWidgets.QApplication.instance()
        if self.plot_app is None:
            self.plot_app = QtWidgets.QApplication([])

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setWindowTitle('温度曲线')
        self.plot_widget.setLabel('left', '温度', units='°C')
        self.plot_widget.setLabel('bottom', '时间', units='s')
        self.curve = self.plot_widget.plot(pen='y')
        # 确保启用鼠标交互
        self.plot_widget.setMouseEnabled(x=True, y=True)
        # 设置自动平移
        self.plot_widget.setAutoPan(x=True, y=False)
        self.plot_widget.show()
        qt_material.apply_stylesheet(self.plot_app, theme='dark_teal.xml')

    def update_plot(self, times, temps, current_time):
        self.curve.setData(times, temps)
        # 设置 x 轴范围，使其显示最近 20 秒的数据
        if current_time > 20:
            self.plot_widget.setXRange(current_time - 20, current_time)
        else:
            self.plot_widget.setXRange(0, 20)


class DatabaseHandler:
    def __init__(self, db_name="experiments.db"):
        self.conn = sqlite3.connect(db_name)
        self.create_tables()

    def create_tables(self):
        cursor = self.conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS experiments (
                id INTEGER PRIMARY KEY,
                name TEXT,
                start_time TEXT
            )
        ''')
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS data (
                experiment_id INTEGER,
                time REAL,
                temperature REAL,
                FOREIGN KEY(experiment_id) REFERENCES experiments(id)
            )
        ''')
        self.conn.commit()

    def add_experiment(self, name):
        cursor = self.conn.cursor()
        cursor.execute("INSERT INTO experiments (name, start_time) VALUES (?, ?)", (name, datetime.now().isoformat()))
        self.conn.commit()
        return cursor.lastrowid

    def add_data(self, experiment_id, time, temperature):
        cursor = self.conn.cursor()
        cursor.execute("INSERT INTO data (experiment_id, time, temperature) VALUES (?, ?, ?)", (experiment_id, time, temperature))
        self.conn.commit()

    def get_experiments(self):
        cursor = self.conn.cursor()
        cursor.execute("SELECT id, name, start_time FROM experiments")
        return cursor.fetchall()

    def get_data(self, experiment_id):
        cursor = self.conn.cursor()
        cursor.execute("SELECT time, temperature FROM data WHERE experiment_id = ?", (experiment_id,))
        return cursor.fetchall()


class SerialApp:
    def __init__(self, root):
        self.root = root
        self.root.title("温度监控系统")

        self.serial_handler = SerialHandler()
        self.plot_handler = PlotHandler()
        self.db_handler = DatabaseHandler()

        self.port_var = tk.StringVar(value="COM1")
        self.baudrate_var = tk.StringVar(value="19200")

        self.available_ports = ["COM1", "COM2", "COM3", "COM4", "COM5"]
        self.available_baudrates = ["9600", "14400", "19200", "38400", "57600", "115200"]

        self.create_widgets()

        self.times = []
        self.temps = []
        self.current_experiment_id = None

        self.root.after(100, self.update_plot)

    def create_widgets(self):
        frame = tk.Frame(self.root)
        frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        control_frame = tk.Frame(self.root)
        control_frame.pack(side=tk.RIGHT, fill=tk.Y)

        tk.Label(control_frame, text="串口号:").grid(row=0, column=0, sticky='e', padx=5, pady=5)
        self.port_combobox = ttk.Combobox(control_frame, textvariable=self.port_var, values=self.available_ports)
        self.port_combobox.grid(row=0, column=1, sticky='w', padx=5, pady=5)

        tk.Label(control_frame, text="波特率:").grid(row=0, column=2, sticky='e', padx=5, pady=5)
        self.baudrate_combobox = ttk.Combobox(control_frame, textvariable=self.baudrate_var, values=self.available_baudrates)
        self.baudrate_combobox.grid(row=0, column=3, sticky='w', padx=5, pady=5)

        self.start_button = ttkb.Button(control_frame, text="开始实验", command=self.start_experiment, bootstyle="success")
        self.start_button.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky='ew')

        self.stop_button = ttkb.Button(control_frame, text="结束实验", command=self.stop_experiment, state=tk.DISABLED, bootstyle="danger")
        self.stop_button.grid(row=1, column=2, columnspan=2, padx=5, pady=5, sticky='ew')

        self.current_temp_label = ttkb.Label(control_frame, text="当前温度: 0.0", bootstyle="inverse")
        self.current_temp_label.grid(row=3, column=0, columnspan=4, padx=5, pady=5, sticky='ew')

        tree_frame = tk.Frame(control_frame)
        tree_frame.grid(row=4, column=0, columnspan=4, padx=5, pady=5, sticky='nsew')
        tree_scrollbar = tk.Scrollbar(tree_frame, orient="vertical")
        self.tree = ttk.Treeview(tree_frame, columns=("time", "temperature"), show='headings', yscrollcommand=tree_scrollbar.set)
        tree_scrollbar.config(command=self.tree.yview)
        tree_scrollbar.pack(side="right", fill="y")
        self.tree.heading("time", text="时间")
        self.tree.heading("temperature", text="温度")
        self.tree.pack(fill="both", expand=True)

        tk.Label(control_frame, text="设定温度:").grid(row=5, column=0, sticky='e', padx=5, pady=5)
        self.set_temp_var = tk.StringVar()
        self.set_temp_entry = ttkb.Entry(control_frame, textvariable=self.set_temp_var)
        self.set_temp_entry.grid(row=5, column=1, sticky='w', padx=5, pady=5)
        self.set_temp_button = ttkb.Button(control_frame, text="设定", command=self.set_temperature, bootstyle="primary")
        self.set_temp_button.grid(row=5, column=2, columnspan=2, padx=5, pady=5, sticky='ew')

        tk.Label(control_frame, text="Kp:").grid(row=6, column=0, sticky='e', padx=5, pady=5)
        self.kp_var = tk.StringVar(value="0.5")
        self.kp_entry = ttkb.Entry(control_frame, textvariable=self.kp_var)
        self.kp_entry.grid(row=6, column=1, sticky='w', padx=5, pady=5)

        tk.Label(control_frame, text="Ki:").grid(row=7, column=0, sticky='e', padx=5, pady=5)
        self.ki_var = tk.StringVar(value="0.01")
        self.ki_entry = ttkb.Entry(control_frame, textvariable=self.ki_var)
        self.ki_entry.grid(row=7, column=1, sticky='w', padx=5, pady=5)

        tk.Label(control_frame, text="Kd:").grid(row=8, column=0, sticky='e', padx=5, pady=5)
        self.kd_var = tk.StringVar(value="0.1")
        self.kd_entry = ttkb.Entry(control_frame, textvariable=self.kd_var)
        self.kd_entry.grid(row=8, column=1, sticky='w', padx=5, pady=5)

        self.set_pid_button = ttkb.Button(control_frame, text="设定PID参数", command=self.set_pid_parameters, bootstyle="primary")
        self.set_pid_button.grid(row=9, column=0, columnspan=2, padx=5, pady=5, sticky='ew')

        tk.Label(control_frame, text="选择实验:").grid(row=10, column=0, sticky='e', padx=5, pady=5)
        self.experiment_var = tk.StringVar()
        self.experiment_combobox = ttk.Combobox(control_frame, textvariable=self.experiment_var)
        self.experiment_combobox.grid(row=10, column=1, columnspan=3, sticky='ew', padx=5, pady=5)
        self.experiment_combobox.bind("<<ComboboxSelected>>", self.load_experiment_data)

        self.load_experiments()

    def load_experiments(self):
        experiments = self.db_handler.get_experiments()
        self.experiment_combobox['values'] = [f"{exp[1]} ({exp[2]})" for exp in experiments]
        self.experiment_map = {f"{exp[1]} ({exp[2]})": exp[0] for exp in experiments}

    def start_experiment(self):
        self.current_experiment_id = self.db_handler.add_experiment("实验 " + time.strftime("%Y-%m-%d %H:%M:%S"))
        self.open_serial()
        self.start_reading()

    def stop_experiment(self):
        self.close_serial()
        self.stop_reading()
        self.current_experiment_id = None

    def open_serial(self):
        port = self.port_var.get()
        baudrate = self.baudrate_var.get()
        self.serial_handler.open_serial(port, baudrate)
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)

    def close_serial(self):
        self.serial_handler.close_serial()
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

    def start_reading(self):
        self.serial_handler.start_reading()
        self.stop_button.config(state=tk.NORMAL)
        self.start_button.config(state=tk.DISABLED)

    def stop_reading(self):
        self.serial_handler.running = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

    def set_temperature(self):
        try:
            temp = int(self.set_temp_var.get())
            self.serial_handler.set_temperature(temp)
        except ValueError:
            print("无效的温度值")

    def set_pid_parameters(self):
        try:
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            kd = float(self.kd_var.get())
            self.serial_handler.set_pid_parameters(kp, ki, kd)
        except ValueError:
            print("无效的PID参数")

    def update_plot(self):
        while not self.serial_handler.data_queue.empty():
            current_time, temp = self.serial_handler.data_queue.get()
            self.times.append(current_time)
            self.temps.append(temp)
            self.plot_handler.update_plot(self.times, self.temps, current_time)
            self.current_temp_label.config(text=f"当前温度: {temp:.2f}°C")
            self.update_table(current_time, temp)
            if self.current_experiment_id is not None:
                self.db_handler.add_data(self.current_experiment_id, current_time, temp)
        self.root.after(100, self.update_plot)

    def update_table(self, current_time, temp):
        time_str = time.strftime("%H:%M:%S", time.localtime())
        self.tree.insert("", "end", values=(time_str, f"{temp:.2f}"))
        self.tree.yview_moveto(1)

    def load_experiment_data(self, event):
        selected_experiment = self.experiment_var.get()
        experiment_id = self.experiment_map[selected_experiment]
        data = self.db_handler.get_data(experiment_id)
        self.times = [d[0] for d in data]
        self.temps = [d[1] for d in data]
        self.plot_handler.update_plot(self.times, self.temps, self.times[-1] if self.times else 0)

    def on_closing(self):
        self.serial_handler.running = False
        if self.serial_handler.serial_port and self.serial_handler.serial_port.is_open:
            self.serial_handler.serial_port.close()
        self.root.quit()
        self.root.destroy()
        self.plot_handler.plot_widget.close()
        self.plot_handler.plot_app.quit()


if __name__ == "__main__":
    root = ttkb.Window(themename="darkly")
    app = SerialApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
