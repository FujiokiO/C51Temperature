import tkinter as tk
from tkinter import ttk
import pyqtgraph as pg
from PySide6 import QtWidgets, QtCore
import threading
import time
import csv
import serial
import qt_material
from queue import Queue


class SerialApp:
    def __init__(self, root):
        self.root = root
        self.root.title("温度监控系统")

        self.serial_port = None
        self.running = False

        # Serial settings
        self.port_var = tk.StringVar(value="COM1")
        self.baudrate_var = tk.StringVar(value="19200")

        # Available options
        self.available_ports = ["COM1", "COM2", "COM3", "COM4", "COM5"]
        self.available_baudrates = ["9600", "14400", "19200", "38400", "57600", "115200"]

        # Create UI elements
        self.create_widgets()

        # Data storage for plotting
        self.times = []
        self.temps = []
        self.start_time = None
        self.last_temp = 0
        self.plot_widget = None
        self.curve = None
        self.create_plot_window()

        # Apply qt_material theme
        self.apply_qt_material_theme()

        # Queue for incoming data
        self.data_queue = Queue()

        # Start timer for updating the plot
        self.root.after(100, self.update_plot)

    def apply_qt_material_theme(self):
        qt_material.apply_stylesheet(self.plot_app, theme='dark_teal.xml')

    def create_plot_window(self):
        self.plot_app = QtWidgets.QApplication.instance()
        if self.plot_app is None:
            self.plot_app = QtWidgets.QApplication([])

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setWindowTitle('温度曲线')
        self.plot_widget.setLabel('left', '温度', units='°C')
        self.plot_widget.setLabel('bottom', '时间', units='s')
        self.curve = self.plot_widget.plot(pen='y')
        self.plot_widget.show()

    def create_widgets(self):
        frame = tk.Frame(self.root)
        frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        control_frame = tk.Frame(self.root)
        control_frame.pack(side=tk.RIGHT, fill=tk.Y)

        tk.Label(control_frame, text="串口号:").grid(row=0, column=0)
        self.port_combobox = ttk.Combobox(control_frame, textvariable=self.port_var, values=self.available_ports)
        self.port_combobox.grid(row=0, column=1)

        tk.Label(control_frame, text="波特率:").grid(row=1, column=0)
        self.baudrate_combobox = ttk.Combobox(control_frame, textvariable=self.baudrate_var,
                                              values=self.available_baudrates)
        self.baudrate_combobox.grid(row=1, column=1)

        self.open_button = tk.Button(control_frame, text="打开串口", command=self.open_serial)
        self.open_button.grid(row=2, column=0, columnspan=2)

        self.close_button = tk.Button(control_frame, text="关闭串口", command=self.close_serial, state=tk.DISABLED)
        self.close_button.grid(row=3, column=0, columnspan=2)

        self.start_button = tk.Button(control_frame, text="开始", command=self.start_reading)
        self.start_button.grid(row=4, column=0, columnspan=2)

        self.stop_button = tk.Button(control_frame, text="暂停", command=self.stop_reading, state=tk.DISABLED)
        self.stop_button.grid(row=5, column=0, columnspan=2)

        self.save_button = tk.Button(control_frame, text="保存数据", command=self.save_data)
        self.save_button.grid(row=6, column=0, columnspan=2)

        self.current_temp_label = tk.Label(control_frame, text="当前温度: 0.0")
        self.current_temp_label.grid(row=7, column=0, columnspan=2)

        # Add a Treeview with a vertical scrollbar
        tree_frame = tk.Frame(control_frame)
        tree_frame.grid(row=8, column=0, columnspan=2, sticky='nsew')
        tree_scrollbar = tk.Scrollbar(tree_frame, orient="vertical")
        self.tree = ttk.Treeview(tree_frame, columns=("time", "temperature"), show='headings',
                                 yscrollcommand=tree_scrollbar.set)
        tree_scrollbar.config(command=self.tree.yview)
        tree_scrollbar.pack(side="right", fill="y")
        self.tree.heading("time", text="时间")
        self.tree.heading("temperature", text="温度")
        self.tree.pack(fill="both", expand=True)

        tk.Label(control_frame, text="设定温度:").grid(row=9, column=0)
        self.set_temp_var = tk.StringVar()
        self.set_temp_entry = tk.Entry(control_frame, textvariable=self.set_temp_var)
        self.set_temp_entry.grid(row=9, column=1)
        self.set_temp_button = tk.Button(control_frame, text="设定", command=self.set_temperature)
        self.set_temp_button.grid(row=10, column=0, columnspan=2)

        self.request_temp_button = tk.Button(control_frame, text="读取温度", command=self.request_temperature)
        self.request_temp_button.grid(row=11, column=0, columnspan=2)

        tk.Label(control_frame, text="Kp:").grid(row=12, column=0)
        self.kp_var = tk.StringVar(value="0.5")
        self.kp_entry = tk.Entry(control_frame, textvariable=self.kp_var)
        self.kp_entry.grid(row=12, column=1)

        tk.Label(control_frame, text="Ki:").grid(row=13, column=0)
        self.ki_var = tk.StringVar(value="0.01")
        self.ki_entry = tk.Entry(control_frame, textvariable=self.ki_var)
        self.ki_entry.grid(row=13, column=1)

        tk.Label(control_frame, text="Kd:").grid(row=14, column=0)
        self.kd_var = tk.StringVar(value="0.1")
        self.kd_entry = tk.Entry(control_frame, textvariable=self.kd_var)
        self.kd_entry.grid(row=14, column=1)

        self.set_pid_button = tk.Button(control_frame, text="设定PID参数", command=self.set_pid_parameters)
        self.set_pid_button.grid(row=15, column=0, columnspan=2)

    def open_serial(self):
        port = self.port_var.get()
        baudrate = self.baudrate_var.get()
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.open_button.config(state=tk.DISABLED)
            self.close_button.config(state=tk.NORMAL)
            self.start_button.config(state=tk.NORMAL)
            print("串口打开成功")
        except Exception as e:
            print(f"串口打开失败: {e}")

    def close_serial(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.open_button.config(state=tk.NORMAL)
        self.close_button.config(state=tk.DISABLED)
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.DISABLED)
        print("串口关闭成功")

    def start_reading(self):
        if self.serial_port and self.serial_port.is_open:
            self.running = True
            self.stop_button.config(state=tk.NORMAL)
            self.start_button.config(state=tk.DISABLED)
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
                        # 解析单片机发送的当前温度
                        temp = (buffer[3] << 8) | buffer[4]
                        self.last_temp = temp / 100.0
                        current_time = time.time() - self.start_time
                        self.data_queue.put((current_time, self.last_temp))
                    elif buffer[1] == 0x06 and len(buffer) == 6:
                        # 解析确认帧
                        if buffer[2] == 0x00 and buffer[3] == 0x00 and buffer[4] == 0x00:
                            # 设定温度确认
                            print("收到确认帧")
                        else:
                            # 设定PID参数确认
                            kp = buffer[2] / 10.0
                            ki = buffer[3] / 100.0
                            kd = buffer[4] / 10.0
                            print(f"收到确认帧: Kp={kp}, Ki={ki}, Kd={kd}")
                    buffer = bytearray()  # 清空缓冲区
            time.sleep(0.01)


    def stop_reading(self):
        self.running = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

    def set_temperature(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                temp = int(self.set_temp_var.get())
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

    def request_temperature(self):
        if self.serial_port and self.serial_port.is_open:
            command = bytes([0x55, 0x02, 0x00, 0x00, 0x00, 0xaa])
            self.serial_port.write(command)
            print("请求当前温度")

    def set_pid_parameters(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                kp = float(self.kp_var.get())
                ki = float(self.ki_var.get())
                kd = float(self.kd_var.get())
                kp_byte = int(kp * 10)
                ki_byte = int(ki * 100)
                kd_byte = int(kd * 10)
                command = bytes([0x55, 0x02, kp_byte, ki_byte, kd_byte, 0xaa])
                self.serial_port.write(command)
            except ValueError:
                print("无效的PID参数")

    def start_pyqtgraph(self):
        self.qapp = QtWidgets.QApplication([])
        self.plot_widget = pg.GraphicsLayoutWidget(show=True)
        self.plot = self.plot_widget.addPlot(title="实时温度")
        self.plot.setLabel('left', '温度', '°C')
        self.plot.setLabel('bottom', '时间', 's')
        self.plot.showGrid(x=True, y=True)
        self.curve = self.plot.plot(pen='r')

        # Embed the pyqtgraph plot in the Tkinter window
        self.tk_frame = tk.Frame(self.root)
        self.tk_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.embed_frame = tk.Frame(self.tk_frame)
        self.embed_frame.pack(fill=tk.BOTH, expand=True)
        self.embed_frame_id = self.embed_frame.winfo_id()

        self.plot_widget.setParent(None)
        self.plot_widget.winId = lambda: self.embed_frame_id
        self.plot_widget.show()

        # Start the PySide6 event loop in a separate thread
        self.thread = threading.Thread(target=self.qapp.exec_)
        self.thread.start()

    def update_plot(self):
        while not self.data_queue.empty():
            current_time, temp = self.data_queue.get()
            self.times.append(current_time)
            self.temps.append(temp)

            # 更新整个曲线
            self.curve.setData(self.times, self.temps)

            # 自动调整 x 轴范围以显示所有数据
            self.plot_widget.setXRange(0, max(self.times))

            # 更新当前温度标签
            self.current_temp_label.config(text=f"当前温度: {temp:.1f}°C")

            # 更新表格
            self.update_table(current_time, temp)

        # Schedule next update
        self.root.after(100, self.update_plot)

    def update_table(self, current_time, temp):
        time_str = time.strftime("%H:%M:%S", time.localtime())
        self.tree.insert("", "end", values=(time_str, f"{temp:.1f}"))

        # Scroll to the last row
        self.tree.yview_moveto(1)

    def save_data(self):
        with open('temperature_data.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["经过时间(s)", "时间", "温度(°C)"])
            for t, temp in zip(self.times, self.temps):
                time_str = time.strftime("%H:%M:%S", time.localtime(self.start_time + t))
                writer.writerow([f"{t:.1f}", time_str, f"{temp:.1f}"])
        print("数据已保存到 temperature_data.csv")

    def simulate_readings(self):
        temp = 20.0
        time_interval = 1  # seconds
        self.start_time = time.time()
        self.times.clear()
        self.temps.clear()
        for i in range(50):  # 模拟50个数据点
            if not self.running:
                break
            current_time = i * time_interval
            temp += (50 - temp) / 10
            self.last_temp = temp
            self.update_plot(current_time, temp)
            self.update_table(current_time, temp)
            time.sleep(time_interval)
        self.running = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

    def on_closing(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.root.quit()
        self.root.destroy()
        self.plot_widget.close()
        self.plot_app.quit()


if __name__    == "__main__":
    root = tk.Tk()
    app = SerialApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
