"""Desktop controls for the standard USB CDC command protocol."""
import time
import tkinter as tk
from tkinter import ttk

from serial_control import MODES, SerialLink, device_ports, response_fields, target_command


class ControlPanel(tk.Toplevel):
    def __init__(self, parent, *, find_ports=device_ports, create_link=SerialLink):
        super().__init__(parent)
        self.find_ports = find_ports
        self.create_link = create_link
        self.link = None
        self.connected = False
        self.deadline = self.next_ping = self.next_feedback = 0.0
        self.port_name = tk.StringVar()
        self.axis = tk.StringVar(value='0')
        self.mode = tk.StringVar(value='力矩')
        self.unit = tk.StringVar(value=MODES['力矩'][1])
        self.target = tk.StringVar()
        self.raw = tk.StringVar()
        self.device = tk.StringVar(value='未连接')
        self.board_mode = tk.StringVar(value='—')
        self.notice = tk.StringVar(value='选择 ODrive 的 USB 串口。')
        self.metrics = {name: tk.StringVar(value='—') for name in
                        ('position_rad', 'speed_rad_s', 'iq_a', 'torque_est_nm', 'fault', 'cal')}
        self.controls = []
        self.title('ODrive FOC · 串口控制')
        self.configure(background='#EDF2F7')
        self.geometry(f'{min(1030,self.winfo_screenwidth()-60)}x{min(655,self.winfo_screenheight()-86)}')
        self.minsize(min(820,self.winfo_screenwidth()-32), 500)
        self._layout()
        self.mode.trace_add('write', lambda *_: self.unit.set(MODES[self.mode.get()][1]))
        self.axis.trace_add('write', self._clear_feedback)
        self.bind('<Destroy>', self._on_destroy, add=True)
        self.scan()
        self._poll_job = self.after(40, self.poll)

    def _button(self, parent, label, action, *, primary=False):
        button = ttk.Button(parent, text=label, command=action,
                            style='Primary.TButton' if primary else 'Outline.TButton')
        self.controls.append(button)
        button.state(['disabled'])
        return button

    def _layout(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        top = ttk.Frame(self, padding=(18, 12), style='Panel.TFrame')
        top.grid(row=0, column=0, sticky='ew', padx=14, pady=(14, 8))
        top.columnconfigure(0, weight=1)
        ttk.Label(top, text='USB 串口控制', style='PanelTitle.TLabel').grid(row=0, column=0, sticky='w')
        self.ports = ttk.Combobox(top, textvariable=self.port_name, width=12, state='readonly')
        self.ports.grid(row=0, column=1, padx=(6, 0))
        ttk.Button(top, text='刷新', style='Outline.TButton', command=self.scan).grid(row=0, column=2, padx=6)
        self.connect_button = ttk.Button(top, text='连接', style='Outline.TButton', command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3)
        ttk.Label(top, textvariable=self.device, style='Caption.TLabel').grid(row=1, column=0, sticky='w')

        body = ttk.Frame(self, style='Panel.TFrame', padding=15)
        body.grid(row=1, column=0, sticky='nsew', padx=14, pady=(0, 8))
        body.rowconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
        commands = ttk.Frame(body, style='Flat.TFrame', padding=(0, 0, 20, 0))
        commands.grid(row=0, column=0, sticky='nsw')
        commands.columnconfigure(1, weight=1)
        ttk.Label(commands, text='控制目标', style='PanelTitle.TLabel').grid(row=0, columnspan=3, sticky='w', pady=(0, 10))
        for row, label, value, choices in (
            (1, '电机轴', self.axis, ('0', '1')), (2, '选择模式', self.mode, tuple(MODES)),
        ):
            ttk.Label(commands, text=label, style='Field.TLabel').grid(row=row, column=0, sticky='w', pady=5)
            ttk.Combobox(commands, textvariable=value, values=choices, state='readonly', width=11).grid(
                row=row, column=1, columnspan=2, sticky='ew', padx=(10, 0))
        ttk.Label(commands, text='目标值', style='Field.TLabel').grid(row=3, column=0, sticky='w', pady=5)
        entry = ttk.Entry(commands, textvariable=self.target, width=13, style='Path.TEntry')
        entry.grid(row=3, column=1, padx=(10, 6), sticky='ew')
        entry.bind('<Return>', lambda _: self.apply_target())
        ttk.Label(commands, textvariable=self.unit, style='Caption.TLabel').grid(row=3, column=2, sticky='w')
        self._button(commands, '发送目标', self.apply_target, primary=True).grid(
            row=4, columnspan=3, sticky='ew', pady=(9, 5))
        for row, (label, action) in enumerate((
            ('应用模式并置空闲', self.apply_mode), ('准备驱动', lambda: self.send('prepare')),
            ('校准所选轴', lambda: self.send(f'cal {self.axis.get()}')),
            ('位置清零', lambda: self.send(f'zero {self.axis.get()}')),
            ('所选轴空闲', lambda: self.send(f'idle {self.axis.get()}')),
            ('清除故障', lambda: self.send('clear')), ('停止两轴', lambda: self.send('stop')),
        ), start=5):
            self._button(commands, label, action).grid(row=row, columnspan=3, sticky='ew', pady=2)

        feedback = ttk.Frame(body, style='Flat.TFrame')
        feedback.grid(row=0, column=1, sticky='nsew')
        feedback.columnconfigure(0, weight=1)
        feedback.rowconfigure(3, weight=1)
        ttk.Label(feedback, text='轴反馈', style='PanelTitle.TLabel').grid(row=0, sticky='w')
        ttk.Label(feedback, textvariable=self.board_mode, style='Caption.TLabel').grid(row=1, sticky='w', pady=(4, 8))
        numbers = ttk.Frame(feedback, style='Flat.TFrame')
        numbers.grid(row=2, sticky='ew', pady=(0, 8))
        for i, (name, label) in enumerate((
            ('position_rad','位置 · rad'), ('speed_rad_s','速度 · rad/s'),
            ('iq_a','Iq · A'), ('torque_est_nm','估算力矩 · N·m'), ('fault','故障'), ('cal','已校准'),
        )):
            column = i % 3
            row = 2 * (i // 3)
            numbers.columnconfigure(column, weight=1)
            ttk.Label(numbers, text=label, style='Caption.TLabel').grid(row=row, column=column, sticky='w')
            ttk.Label(numbers, textvariable=self.metrics[name], style='Field.TLabel').grid(
                row=row+1, column=column, sticky='w', pady=(1, 7))
        self.log = tk.Text(feedback, height=8, state='disabled', background='#101E2E',
                           foreground='#DAE8F6', relief='flat', padx=9, pady=7, wrap='word')
        self.log.grid(row=3, sticky='nsew')
        raw = ttk.Frame(feedback, style='Flat.TFrame')
        raw.grid(row=4, sticky='ew', pady=(8, 0))
        raw.columnconfigure(0, weight=1)
        entry = ttk.Entry(raw, textvariable=self.raw, style='Path.TEntry')
        entry.grid(row=0, column=0, sticky='ew')
        entry.bind('<Return>', lambda _: self.send_raw())
        self._button(raw, '发送命令', self.send_raw).grid(row=0, column=1, padx=(7, 0))
        self._button(raw, '帮助', lambda: self.send('help')).grid(row=0, column=2, padx=(7, 0))
        ttk.Label(self, textvariable=self.notice, background='#EDF2F7', foreground='#52677E').grid(
            row=2, column=0, sticky='w', padx=17, pady=(0, 8))

    def _clear_feedback(self, *_):
        self.board_mode.set('—')
        for metric in self.metrics.values():
            metric.set('—')

    def scan(self):
        if self.link is not None:
            return
        try:
            ports = self.find_ports()
        except (OSError, RuntimeError) as error:
            self.notice.set(str(error))
            return
        self.ports.configure(values=ports)
        if self.port_name.get() not in ports:
            self.port_name.set(ports[0] if ports else '')
        self.notice.set('选择 ODrive 的 USB 串口。' if ports else '未发现 ODrive USB 串口。')

    def toggle_connection(self):
        if self.link is not None:
            self.disconnect('串口已断开。')
            return
        name = self.port_name.get()
        if not name:
            self.notice.set('请先选择串口。')
            return
        try:
            self.link = self.create_link(name)
            self.link.send('info')
        except (OSError, RuntimeError, ValueError) as error:
            self.disconnect(str(error))
            return
        self.deadline = time.monotonic() + 1.5
        self.ports.state(['disabled'])
        self.connect_button.configure(text='断开')
        self.device.set(f'{name} · 正在识别')

    def disconnect(self, message):
        if self.link is not None:
            try:
                self.link.close()
            except OSError:
                pass
        self.link = None
        self.connected = False
        self.device.set('未连接')
        self.ports.state(['!disabled'])
        self.connect_button.configure(text='连接')
        self._clear_feedback()
        for button in self.controls:
            button.state(['disabled'])
        self.notice.set(message)

    def _on_destroy(self, event):
        if event.widget is not self:
            return
        self.after_cancel(self._poll_job)
        if self.link is not None:
            try:
                self.link.close()
            except OSError:
                pass
            self.link = None

    def append(self, text):
        self.log.configure(state='normal')
        self.log.insert('end', text+'\n')
        if int(self.log.index('end-1c').split('.')[0]) > 600:
            self.log.delete('1.0', '101.0')
        self.log.configure(state='disabled')
        self.log.see('end')

    def send(self, command):
        if not self.connected:
            return
        try:
            self.link.send(command)
        except ValueError as error:
            self.notice.set(str(error))
        except OSError as error:
            self.disconnect(str(error))
        else:
            self.append('> '+command)

    def send_raw(self):
        value = self.raw.get()
        if value.strip():
            self.send(value)

    def apply_mode(self):
        self.send(f'mode {self.axis.get()} {MODES[self.mode.get()][0]}')

    def apply_target(self):
        try:
            command = target_command(self.mode.get(), self.axis.get(), self.target.get())
        except ValueError:
            self.notice.set('请输入有限的目标数值。')
        else:
            self.send(command)

    def _response(self, line):
        fields = response_fields(line)
        if line.startswith('OK info ') and fields.get('protocol') == '2' and fields.get('transport') == 'usb-cdc':
            self.connected = True
            self.device.set(f'{self.port_name.get()} · {fields.get("fw", "ODrive FOC")}')
            for button in self.controls:
                button.state(['!disabled'])
            self.notice.set('已连接 · 115200 8N1')
        if line.startswith('OK feedback '):
            if fields.get('axis') == self.axis.get():
                self.board_mode.set(f'模式 {fields.get("mode", "—")} · 状态 {fields.get("state", "—")}')
                for name, metric in self.metrics.items():
                    metric.set(fields.get(name, '—'))
        elif line != 'pong':
            self.append(line)
            if line.startswith('ERR '):
                self.notice.set(line)

    def poll(self):
        now = time.monotonic()
        if self.link is not None:
            try:
                for line in self.link.read():
                    self._response(line)
                if not self.connected and now >= self.deadline:
                    self.disconnect('串口未返回协议 2，检查固件版本。')
                elif self.connected:
                    if self.link.active and now >= self.next_ping:
                        self.link.send('ping')
                        self.next_ping = now + 0.25
                    if now >= self.next_feedback:
                        axes = self.link.active | {int(self.axis.get())}
                        for axis in sorted(axes):
                            self.link.send(f'feedback {axis}')
                        self.next_feedback = now + 0.35
            except OSError as error:
                self.disconnect(str(error))
        self._poll_job = self.after(40, self.poll)


if __name__ == '__main__':
    from deploy_gui import apply_styles, enable_dpi_scaling
    enable_dpi_scaling()
    root = tk.Tk()
    root.withdraw()
    apply_styles(root)
    panel = ControlPanel(root)
    panel.protocol('WM_DELETE_WINDOW', root.destroy)
    root.mainloop()
