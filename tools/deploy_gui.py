"""Windows desktop interface for the ODrive FOC deployment pipeline."""
import argparse
import ctypes
import json
import os
from pathlib import Path
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, font, messagebox, ttk

from deploy_environment import (detect_gcc, detect_matlab, detect_openocd, detect_project,
                                matlab_release, resolve_project, validate_matlab)
from deploy_foc import Deployment
from deploy_session import BuildSession, external_process
from install_foc import has_eide_target

INK = '#142338'
MUTED = '#52677E'
SURFACE = '#FFFFFF'
CANVAS = '#EDF2F7'
LINE = '#D6E0EA'
HEADER = '#14243A'
CONSOLE = '#101E2E'
TEAL = '#086B62'
TEAL_DARK = '#07584F'
TEAL_PALE = '#E0F3ED'
PALE = '#EAF0F6'
AMBER = '#8A5916'
ERROR = '#A53949'
MODE_INSTALL = '移植并构建'
MODE_BUILD = '离线构建'
MODE_FLASH = '构建并烧录'
MODES = (MODE_INSTALL, MODE_BUILD, MODE_FLASH)
STAGES = (
    ('model', '代码生成'),
    ('compile', '固件编译'),
    ('verify', 'ARM 校验'),
    ('install', '同步工程'),
    ('flash', 'SWD 烧录'),
)
PATH_KEYS = ('project', 'model', 'output', 'matlab', 'gcc', 'openocd')
SETTINGS = Path(os.environ.get('LOCALAPPDATA', str(Path.home()))) / 'ODriveFOC/settings.json'


def read_settings():
    try:
        data = json.loads(SETTINGS.read_text(encoding='utf-8'))
        if isinstance(data, dict):
            return data
    except (OSError, ValueError):
        pass
    return {}


def enable_dpi_scaling():
    if os.name == 'nt':
        try:
            # Tk 8.6 supports system-DPI fonts and native dialogs.
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (OSError, AttributeError):
            ctypes.windll.user32.SetProcessDPIAware()


def system_font(root, *choices):
    installed = set(font.families(root))
    return next((name for name in choices if name in installed), 'TkDefaultFont')


def apply_styles(root):
    ui_font = system_font(root, 'Microsoft YaHei UI', 'Microsoft YaHei', 'Segoe UI')
    display = system_font(root, 'Inter', 'Segoe UI')
    mono = system_font(root, 'Cascadia Mono', 'Consolas')
    style = ttk.Style(root)
    style.theme_use('clam')
    style.configure('.', font=(ui_font, 10), foreground=INK)
    style.configure('Panel.TFrame', background=SURFACE, relief='solid', borderwidth=1, bordercolor=LINE)
    style.configure('Flat.TFrame', background=SURFACE)
    style.configure('Panel.TLabel', background=SURFACE, foreground=INK)
    style.configure('PanelTitle.TLabel', background=SURFACE, font=(ui_font, 14, 'bold'), foreground=INK)
    style.configure('Caption.TLabel', background=SURFACE, foreground=MUTED, font=(ui_font, 9))
    style.configure('Field.TLabel', background=SURFACE, font=(ui_font, 9, 'bold'))
    style.configure('Outline.TButton', background='#F6F9FC', foreground=INK, bordercolor=LINE,
                    darkcolor='#F6F9FC', lightcolor='#F6F9FC', padding=(12, 6), relief='flat')
    style.map('Outline.TButton', background=[('disabled', '#F3F6F9'), ('active', '#E4EDF6')],
              foreground=[('disabled', '#8493A4')], bordercolor=[('focus', TEAL)])
    style.configure('Primary.TButton', background=TEAL, foreground='white', bordercolor=TEAL,
                    darkcolor=TEAL, lightcolor=TEAL, padding=(16, 9), font=(ui_font, 11, 'bold'))
    style.map('Primary.TButton', background=[('disabled', '#D9E3EB'), ('active', TEAL_DARK)],
              foreground=[('disabled', MUTED), ('active', 'white')])
    style.configure('Path.TEntry', padding=(8, 5), fieldbackground='#F7FAFD',
                    foreground=INK, bordercolor=LINE, insertcolor=INK)
    style.map('Path.TEntry', bordercolor=[('focus', TEAL)], fieldbackground=[('disabled', PALE)])
    style.configure('Mode.TCombobox', padding=(6, 5), bordercolor=LINE, arrowsize=12)
    style.map('Mode.TCombobox', fieldbackground=[('readonly', '#F7FAFD')])
    style.configure('Panel.TCheckbutton', background=SURFACE, foreground=INK,
                    indicatorcolor='#F7FAFD', padding=(0, 2))
    style.map('Panel.TCheckbutton', background=[('active', SURFACE)],
              indicatorcolor=[('selected', TEAL)], foreground=[('disabled', MUTED)])
    style.configure('Build.Horizontal.TProgressbar', background=TEAL, troughcolor=PALE,
                    bordercolor=PALE, darkcolor=TEAL, lightcolor=TEAL, thickness=5)
    style.configure('Log.Vertical.TScrollbar', background='#3C536C', troughcolor=CONSOLE, arrowcolor='#C3D5E5',
                    bordercolor=CONSOLE, darkcolor='#3C536C', lightcolor='#3C536C', arrowsize=10)
    return ui_font, display, mono


class DeployApp:
    def __init__(self, root, preferences=None, on_done=None):
        self.root = root
        self.on_done = on_done
        self.persist = preferences is None
        self.ui_font, self.display_font, self.mono_font = apply_styles(root)
        self.root.title('ODrive FOC · Simulink 固件部署')
        self.root.configure(background=CANVAS)
        icon = Path(__file__).resolve().parent / 'ui_assets/odrive_foc.ico'
        if icon.is_file() and os.name == 'nt':
            self.root.iconbitmap(str(icon))
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.bind('<F5>', lambda _: self.start())

        self.events = queue.Queue()
        self.session = None
        self.last_folder = None
        self.closing = False
        self.started = None
        self.refresh_job = None
        self.locked = []
        self.stage_labels = {}
        self.dialog = None
        self.control_panel = None
        preset = preferences if preferences is not None else read_settings()
        project = detect_project()
        if isinstance(preset.get('project'), str) and preset['project']:
            try:
                project = resolve_project(preset['project'])
            except (ValueError, OSError):
                pass
        defaults = {
            'project': project, 'model': project / 'simulink/odrive_foc.slx' if project else None,
            'output': project / 'build/Simulink' if project else None,
            'matlab': detect_matlab(), 'gcc': detect_gcc(), 'openocd': detect_openocd(),
        }
        self.paths = {
            key: tk.StringVar(value=preset[key] if isinstance(preset.get(key), str) else str(defaults[key] or ''))
            for key in PATH_KEYS
        }
        self.verify = tk.BooleanVar(value=preset.get('verify_arm', True) is not False)
        self.mode = tk.StringVar(value=MODE_INSTALL if has_eide_target(project) else MODE_BUILD)
        self.readiness = tk.StringVar()
        self.message = tk.StringVar(value='选择项目后即可构建。')
        self.current = tk.StringVar(value='等待开始')
        self.output_text = tk.StringVar(value='完成后可打开本次构建产物。')
        self.timer = tk.StringVar(value='')
        self._layout()
        self.root.update_idletasks()
        width, height = root.winfo_screenwidth(), root.winfo_screenheight()
        desired_height = max(736, root.winfo_reqheight() + 10)
        self.root.geometry(f'{min(1200, width - 60)}x{min(desired_height, height - 80)}')
        self.root.minsize(min(900, width - 32), min(max(650, root.winfo_reqheight()-50), height - 72))
        for variable in (*self.paths.values(), self.mode, self.verify):
            variable.trace_add('write', self.schedule_refresh)
        self.refresh()
        self.root.after(80, self.pump)

    def _layout(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)
        header = tk.Frame(self.root, background=HEADER)
        header.grid(row=0, column=0, sticky='ew')
        header.columnconfigure(1, weight=1)
        mark = tk.Canvas(header, width=38, height=38, background=TEAL, highlightthickness=0)
        mark.create_line(6, 22, 13, 22, 17, 12, 23, 28, 27, 16, 33, 16,
                         fill='#E8FFF6', width=2, joinstyle='round')
        mark.grid(row=0, column=0, rowspan=2, padx=(26, 14), pady=20)
        tk.Label(header, text='ODrive FOC', background=HEADER, foreground='#F7FCFF',
                 font=(self.display_font, 24, 'bold'), anchor='w').grid(
                     row=0, column=1, sticky='sw', pady=(14, 0))
        tk.Label(header, text='Simulink 固件部署', background=HEADER, foreground='#BBCDE2',
                 font=(self.ui_font, 10), anchor='w').grid(row=1, column=1, sticky='nw', pady=(0, 16))
        ttk.Button(header, text='串口控制', style='Outline.TButton', command=self.open_control).grid(
            row=0, column=2, rowspan=2, padx=(8, 0))
        tk.Label(header, text='STM32F405', background='#293C53', foreground='#DCE9F7',
                 font=(self.display_font, 10, 'bold'), padx=13, pady=6).grid(
                     row=0, column=3, rowspan=2, padx=(16, 26))

        workspace = tk.Frame(self.root, background=CANVAS)
        workspace.grid(row=1, column=0, sticky='nsew', padx=20, pady=(18, 15))
        workspace.columnconfigure(0, weight=2, minsize=350)
        workspace.columnconfigure(1, weight=3, minsize=460)
        workspace.rowconfigure(0, weight=1)
        inputs = ttk.Frame(workspace, style='Panel.TFrame', padding=(20, 16))
        inputs.grid(row=0, column=0, sticky='nsew', padx=(0, 14))
        inputs.columnconfigure(0, weight=1)
        ttk.Label(inputs, text='工程配置', style='PanelTitle.TLabel').grid(row=0, sticky='w', pady=(0, 11))
        self.path_row(inputs, 1, '工程目录', 'project', lambda: self.choose_directory('project'))
        self.path_row(inputs, 3, 'Simulink 模型', 'model', lambda: self.choose_file('model'))
        self.path_row(inputs, 5, '输出目录', 'output', lambda: self.choose_directory('output'))

        tk.Frame(inputs, background=LINE, height=1).grid(row=7, sticky='ew', pady=(15, 12))
        env_title = ttk.Frame(inputs, style='Flat.TFrame')
        env_title.grid(row=8, sticky='ew')
        env_title.columnconfigure(0, weight=1)
        ttk.Label(env_title, text='工具链', style='Field.TLabel').grid(row=0, column=0, sticky='w')
        self.locked.append(ttk.Button(env_title, text='工具链设置', style='Outline.TButton',
                                      command=self.configure_tools))
        self.locked[-1].grid(row=0, column=1, sticky='e')
        ttk.Label(inputs, textvariable=self.readiness, style='Caption.TLabel').grid(
            row=9, sticky='w', pady=(5, 6))
        self.open_model_button = ttk.Button(inputs, text='在 MATLAB 中打开模型', style='Outline.TButton',
                                            command=self.open_model)
        self.open_model_button.grid(row=10, sticky='w', pady=(2, 5))
        self.locked.append(self.open_model_button)

        inputs.rowconfigure(11, weight=1)
        tk.Frame(inputs, background=LINE, height=1).grid(row=12, sticky='ew', pady=(13, 12))
        mode_row = ttk.Frame(inputs, style='Flat.TFrame')
        mode_row.grid(row=13, sticky='ew')
        mode_row.columnconfigure(0, weight=1)
        choose_mode = ttk.Combobox(mode_row, textvariable=self.mode, values=MODES, state='readonly',
                                   style='Mode.TCombobox', width=14)
        choose_mode.grid(row=0, column=0, sticky='ew', padx=(0, 13))
        verify = ttk.Checkbutton(mode_row, text='ARM 校验', variable=self.verify,
                                 style='Panel.TCheckbutton')
        verify.grid(row=0, column=1, sticky='e')
        self.locked.extend((choose_mode, verify))
        self.start_button = ttk.Button(inputs, text='开始构建', style='Primary.TButton', command=self.start)
        self.start_button.grid(row=14, sticky='ew', pady=(12, 0))

        output = ttk.Frame(workspace, style='Panel.TFrame', padding=(20, 16))
        output.grid(row=0, column=1, sticky='nsew')
        output.columnconfigure(0, weight=1)
        output.rowconfigure(5, weight=1)
        status = ttk.Frame(output, style='Flat.TFrame')
        status.grid(row=0, sticky='ew')
        status.columnconfigure(0, weight=1)
        ttk.Label(status, text='构建进度', style='PanelTitle.TLabel').grid(row=0, column=0, sticky='w')
        self.state = tk.Label(status, text='就绪', background=TEAL_PALE, foreground=TEAL,
                              font=(self.ui_font, 9, 'bold'), padx=11, pady=4)
        self.state.grid(row=0, column=1, sticky='e')

        steps = ttk.Frame(output, style='Flat.TFrame')
        steps.grid(row=1, sticky='ew', pady=(14, 6))
        for index, (name, label) in enumerate(STAGES):
            chip = tk.Label(steps, text=label, background=PALE, foreground=MUTED,
                            font=(self.ui_font, 9, 'bold'), anchor='center', pady=6)
            chip.grid(row=0, column=index, sticky='ew', padx=(0, 6))
            steps.columnconfigure(index, weight=1)
            self.stage_labels[name] = chip
        self.steps = steps
        progress_slot = tk.Frame(output, height=5, background=PALE)
        progress_slot.grid(row=2, sticky='ew', pady=(0, 9))
        self.progress = ttk.Progressbar(progress_slot, maximum=100, style='Build.Horizontal.TProgressbar')
        self.progress.place(x=0, y=0, relwidth=1, relheight=1)
        ttk.Label(output, textvariable=self.current, style='Caption.TLabel').grid(row=3, sticky='w')

        log_header = ttk.Frame(output, style='Flat.TFrame')
        log_header.grid(row=4, sticky='ew', pady=(14, 7))
        log_header.columnconfigure(0, weight=1)
        ttk.Label(log_header, text='构建日志', style='Field.TLabel').grid(row=0, column=0, sticky='w')
        ttk.Button(log_header, text='复制日志', style='Outline.TButton', command=self.copy_log).grid(
            row=0, column=1, sticky='e')
        log = ttk.Frame(output, style='Flat.TFrame')
        log.grid(row=5, sticky='nsew')
        log.columnconfigure(0, weight=1)
        log.rowconfigure(0, weight=1)
        self.log = tk.Text(log, height=9, background=CONSOLE, foreground='#D7E5F4', insertbackground='white',
                           selectbackground='#28576D', borderwidth=0, highlightthickness=0, wrap='word',
                           font=(self.mono_font, 9), padx=13, pady=11, state='disabled', takefocus=True)
        self.log.grid(row=0, column=0, sticky='nsew')
        bar = ttk.Scrollbar(log, orient='vertical', command=self.log.yview, style='Log.Vertical.TScrollbar')
        bar.grid(row=0, column=1, sticky='ns')
        self.log.configure(yscrollcommand=bar.set)
        self.log.tag_configure('error', foreground='#FFB6BA')

        ttk.Label(output, textvariable=self.output_text, style='Caption.TLabel').grid(
            row=6, sticky='ew', pady=(11, 7))
        actions = ttk.Frame(output, style='Flat.TFrame')
        actions.grid(row=7, sticky='ew')
        actions.columnconfigure(0, weight=1)
        self.files_button = ttk.Button(actions, text='打开产物目录', style='Outline.TButton',
                                       command=self.open_output, state='disabled')
        self.files_button.grid(row=0, column=0, sticky='w')
        self.cancel_button = ttk.Button(actions, text='取消构建', style='Outline.TButton',
                                        command=self.cancel, state='disabled')
        self.cancel_button.grid(row=0, column=1, sticky='e')
        output.bind('<Configure>', lambda event: self.wrap_output(output, event.width))

        footer = tk.Frame(self.root, background=CANVAS)
        footer.grid(row=2, column=0, sticky='ew', padx=22, pady=(0, 9))
        footer.columnconfigure(0, weight=1)
        hint = tk.Label(footer, textvariable=self.message, background=CANVAS, foreground=MUTED,
                        font=(self.ui_font, 9), anchor='w', justify='left')
        hint.grid(row=0, column=0, sticky='ew')
        footer.bind('<Configure>', lambda event: hint.configure(wraplength=max(200, event.width - 84)))
        tk.Label(footer, textvariable=self.timer, background=CANVAS, foreground=MUTED,
                 font=(self.display_font, 9)).grid(row=0, column=1, sticky='e')

    def wrap_output(self, frame, width):
        for widget in frame.grid_slaves(row=6):
            widget.configure(wraplength=max(280, width - 46))

    def path_row(self, parent, row, label, key, choose):
        ttk.Label(parent, text=label, style='Field.TLabel').grid(row=row, sticky='w', pady=(6, 4))
        field = ttk.Frame(parent, style='Flat.TFrame')
        field.grid(row=row+1, sticky='ew')
        field.columnconfigure(0, weight=1)
        entry = ttk.Entry(field, textvariable=self.paths[key], style='Path.TEntry', width=18)
        entry.grid(row=0, column=0, sticky='ew')
        button = ttk.Button(field, text='浏览…', command=choose, style='Outline.TButton', width=6)
        button.grid(row=0, column=1, padx=(7, 0))
        self.locked.extend((entry, button))

        def show_tail(*_):
            if entry.winfo_exists() and entry.focus_get() != entry:
                entry.xview_moveto(1)

        entry.bind('<Configure>', show_tail)
        entry.bind('<FocusOut>', show_tail)
        variable = self.paths[key]
        trace = variable.trace_add('write', lambda *_: self.root.after_idle(show_tail))
        entry.bind('<Destroy>', lambda _: variable.trace_remove('write', trace))

    def choose_directory(self, key):
        value = filedialog.askdirectory(parent=self.root, initialdir=self.paths[key].get() or None,
                                        title='选择工程目录' if key == 'project' else '选择输出目录')
        if not value:
            return
        if key == 'project':
            try:
                project = resolve_project(value)
            except ValueError as error:
                self.message.set(str(error))
                return
            self.paths['project'].set(str(project))
            self.paths['model'].set(str(project / 'simulink/odrive_foc.slx'))
            self.paths['output'].set(str(project / 'build/Simulink'))
            if self.mode.get() != MODE_FLASH:
                self.mode.set(MODE_INSTALL if has_eide_target(project) else MODE_BUILD)
        else:
            self.paths[key].set(value)
        self.save_settings()

    def choose_file(self, key, parent=None):
        current = self.paths[key].get()
        types = [('Simulink 模型', '*.slx')] if key == 'model' else [('Windows 可执行文件', '*.exe')]
        choice = filedialog.askopenfilename(parent=parent or self.root,
                                            initialdir=str(Path(current).parent) if current else None,
                                            filetypes=types)
        if choice:
            self.paths[key].set(choice)
            if key == 'model':
                self.save_settings()

    def open_control(self):
        from control_gui import ControlPanel
        if self.control_panel is not None and self.control_panel.winfo_exists():
            self.control_panel.lift()
        else:
            self.control_panel = ControlPanel(self.root)

    def configure_tools(self):
        if self.dialog is not None and self.dialog.winfo_exists():
            self.dialog.lift()
            return
        dialog = tk.Toplevel(self.root)
        self.dialog = dialog
        dialog.title('工具链设置')
        dialog.transient(self.root)
        dialog.configure(background=CANVAS)
        dialog.columnconfigure(0, weight=1)
        body = ttk.Frame(dialog, style='Panel.TFrame', padding=(19, 15))
        body.grid(row=0, column=0, sticky='nsew', padx=13, pady=13)
        body.columnconfigure(0, weight=1)
        ttk.Label(body, text='工具链', style='PanelTitle.TLabel').grid(row=0, sticky='w', pady=(0, 6))
        old_controls = len(self.locked)
        for row, title, key in (
            (1, 'MATLAB R2024b', 'matlab'), (3, 'Arm GNU Toolchain', 'gcc'),
            (5, 'OpenOCD（SWD 烧录）', 'openocd'),
        ):
            self.path_row(body, row, title, key, lambda k=key: self.choose_file(k, dialog))
        del self.locked[old_controls:]
        bottom = ttk.Frame(body, style='Flat.TFrame')
        bottom.grid(row=7, sticky='ew', pady=(14, 0))
        bottom.columnconfigure(0, weight=1)
        ttk.Button(bottom, text='自动查找', style='Outline.TButton', command=self.auto_tools).grid(
            row=0, column=0, sticky='w')
        ttk.Button(bottom, text='完成', style='Primary.TButton',
                   command=lambda: (self.save_settings(), dialog.destroy())).grid(row=0, column=1)
        dialog.protocol('WM_DELETE_WINDOW', lambda: (self.save_settings(), dialog.destroy()))
        dialog.update_idletasks()
        size = max(340, dialog.winfo_reqheight()+8)
        dialog.geometry(f'690x{size}')
        dialog.minsize(540, size)
        dialog.grab_set()

    def auto_tools(self):
        for key, locate in (('matlab', detect_matlab), ('gcc', detect_gcc), ('openocd', detect_openocd)):
            found = locate()
            if found:
                self.paths[key].set(str(found))
        self.refresh()

    def options(self):
        value = {key: variable.get().strip() for key, variable in self.paths.items()}
        if not value['project']:
            raise ValueError('请先选择 CM_Odrive 工程目录。')
        project = resolve_project(value['project'])
        def path(key, fallback=None):
            item = value[key] or fallback
            return Path(item).expanduser().resolve() if item else None
        options = Deployment(
            project=project,
            model=path('model', project / 'simulink/odrive_foc.slx'),
            matlab=path('matlab', 'matlab.exe'),
            gcc=path('gcc', 'arm-none-eabi-gcc.exe'),
            output=path('output', project / 'build/Simulink'),
            verify_arm=self.verify.get(), flash=self.mode.get() == MODE_FLASH, openocd=path('openocd'),
            install_project=self.mode.get() == MODE_INSTALL,
        )
        options.validate()
        return options

    def schedule_refresh(self, *_):
        if self.refresh_job is not None:
            self.root.after_cancel(self.refresh_job)
        self.refresh_job = self.root.after(130, self.refresh)

    def refresh(self):
        self.refresh_job = None
        if self.session is not None:
            return
        mode = self.mode.get()
        self.start_button.configure(text={
            MODE_INSTALL: '一键移植并构建', MODE_BUILD: '开始构建', MODE_FLASH: '构建并烧录',
        }[mode])
        uses_stage = {'install': mode == MODE_INSTALL, 'flash': mode == MODE_FLASH}
        for index, (name, _) in enumerate(STAGES):
            visible = uses_stage.get(name, True)
            if visible:
                self.stage_labels[name].grid()
            else:
                self.stage_labels[name].grid_remove()
            self.steps.columnconfigure(index, weight=int(visible))
        self.stage_labels['verify'].configure(text='ARM 校验' if self.verify.get() else 'ARM 校验跳过')
        matlab = self.paths['matlab'].get().strip()
        gcc = self.paths['gcc'].get().strip()
        release = matlab_release(Path(matlab)) if matlab else ''
        self.readiness.set(' · '.join((
            ('MATLAB ' + release) if release else 'MATLAB 待配置',
            'ARM GCC 已找到' if gcc and Path(gcc).is_file() else 'ARM GCC 待配置',
        )))
        try:
            self.options()
        except (ValueError, OSError) as error:
            self.start_button.state(['disabled'])
            self.message.set(str(error))
            self.state.configure(text='待配置', background=PALE, foreground=MUTED)
        else:
            self.start_button.state(['!disabled'])
            self.message.set({
                MODE_INSTALL: '离线移植到 EIDE 的 Simulink_FOC 目标 · 按 F5 开始',
                MODE_BUILD: '离线构建 · 按 F5 开始',
                MODE_FLASH: 'DAPLink HID / SWD · 写入并读回校验',
            }[mode])
            self.state.configure(text='就绪', background=TEAL_PALE, foreground=TEAL)

    def save_settings(self):
        if not self.persist:
            return
        data = {key: self.paths[key].get() for key in PATH_KEYS}
        data['verify_arm'] = self.verify.get()
        try:
            SETTINGS.parent.mkdir(parents=True, exist_ok=True)
            tmp = SETTINGS.with_name(f'settings-{os.getpid()}.tmp')
            tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
            tmp.replace(SETTINGS)
        except OSError as error:
            self.append_log(f'设置保存失败：{error}', error=True)

    def open_model(self):
        try:
            matlab = Path(self.paths['matlab'].get()).expanduser().resolve()
            model = Path(self.paths['model'].get()).expanduser().resolve()
            validate_matlab(matlab)
            if not model.is_file():
                raise ValueError('请选择要打开的 Simulink 模型。')
            external_process([str(matlab), '-nosplash', '-sd', str(model.parent), '-r',
                              "open_system(getenv('ODRIVE_FOC_UI_MODEL'));"], hidden=False,
                             env=dict(os.environ, ODRIVE_FOC_UI_MODEL=str(model)))
        except (ValueError, OSError) as error:
            self.message.set(str(error))

    def start(self):
        if self.session is not None:
            return
        try:
            options = self.options()
        except (ValueError, OSError) as error:
            self.message.set(str(error))
            if self.on_done:
                self.on_done('error', None, str(error))
            return
        self.save_settings()
        self.session = BuildSession(lambda kind, payload: self.events.put((kind, payload)))
        self.started = time.monotonic()
        self.last_folder = None
        self.current.set('启动 MATLAB R2024b')
        self.output_text.set('构建进行中')
        self.message.set('正在构建')
        self.state.configure(text='进行中', background='#E5EFFA', foreground='#1C507B')
        for chip in self.stage_labels.values():
            chip.configure(background=PALE, foreground=MUTED)
        for widget in (*self.locked, self.start_button, self.files_button):
            widget.state(['disabled'])
        self.cancel_button.state(['!disabled'])
        self.progress.configure(mode='indeterminate')
        self.progress.start(15)
        self.log.configure(state='normal')
        self.log.delete('1.0', 'end')
        self.log.configure(state='disabled')
        self.append_log('模式：' + self.mode.get())
        self.append_log('模型：' + str(options.model))
        threading.Thread(target=self.session.execute, args=(options,), daemon=True).start()

    def append_log(self, value, error=False):
        self.log.configure(state='normal')
        self.log.insert('end', str(value).rstrip() + '\n', 'error' if error else '')
        lines = int(self.log.index('end-1c').split('.')[0])
        if lines > 1600:
            self.log.delete('1.0', f'{lines-1600}.0')
        self.log.configure(state='disabled')
        self.log.see('end')

    def pump(self):
        while True:
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == 'log':
                self.append_log(value)
            elif kind == 'phase':
                self.show_phase(value)
            elif kind == 'finish':
                self.finish(*value)
                if self.closing or self.on_done:
                    return
        if self.session is not None:
            self.timer.set(f'{int(time.monotonic() - self.started)} s')
        self.root.after(80, self.pump)

    def show_phase(self, name):
        names = [stage for stage, _ in STAGES]
        index = names.index(name)
        for prior in names[:index]:
            if prior != 'verify' or self.verify.get():
                self.stage_labels[prior].configure(background=TEAL_PALE, foreground=TEAL)
        self.stage_labels[name].configure(background=TEAL, foreground='white')
        self.current.set(dict(STAGES)[name])
        if name == 'install':
            self.cancel_button.state(['disabled'])
            self.message.set('正在同步已通过编译的 C 源码。')
        if name == 'flash':
            self.cancel_button.state(['disabled'])
            self.message.set('正在写入 Flash 并校验。')

    def finish(self, status, stage, problem):
        ok = status == 'ok'
        self.session = None
        self.last_folder = stage
        self.progress.stop()
        self.progress.configure(mode='determinate', value=100 if ok else 0)
        self.current.set({'ok': '构建完成', 'cancelled': '构建已取消', 'error': '构建失败'}[status])
        for widget in (*self.locked, self.start_button):
            widget.state(['!disabled'])
        self.cancel_button.state(['disabled'])
        self.refresh()
        self.state.configure(text={'ok': '完成', 'cancelled': '已取消', 'error': '失败'}[status],
                             background=TEAL_PALE if ok else PALE if status == 'cancelled' else '#FCE9ED',
                             foreground=TEAL if ok else MUTED if status == 'cancelled' else ERROR)
        self.message.set('构建完成。' if ok else problem)
        self.timer.set(f'{int(time.monotonic() - self.started)} s')
        manifest_file = stage / 'manifest.json' if stage is not None else None
        try:
            info = json.loads(manifest_file.read_text(encoding='utf-8')) if manifest_file else None
        except (OSError, ValueError):
            info = None
        if info:
            checks = (info.get('arm_regression') or {}).get('checks')
            suffix = f'  ·  ARM 校验 {checks:,} 项通过' if checks else ''
            if info.get('project_install'):
                suffix += '  ·  EIDE 已同步'
            self.output_text.set('ELF / HEX / BIN' + suffix + '\n' + str(stage / 'firmware'))
            self.files_button.configure(text='打开产物目录')
        else:
            self.output_text.set(str(stage) if stage is not None else '等待构建产物。')
            self.files_button.configure(text='打开日志目录')
        self.files_button.state(['!disabled'] if stage is not None else ['disabled'])
        if ok:
            for name, _ in STAGES:
                if name != 'verify' or self.verify.get():
                    self.stage_labels[name].configure(background=TEAL_PALE, foreground=TEAL)
        if self.on_done:
            self.on_done(status, stage, problem)
        elif self.closing:
            self.root.destroy()

    def open_output(self):
        if self.last_folder is None:
            return
        dest = self.last_folder
        if (dest / 'manifest.json').exists():
            dest = dest / 'firmware'
        try:
            os.startfile(str(dest))
        except OSError as error:
            self.message.set(str(error))

    def copy_log(self):
        try:
            text = (self.last_folder / 'deploy.log').read_text(encoding='utf-8') if self.last_folder else ''
        except OSError:
            text = ''
        self.root.clipboard_clear()
        self.root.clipboard_append(text or self.log.get('1.0', 'end-1c'))
        self.message.set('日志已复制。')

    def cancel(self):
        if self.session is not None and self.session.request_cancel():
            self.cancel_button.state(['disabled'])
            self.current.set('正在取消构建')

    def close(self):
        if self.session is None:
            self.save_settings()
            self.root.destroy()
        elif self.session.request_cancel():
            self.closing = True
            self.cancel()
        else:
            self.message.set('当前操作完成后可关闭窗口。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    enable_dpi_scaling()
    root = tk.Tk()
    if args.self_test:
        # Exercise the same widgets, event loop and worker in a frozen build.
        root.withdraw()
        job = json.loads(args.self_test.read_text(encoding='utf-8'))
        outcome = {}

        def done(status, folder, error):
            outcome.update(status=status, output=str(folder or ''), error=error,
                           python=sys.version.split()[0], tk=root.tk.call('info', 'patchlevel'),
                           frozen=bool(getattr(sys, 'frozen', False)))
            Path(job['result']).write_text(json.dumps(outcome, ensure_ascii=False, indent=2), encoding='utf-8')
            root.after_idle(root.destroy)

        app = DeployApp(root, job, on_done=done)
        if job.get('build', False):
            root.after_idle(app.start)
        elif job.get('check_runtime', False):
            def check_runtime():
                try:
                    from test_firmware import check_arm_runtime
                    from serial_control import check_serial_runtime
                    session = BuildSession(lambda *_: None)
                    check_arm_runtime(run=session.run, report=session.report)
                    check_serial_runtime()
                except Exception as error:
                    done('error', None, str(error))
                else:
                    done('ready', None, '')

            root.after_idle(check_runtime)
        else:
            root.after_idle(lambda: done('blocked', None, app.message.get())
                            if app.start_button.instate(['disabled']) else done('ready', None, ''))
        root.mainloop()
        return 0 if outcome.get('status') in ('ready', 'ok') else 1
    DeployApp(root)
    root.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
