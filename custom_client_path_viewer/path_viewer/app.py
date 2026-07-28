from __future__ import annotations

import queue
import tkinter as tk
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, ttk

from PIL import Image, ImageDraw, ImageTk

from .config import AppConfig
from .mqtt_receiver import MqttPathSubscriber
from .protocol import DecodedPath, decode_robot_path_plan, make_demo_payload
from .renderer import BACKGROUND, MUTED, TEXT, PathRenderer


class PathViewerApp:
    def __init__(self, root: tk.Tk, config: AppConfig, demo: bool = False):
        self.root = root
        self.config = config
        self.current_path: DecodedPath | None = None
        self.selected_index: int | None = None
        self.message_count = 0
        self._render_job: str | None = None
        self._tk_image: ImageTk.PhotoImage | None = None
        self._events: queue.Queue[tuple[str, object]] = queue.Queue()

        self.renderer = PathRenderer(config.map)
        self.subscriber = MqttPathSubscriber(
            config.mqtt,
            on_path=lambda path: self._events.put(("path", path)),
            on_status=lambda level, message: self._events.put(
                ("status", (level, message))
            ),
        )

        self.root.title("RoboMaster 2026 哨兵轨迹查看器")
        self.root.geometry("1420x820")
        self.root.minsize(1050, 650)
        self.root.configure(background=BACKGROUND)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        self._build_style()
        self._build_ui()
        self.root.after(50, self._poll_events)
        self.root.after(150, self._render)

        if demo:
            self.root.after(250, self.show_demo)

    def _build_style(self) -> None:
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Root.TFrame", background=BACKGROUND)
        style.configure("Panel.TFrame", background="#0e2131")
        style.configure(
            "Heading.TLabel",
            background=BACKGROUND,
            foreground=TEXT,
            font=("Segoe UI", 15, "bold"),
        )
        style.configure(
            "Body.TLabel",
            background="#0e2131",
            foreground=TEXT,
            font=("Segoe UI", 10),
        )
        style.configure(
            "Muted.TLabel",
            background="#0e2131",
            foreground=MUTED,
            font=("Segoe UI", 9),
        )
        style.configure(
            "Status.TLabel",
            background=BACKGROUND,
            foreground="#8aa1ad",
            font=("Segoe UI", 9),
        )
        style.configure(
            "Accent.TButton",
            font=("Segoe UI", 9, "bold"),
            padding=(12, 7),
        )
        style.configure("TButton", padding=(10, 7))
        style.configure(
            "Treeview",
            background="#102635",
            fieldbackground="#102635",
            foreground=TEXT,
            rowheight=25,
            borderwidth=0,
        )
        style.configure(
            "Treeview.Heading",
            background="#18384a",
            foreground=TEXT,
            font=("Segoe UI", 9, "bold"),
        )
        style.map(
            "Treeview",
            background=[("selected", "#2f7797")],
            foreground=[("selected", "#ffffff")],
        )
        style.configure(
            "Dark.TCheckbutton",
            background=BACKGROUND,
            foreground=TEXT,
            font=("Segoe UI", 9),
        )
        style.map(
            "Dark.TCheckbutton",
            background=[("active", BACKGROUND)],
            foreground=[("active", "#ffffff")],
        )

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, style="Root.TFrame", padding=12)
        outer.pack(fill=tk.BOTH, expand=True)

        toolbar = ttk.Frame(outer, style="Root.TFrame")
        toolbar.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(
            toolbar,
            text="哨兵轨迹规划",
            style="Heading.TLabel",
        ).pack(side=tk.LEFT, padx=(2, 18))

        self.connect_button = ttk.Button(
            toolbar,
            text="连接裁判服务器",
            style="Accent.TButton",
            command=self.subscriber.connect,
        )
        self.connect_button.pack(side=tk.LEFT, padx=4)
        ttk.Button(
            toolbar,
            text="断开",
            command=self.subscriber.disconnect,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            toolbar,
            text="演示轨迹",
            command=self.show_demo,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            toolbar,
            text="载入场地图",
            command=self._choose_background,
        ).pack(side=tk.LEFT, padx=4)

        self.show_all_labels = tk.BooleanVar(
            value=self.config.map.show_all_labels
        )
        ttk.Checkbutton(
            toolbar,
            text="标注全部点号",
            variable=self.show_all_labels,
            style="Dark.TCheckbutton",
            command=self._schedule_render,
        ).pack(side=tk.LEFT, padx=(14, 4))

        endpoint = (
            f"{self.config.mqtt.host}:{self.config.mqtt.port}  "
            f"topic={self.config.mqtt.topic}  "
            f"clientID={self.config.mqtt.client_id}"
        )
        ttk.Label(
            toolbar,
            text=endpoint,
            style="Status.TLabel",
        ).pack(side=tk.RIGHT, padx=4)

        paned = ttk.Panedwindow(outer, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True)

        map_panel = tk.Frame(paned, background=BACKGROUND)
        side_panel = ttk.Frame(paned, style="Panel.TFrame", padding=12)
        paned.add(map_panel, weight=4)
        paned.add(side_panel, weight=1)

        self.map_label = tk.Label(
            map_panel,
            background=BACKGROUND,
            foreground=MUTED,
            text="等待 RobotPathPlanInfo...",
            font=("Segoe UI", 12),
        )
        self.map_label.pack(fill=tk.BOTH, expand=True)
        self.map_label.bind("<Configure>", lambda event: self._schedule_render())

        self.status_var = tk.StringVar(value="尚未连接")
        self.last_message_var = tk.StringVar(value="-")
        self.intention_var = tk.StringVar(value="-")
        self.sender_var = tk.StringVar(value="-")
        self.point_count_var = tk.StringVar(value="0")
        self.out_of_bounds_var = tk.StringVar(value="0")

        ttk.Label(
            side_panel,
            text="接收状态",
            style="Body.TLabel",
            font=("Segoe UI", 11, "bold"),
        ).pack(anchor=tk.W)
        self.status_label = tk.Label(
            side_panel,
            textvariable=self.status_var,
            background="#0e2131",
            foreground="#8aa1ad",
            justify=tk.LEFT,
            anchor=tk.W,
            wraplength=320,
            font=("Segoe UI", 9),
        )
        self.status_label.pack(fill=tk.X, pady=(5, 12))

        info = ttk.Frame(side_panel, style="Panel.TFrame")
        info.pack(fill=tk.X, pady=(0, 10))
        rows = [
            ("最近消息", self.last_message_var),
            ("意图", self.intention_var),
            ("发送者", self.sender_var),
            ("点数", self.point_count_var),
            ("场外点", self.out_of_bounds_var),
        ]
        for row, (label, variable) in enumerate(rows):
            ttk.Label(info, text=label, style="Muted.TLabel").grid(
                row=row,
                column=0,
                sticky=tk.W,
                padx=(0, 12),
                pady=2,
            )
            ttk.Label(info, textvariable=variable, style="Body.TLabel").grid(
                row=row,
                column=1,
                sticky=tk.W,
                pady=2,
            )

        ttk.Separator(side_panel).pack(fill=tk.X, pady=(2, 10))
        ttk.Label(
            side_panel,
            text="点位坐标（双击/选择可高亮）",
            style="Body.TLabel",
            font=("Segoe UI", 10, "bold"),
        ).pack(anchor=tk.W, pady=(0, 7))

        tree_frame = ttk.Frame(side_panel, style="Panel.TFrame")
        tree_frame.pack(fill=tk.BOTH, expand=True)
        self.point_tree = ttk.Treeview(
            tree_frame,
            columns=("point", "x_dm", "y_dm", "x_m", "y_m"),
            show="headings",
            selectmode="browse",
        )
        headings = [
            ("point", "点", 44),
            ("x_dm", "X/dm", 58),
            ("y_dm", "Y/dm", 58),
            ("x_m", "X/m", 58),
            ("y_m", "Y/m", 58),
        ]
        for column, title, width in headings:
            self.point_tree.heading(column, text=title)
            self.point_tree.column(column, width=width, anchor=tk.CENTER)
        scrollbar = ttk.Scrollbar(
            tree_frame,
            orient=tk.VERTICAL,
            command=self.point_tree.yview,
        )
        self.point_tree.configure(yscrollcommand=scrollbar.set)
        self.point_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.point_tree.bind("<<TreeviewSelect>>", self._select_point)

        background_label = (
            Path(self.config.map.background_image).name
            if self.config.map.background_image
            else "坐标网格"
        )
        self.footer_var = tk.StringVar(
            value=(
                f"场地 {self.config.map.field_width_dm / 10:g} m × "
                f"{self.config.map.field_height_dm / 10:g} m；"
                f"背景 {background_label}；"
                "原点在左下角，+X 向右，+Y 向上；"
                "橙色点表示坐标越界并钳制到边缘显示"
            )
        )
        ttk.Label(
            outer,
            textvariable=self.footer_var,
            style="Status.TLabel",
        ).pack(fill=tk.X, pady=(8, 0))

    def _poll_events(self) -> None:
        try:
            while True:
                event_type, value = self._events.get_nowait()
                if event_type == "path":
                    self._show_path(value)
                elif event_type == "status":
                    level, message = value
                    self._show_status(level, message)
        except queue.Empty:
            pass
        self.root.after(50, self._poll_events)

    def _show_status(self, level: str, message: str) -> None:
        colors = {
            "connected": "#59d98e",
            "subscribed": "#59d98e",
            "message": "#59d98e",
            "connecting": "#ffcf66",
            "disconnected": "#8aa1ad",
            "error": "#ff6b75",
        }
        self.status_var.set(message)
        self.status_label.configure(foreground=colors.get(level, "#8aa1ad"))

    def _show_path(self, path: DecodedPath) -> None:
        self.current_path = path
        self.selected_index = None
        self.message_count += 1
        self.last_message_var.set(
            f"{datetime.now().strftime('%H:%M:%S')}  #{self.message_count}"
        )
        self.intention_var.set(f"{path.intention} - {path.intention_name}")
        self.sender_var.set(str(path.sender_id))
        self.point_count_var.set(str(len(path.points)))
        out_of_bounds = sum(
            1
            for point in path.points
            if not (
                0 <= point.x_dm <= self.config.map.field_width_dm
                and 0 <= point.y_dm <= self.config.map.field_height_dm
            )
        )
        self.out_of_bounds_var.set(str(out_of_bounds))

        for item in self.point_tree.get_children():
            self.point_tree.delete(item)
        for point in path.points:
            self.point_tree.insert(
                "",
                tk.END,
                iid=str(point.index),
                values=(
                    f"P{point.index}",
                    point.x_dm,
                    point.y_dm,
                    f"{point.x_m:.1f}",
                    f"{point.y_m:.1f}",
                ),
            )
        self._schedule_render()

    def show_demo(self) -> None:
        path = decode_robot_path_plan(make_demo_payload())
        self._show_path(path)
        self._show_status(
            "message",
            "当前为离线演示轨迹；点击“连接裁判服务器”接收真实数据",
        )

    def _select_point(self, event=None) -> None:
        del event
        selected = self.point_tree.selection()
        if not selected:
            return
        self.selected_index = int(selected[0])
        self._schedule_render()

    def _choose_background(self) -> None:
        filename = filedialog.askopenfilename(
            title="选择已裁剪到场地边界的地图图片",
            filetypes=[
                ("Image files", "*.png *.jpg *.jpeg *.bmp *.webp"),
                ("All files", "*.*"),
            ],
        )
        if not filename:
            return
        self.config = replace(
            self.config,
            map=replace(self.config.map, background_image=filename),
        )
        self.renderer = PathRenderer(self.config.map)
        self.footer_var.set(
            f"场地图：{Path(filename).name}（本次运行有效；持久化请修改 config.json）"
        )
        self._schedule_render()

    def _schedule_render(self) -> None:
        if self._render_job is not None:
            self.root.after_cancel(self._render_job)
        self._render_job = self.root.after(100, self._render)

    def _render(self) -> None:
        self._render_job = None
        if self.current_path is None:
            width = max(self.map_label.winfo_width(), 640)
            height = max(self.map_label.winfo_height(), 420)
            image = Image.new("RGB", (width, height), BACKGROUND)
            draw = ImageDraw.Draw(image)
            message = "Waiting for RobotPathPlanInfo..."
            draw.text((width // 2 - 110, height // 2), message, fill=MUTED)
        else:
            image = self.renderer.render(
                self.current_path,
                width=max(self.map_label.winfo_width(), 640),
                height=max(self.map_label.winfo_height(), 420),
                selected_index=self.selected_index,
                show_all_labels=self.show_all_labels.get(),
            )
        self._tk_image = ImageTk.PhotoImage(image)
        self.map_label.configure(image=self._tk_image, text="")

    def _close(self) -> None:
        self.subscriber.disconnect()
        self.root.destroy()
