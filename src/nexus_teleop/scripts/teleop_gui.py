#!/usr/bin/env python3

import os
import shutil
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import ttk
from typing import Optional

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter


@dataclass(frozen=True)
class TunableParameter:
    node_name: str
    config_key: str
    name: str
    label: str
    section: str
    value_type: str
    min_value: Optional[float] = None
    max_value: Optional[float] = None
    step: Optional[float] = None
    odd_only: bool = False
    digits: int = 3


TUNING_PARAMETERS = (
    TunableParameter(
        node_name="traversability_to_map",
        config_key="/traversability_to_map",
        name="kernel_size",
        label="Height Range Kernel",
        section="Map Filtering",
        value_type="int",
        min_value=1,
        max_value=15,
        step=2,
        odd_only=True,
    ),
    TunableParameter(
        node_name="traversability_to_map",
        config_key="/traversability_to_map",
        name="clear_below_m",
        label="Free Threshold (m)",
        section="Map Filtering",
        value_type="float",
        min_value=0.0,
        max_value=0.20,
        step=0.005,
        digits=3,
    ),
    TunableParameter(
        node_name="traversability_to_map",
        config_key="/traversability_to_map",
        name="accumulate_from_m",
        label="Ramp Start (m)",
        section="Map Filtering",
        value_type="float",
        min_value=0.0,
        max_value=0.25,
        step=0.005,
        digits=3,
    ),
    TunableParameter(
        node_name="traversability_to_map",
        config_key="/traversability_to_map",
        name="full_at_m",
        label="Full Occupancy (m)",
        section="Map Filtering",
        value_type="float",
        min_value=0.01,
        max_value=0.40,
        step=0.01,
        digits=3,
    ),
    TunableParameter(
        node_name="traversability_to_map",
        config_key="/traversability_to_map",
        name="median_filter_size",
        label="Median Filter Size",
        section="Map Filtering",
        value_type="int",
        min_value=1,
        max_value=9,
        step=2,
        odd_only=True,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="batch_size",
        label="Batch Size",
        section="MPPI Sampling",
        value_type="int",
        min_value=32,
        max_value=2048,
        step=32,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="time_steps",
        label="Horizon Steps",
        section="MPPI Sampling",
        value_type="int",
        min_value=5,
        max_value=80,
        step=1,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="iteration_count",
        label="Inner Iterations",
        section="MPPI Sampling",
        value_type="int",
        min_value=1,
        max_value=8,
        step=1,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="temperature",
        label="Temperature",
        section="MPPI Sampling",
        value_type="float",
        min_value=0.01,
        max_value=2.0,
        step=0.01,
        digits=3,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="gamma",
        label="Control Prior Gamma",
        section="MPPI Sampling",
        value_type="float",
        min_value=0.0,
        max_value=0.20,
        step=0.001,
        digits=3,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="vx_std",
        label="Noise Sigma Vx",
        section="MPPI Sampling",
        value_type="float",
        min_value=0.01,
        max_value=1.5,
        step=0.01,
        digits=3,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="vy_std",
        label="Noise Sigma Vy",
        section="MPPI Sampling",
        value_type="float",
        min_value=0.01,
        max_value=1.5,
        step=0.01,
        digits=3,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="wz_std",
        label="Noise Sigma Wz",
        section="MPPI Sampling",
        value_type="float",
        min_value=0.01,
        max_value=1.5,
        step=0.01,
        digits=3,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="traversability_cost_weight",
        label="Traversability Weight",
        section="Terrain Costs",
        value_type="float",
        min_value=0.0,
        max_value=20.0,
        step=0.1,
        digits=2,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="variance_cost_weight",
        label="Variance Weight",
        section="Terrain Costs",
        value_type="float",
        min_value=0.0,
        max_value=10.0,
        step=0.1,
        digits=2,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="variance_full_scale",
        label="Variance Full Scale",
        section="Terrain Costs",
        value_type="float",
        min_value=0.001,
        max_value=0.50,
        step=0.005,
        digits=3,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="slope_cost_weight",
        label="Slope Weight",
        section="Terrain Costs",
        value_type="float",
        min_value=0.0,
        max_value=20.0,
        step=0.1,
        digits=2,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="slope_start_deg",
        label="Slope Start (deg)",
        section="Terrain Costs",
        value_type="float",
        min_value=0.0,
        max_value=45.0,
        step=1.0,
        digits=1,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="slope_max_deg",
        label="Slope Max (deg)",
        section="Terrain Costs",
        value_type="float",
        min_value=1.0,
        max_value=89.0,
        step=1.0,
        digits=1,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="unknown_cost_weight",
        label="Unknown Cost Weight",
        section="Terrain Costs",
        value_type="float",
        min_value=0.0,
        max_value=5.0,
        step=0.05,
        digits=2,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="unknown_is_obstacle",
        label="Unknown Is Obstacle",
        section="Terrain Costs",
        value_type="bool",
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="traversability_stop_threshold",
        label="Stop Threshold",
        section="Terrain Costs",
        value_type="float",
        min_value=0.0,
        max_value=1.0,
        step=0.01,
        digits=2,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="collision_cost",
        label="Collision Penalty",
        section="Terrain Costs",
        value_type="float",
        min_value=100.0,
        max_value=20000.0,
        step=100.0,
        digits=1,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="footprint_radius",
        label="Footprint Radius",
        section="Terrain Costs",
        value_type="float",
        min_value=0.0,
        max_value=1.5,
        step=0.01,
        digits=2,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="footprint_sample_count",
        label="Footprint Samples",
        section="Terrain Costs",
        value_type="int",
        min_value=0,
        max_value=32,
        step=1,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="goal_distance_weight",
        label="Goal Distance Weight",
        section="Goal && Control Costs",
        value_type="float",
        min_value=0.0,
        max_value=30.0,
        step=0.1,
        digits=2,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="goal_progress_weight",
        label="Goal Progress Weight",
        section="Goal && Control Costs",
        value_type="float",
        min_value=0.0,
        max_value=30.0,
        step=0.1,
        digits=2,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="goal_heading_weight",
        label="Goal Heading Weight",
        section="Goal && Control Costs",
        value_type="float",
        min_value=0.0,
        max_value=10.0,
        step=0.1,
        digits=2,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="goal_heading_activation_distance",
        label="Heading Gate Distance",
        section="Goal && Control Costs",
        value_type="float",
        min_value=0.05,
        max_value=5.0,
        step=0.05,
        digits=2,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="path_distance_weight",
        label="Path Tracking Weight",
        section="Goal && Control Costs",
        value_type="float",
        min_value=0.0,
        max_value=10.0,
        step=0.1,
        digits=2,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="control_effort_weight",
        label="Control Effort Weight",
        section="Goal && Control Costs",
        value_type="float",
        min_value=0.0,
        max_value=1.0,
        step=0.001,
        digits=3,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="control_smoothness_weight",
        label="Smoothness Weight",
        section="Goal && Control Costs",
        value_type="float",
        min_value=0.0,
        max_value=1.0,
        step=0.005,
        digits=3,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="twirling_weight",
        label="Twirling Weight",
        section="Goal && Control Costs",
        value_type="float",
        min_value=0.0,
        max_value=1.0,
        step=0.005,
        digits=3,
    ),
    TunableParameter(
        node_name="mppi_navigator",
        config_key="/mppi_navigator",
        name="prefer_forward_weight",
        label="Prefer Forward Weight",
        section="Goal && Control Costs",
        value_type="float",
        min_value=0.0,
        max_value=5.0,
        step=0.05,
        digits=2,
    ),
)


class TeleopGui(Node):
    def __init__(self) -> None:
        super().__init__("teleop_gui")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("linear_speed", 1.0)
        self.declare_parameter("strafe_speed", 1.0)
        self.declare_parameter("angular_speed", 1.2)
        self.declare_parameter("config_path", "")

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.default_linear_speed = float(self.get_parameter("linear_speed").value)
        self.default_strafe_speed = float(self.get_parameter("strafe_speed").value)
        self.default_angular_speed = float(self.get_parameter("angular_speed").value)
        self.initial_config_path = str(self.get_parameter("config_path").value).strip()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.get_clients = {}
        self.set_clients = {}
        for node_name in sorted({meta.node_name for meta in TUNING_PARAMETERS}):
            self.get_clients[node_name] = self.create_client(
                GetParameters,
                f"/{node_name}/get_parameters",
            )
            self.set_clients[node_name] = self.create_client(
                SetParameters,
                f"/{node_name}/set_parameters",
            )

        self.param_by_key = {self.param_key(meta): meta for meta in TUNING_PARAMETERS}
        self.section_order = []
        for meta in TUNING_PARAMETERS:
            if meta.section not in self.section_order:
                self.section_order.append(meta.section)

        self.linear_sign = 0
        self.strafe_sign = 0
        self.angular_sign = 0
        self.mouse_linear_sign = 0
        self.mouse_strafe_sign = 0
        self.mouse_angular_sign = 0

        self.root = tk.Tk()
        self.root.title("NEXUS Teleop & Tuning")
        self.root.geometry("1260x820")
        self.root.minsize(1080, 720)
        self.root.configure(bg="#101418")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.root.bind("<FocusOut>", self.on_focus_out)

        self.style = ttk.Style()
        try:
            self.style.theme_use("clam")
        except tk.TclError:
            pass
        self.style.configure("Root.TFrame", background="#101418")
        self.style.configure("Card.TFrame", background="#171d24")
        self.style.configure("Status.TLabel", background="#171d24", foreground="#d7dee7")
        self.style.configure("Title.TLabel", background="#101418", foreground="#eef2f7")
        self.style.configure("Caption.TLabel", background="#101418", foreground="#a9b4c0")
        self.style.configure("Section.TLabel", background="#171d24", foreground="#eef2f7")
        self.style.configure("Action.TButton", padding=10)
        self.style.configure("Field.TLabel", background="#171d24", foreground="#e5ebf4")
        self.style.configure("Toolbar.TCheckbutton", background="#101418", foreground="#d7dee7")

        self.linear_speed_var = tk.DoubleVar(value=self.default_linear_speed)
        self.strafe_speed_var = tk.DoubleVar(value=self.default_strafe_speed)
        self.angular_speed_var = tk.DoubleVar(value=self.default_angular_speed)
        self.status_var = tk.StringVar(value="idle")
        self.value_var = tk.StringVar(value="vx=0.00  vy=0.00  wz=0.00")
        self.config_path_var = tk.StringVar(value=self.initial_config_path)
        self.tuning_status_var = tk.StringVar(value="Load config or refresh live parameters.")
        self.node_status_var = tk.StringVar(value="Live nodes: waiting")
        self.auto_apply_var = tk.BooleanVar(value=True)

        self.param_vars = {}
        self.param_widgets = {}
        self.publish_timer = None

        self.build_ui()
        self.load_config_values()
        self.refresh_live_parameters()
        self.publish_timer = self.root.after(self.publish_period_ms, self.publish_loop)

        self.get_logger().info(
            "teleop gui ready: topic=%s publish_rate=%.1fHz config=%s"
            % (
                self.cmd_vel_topic,
                self.publish_rate,
                self.initial_config_path or "<none>",
            )
        )

    @staticmethod
    def param_key(meta: TunableParameter) -> str:
        return f"{meta.node_name}:{meta.name}"

    @property
    def publish_period_ms(self) -> int:
        return max(20, int(round(1000.0 / self.publish_rate)))

    def build_ui(self) -> None:
        outer = ttk.Frame(self.root, style="Root.TFrame", padding=16)
        outer.pack(fill="both", expand=True)

        header = ttk.Frame(outer, style="Root.TFrame")
        header.pack(fill="x")
        ttk.Label(
            header,
            text="NEXUS Teleop & MPPI Tuning",
            style="Title.TLabel",
            font=("TkDefaultFont", 18, "bold"),
        ).pack(anchor="w")
        ttk.Label(
            header,
            text=f"cmd_vel -> {self.cmd_vel_topic}",
            style="Caption.TLabel",
        ).pack(anchor="w", pady=(4, 10))

        body = ttk.Frame(outer, style="Root.TFrame")
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=0)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        self.build_teleop_panel(body)
        self.build_tuning_panel(body)

    def build_teleop_panel(self, parent: ttk.Frame) -> None:
        teleop_card = ttk.Frame(parent, style="Card.TFrame", padding=16)
        teleop_card.grid(row=0, column=0, sticky="nsw", padx=(0, 14))
        teleop_card.columnconfigure(0, weight=1)

        ttk.Label(
            teleop_card,
            text="Teleop",
            style="Section.TLabel",
            font=("TkDefaultFont", 15, "bold"),
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            teleop_card,
            text="W/S forward-back, A/D strafe, Q/E rotate, Space stop",
            style="Status.TLabel",
            wraplength=280,
            justify="left",
        ).grid(row=1, column=0, sticky="w", pady=(6, 12))

        controls = ttk.Frame(teleop_card, style="Card.TFrame")
        controls.grid(row=2, column=0, sticky="nsew")
        for column in range(3):
            controls.columnconfigure(column, weight=1)

        self.bind_button(controls, 0, 0, "Q", "angular", 1)
        self.bind_button(controls, 0, 1, "W", "linear", 1)
        self.bind_button(controls, 0, 2, "E", "angular", -1)
        self.bind_button(controls, 1, 0, "A", "strafe", 1)
        self.bind_button(controls, 1, 1, "STOP", "stop", 0)
        self.bind_button(controls, 1, 2, "D", "strafe", -1)
        self.bind_button(controls, 2, 1, "S", "linear", -1)

        tuning = ttk.Frame(teleop_card, style="Card.TFrame", padding=(0, 14, 0, 0))
        tuning.grid(row=3, column=0, sticky="ew")
        tuning.columnconfigure(0, weight=1)

        self.add_slider(tuning, "vx", self.linear_speed_var, 0.1, 3.0, 0)
        self.add_slider(tuning, "vy", self.strafe_speed_var, 0.1, 3.0, 1)
        self.add_slider(tuning, "wz", self.angular_speed_var, 0.1, 3.0, 2)

        ttk.Label(
            tuning,
            textvariable=self.status_var,
            style="Status.TLabel",
            font=("TkDefaultFont", 12, "bold"),
        ).grid(row=6, column=0, sticky="w", pady=(12, 4))
        ttk.Label(
            tuning,
            textvariable=self.value_var,
            style="Status.TLabel",
        ).grid(row=7, column=0, sticky="w")

    def build_tuning_panel(self, parent: ttk.Frame) -> None:
        panel = ttk.Frame(parent, style="Root.TFrame")
        panel.grid(row=0, column=1, sticky="nsew")
        panel.columnconfigure(0, weight=1)
        panel.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(panel, style="Card.TFrame", padding=16)
        toolbar.grid(row=0, column=0, sticky="ew")
        toolbar.columnconfigure(1, weight=1)

        ttk.Label(
            toolbar,
            text="Runtime Tuning",
            style="Section.TLabel",
            font=("TkDefaultFont", 15, "bold"),
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            toolbar,
            textvariable=self.node_status_var,
            style="Status.TLabel",
        ).grid(row=0, column=1, sticky="w", padx=(12, 0))

        ttk.Label(toolbar, text="Config", style="Field.TLabel").grid(row=1, column=0, sticky="w", pady=(12, 0))
        config_entry = ttk.Entry(toolbar, textvariable=self.config_path_var)
        config_entry.grid(row=1, column=1, sticky="ew", padx=(12, 12), pady=(12, 0))

        actions = ttk.Frame(toolbar, style="Card.TFrame")
        actions.grid(row=1, column=2, sticky="e", pady=(12, 0))
        ttk.Button(actions, text="Load Config", command=self.load_config_values).pack(side="left", padx=(0, 8))
        ttk.Button(actions, text="Refresh Live", command=self.refresh_live_parameters).pack(side="left", padx=(0, 8))
        ttk.Button(actions, text="Apply Live", command=self.apply_all_parameters).pack(side="left", padx=(0, 8))
        ttk.Button(actions, text="Save YAML", command=self.save_config_values).pack(side="left")

        options = ttk.Frame(toolbar, style="Card.TFrame")
        options.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(12, 0))
        ttk.Checkbutton(
            options,
            text="Auto apply on Enter / focus change",
            variable=self.auto_apply_var,
            style="Toolbar.TCheckbutton",
        ).pack(side="left")
        ttk.Label(
            options,
            textvariable=self.tuning_status_var,
            style="Status.TLabel",
        ).pack(side="left", padx=(16, 0))

        scroll_host = ttk.Frame(panel, style="Card.TFrame", padding=0)
        scroll_host.grid(row=1, column=0, sticky="nsew", pady=(14, 0))
        scroll_host.columnconfigure(0, weight=1)
        scroll_host.rowconfigure(0, weight=1)

        canvas = tk.Canvas(
            scroll_host,
            background="#101418",
            highlightthickness=0,
            bd=0,
        )
        scrollbar = ttk.Scrollbar(scroll_host, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        self.scroll_body = ttk.Frame(canvas, style="Root.TFrame", padding=14)
        self.scroll_window = canvas.create_window((0, 0), window=self.scroll_body, anchor="nw")

        def on_canvas_configure(event: tk.Event) -> None:
            canvas.itemconfigure(self.scroll_window, width=event.width)

        def on_frame_configure(_event: tk.Event) -> None:
            canvas.configure(scrollregion=canvas.bbox("all"))

        def on_mousewheel(event: tk.Event) -> None:
            if getattr(event, "delta", 0):
                canvas.yview_scroll(int(-event.delta / 120), "units")
            elif getattr(event, "num", None) == 4:
                canvas.yview_scroll(-1, "units")
            elif getattr(event, "num", None) == 5:
                canvas.yview_scroll(1, "units")

        canvas.bind("<Configure>", on_canvas_configure)
        self.scroll_body.bind("<Configure>", on_frame_configure)
        canvas.bind("<MouseWheel>", on_mousewheel)
        canvas.bind("<Button-4>", on_mousewheel)
        canvas.bind("<Button-5>", on_mousewheel)
        self.scroll_body.bind("<MouseWheel>", on_mousewheel)
        self.scroll_body.bind("<Button-4>", on_mousewheel)
        self.scroll_body.bind("<Button-5>", on_mousewheel)

        self.build_tuning_sections()

    def build_tuning_sections(self) -> None:
        for section in self.section_order:
            card = ttk.Frame(self.scroll_body, style="Card.TFrame", padding=14)
            card.pack(fill="x", pady=(0, 12))
            card.columnconfigure(1, weight=1)

            ttk.Label(
                card,
                text=section,
                style="Section.TLabel",
                font=("TkDefaultFont", 13, "bold"),
            ).grid(row=0, column=0, sticky="w", pady=(0, 10))
            ttk.Label(
                card,
                text="Live parameters write directly to the active ROS2 nodes.",
                style="Status.TLabel",
            ).grid(row=0, column=1, sticky="w", pady=(0, 10))

            row = 1
            for meta in TUNING_PARAMETERS:
                if meta.section != section:
                    continue

                key = self.param_key(meta)
                ttk.Label(card, text=meta.label, style="Field.TLabel").grid(row=row, column=0, sticky="w", pady=4)

                if meta.value_type == "bool":
                    variable = tk.BooleanVar(value=False)
                    widget = ttk.Checkbutton(
                        card,
                        variable=variable,
                        style="Toolbar.TCheckbutton",
                        command=lambda param_key=key: self.on_parameter_widget_commit(param_key),
                    )
                    widget.grid(row=row, column=1, sticky="w", pady=4)
                else:
                    variable = tk.StringVar(value="")
                    spinbox_kwargs = {
                        "textvariable": variable,
                        "width": 14,
                        "command": lambda param_key=key: self.on_parameter_widget_commit(param_key),
                    }
                    if meta.min_value is not None:
                        spinbox_kwargs["from_"] = meta.min_value
                    if meta.max_value is not None:
                        spinbox_kwargs["to"] = meta.max_value
                    if meta.step is not None:
                        spinbox_kwargs["increment"] = meta.step
                    widget = ttk.Spinbox(card, **spinbox_kwargs)
                    widget.grid(row=row, column=1, sticky="w", pady=4)
                    widget.bind(
                        "<Return>",
                        lambda _event, param_key=key: self.on_parameter_widget_commit(param_key),
                    )
                    widget.bind(
                        "<FocusOut>",
                        lambda _event, param_key=key: self.on_parameter_widget_commit(param_key),
                    )

                self.param_vars[key] = variable
                self.param_widgets[key] = widget

                range_text = self.describe_range(meta)
                ttk.Label(card, text=range_text, style="Status.TLabel").grid(row=row, column=2, sticky="w", padx=(12, 0))
                row += 1

    def add_slider(
        self,
        parent: ttk.Frame,
        label: str,
        variable: tk.DoubleVar,
        min_value: float,
        max_value: float,
        row: int,
    ) -> None:
        ttk.Label(parent, text=label, style="Status.TLabel").grid(row=row * 2, column=0, sticky="w")
        scale = ttk.Scale(parent, from_=min_value, to=max_value, variable=variable)
        scale.grid(row=row * 2 + 1, column=0, sticky="ew", pady=(4, 12))

    def bind_button(self, parent: ttk.Frame, row: int, col: int, text: str, axis: str, sign: int) -> None:
        button = ttk.Button(parent, text=text, style="Action.TButton")
        button.grid(row=row, column=col, padx=4, pady=4, sticky="nsew")

        if axis == "stop":
            button.configure(command=self.stop_motion)
            return

        button.bind("<ButtonPress-1>", lambda _event, a=axis, s=sign: self.set_mouse_axis(a, s))
        button.bind("<ButtonRelease-1>", lambda _event, a=axis: self.clear_mouse_axis(a))
        button.bind("<Leave>", lambda _event, a=axis: self.clear_mouse_axis(a))

    def describe_range(self, meta: TunableParameter) -> str:
        if meta.value_type == "bool":
            return f"{meta.node_name}"
        min_text = f"{meta.min_value:g}" if meta.min_value is not None else "any"
        max_text = f"{meta.max_value:g}" if meta.max_value is not None else "any"
        extra = " odd" if meta.odd_only else ""
        return f"{meta.node_name}  {min_text} .. {max_text}{extra}"

    def on_parameter_widget_commit(self, key: str) -> None:
        if not self.auto_apply_var.get():
            return
        self.apply_parameters([key])

    def set_mouse_axis(self, axis: str, sign: int) -> None:
        if axis == "linear":
            self.mouse_linear_sign = sign
        elif axis == "strafe":
            self.mouse_strafe_sign = sign
        elif axis == "angular":
            self.mouse_angular_sign = sign

    def clear_mouse_axis(self, axis: str) -> None:
        if axis == "linear":
            self.mouse_linear_sign = 0
        elif axis == "strafe":
            self.mouse_strafe_sign = 0
        elif axis == "angular":
            self.mouse_angular_sign = 0

    def stop_motion(self) -> None:
        self.linear_sign = 0
        self.strafe_sign = 0
        self.angular_sign = 0
        self.mouse_linear_sign = 0
        self.mouse_strafe_sign = 0
        self.mouse_angular_sign = 0
        self.publish_once()

    def on_key_press(self, event: tk.Event) -> None:
        keysym = (event.keysym or "").lower()
        if keysym in ("w", "up"):
            self.linear_sign = 1
        elif keysym in ("s", "down"):
            self.linear_sign = -1
        elif keysym in ("a", "left"):
            self.strafe_sign = 1
        elif keysym in ("d", "right"):
            self.strafe_sign = -1
        elif keysym == "q":
            self.angular_sign = 1
        elif keysym == "e":
            self.angular_sign = -1
        elif keysym in ("space", "x"):
            self.stop_motion()

    def on_key_release(self, event: tk.Event) -> None:
        keysym = (event.keysym or "").lower()
        if keysym in ("w", "up", "s", "down"):
            self.linear_sign = 0
        elif keysym in ("a", "left", "d", "right"):
            self.strafe_sign = 0
        elif keysym in ("q", "e"):
            self.angular_sign = 0

    def on_focus_out(self, _event: tk.Event) -> None:
        self.stop_motion()

    def current_axes(self) -> tuple[float, float, float]:
        linear_sign = self.mouse_linear_sign or self.linear_sign
        strafe_sign = self.mouse_strafe_sign or self.strafe_sign
        angular_sign = self.mouse_angular_sign or self.angular_sign

        vx = linear_sign * float(self.linear_speed_var.get())
        vy = strafe_sign * float(self.strafe_speed_var.get())
        wz = angular_sign * float(self.angular_speed_var.get())
        return vx, vy, wz

    def publish_once(self) -> None:
        vx, vy, wz = self.current_axes()
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.cmd_pub.publish(msg)

        active = []
        if abs(vx) > 1e-6:
            active.append("vx")
        if abs(vy) > 1e-6:
            active.append("vy")
        if abs(wz) > 1e-6:
            active.append("wz")
        self.status_var.set(" + ".join(active) if active else "idle")
        self.value_var.set(f"vx={vx:+.2f}  vy={vy:+.2f}  wz={wz:+.2f}")

    def publish_loop(self) -> None:
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except (ExternalShutdownException, RuntimeError):
            self.on_close()
            return

        self.publish_once()
        self.publish_timer = self.root.after(self.publish_period_ms, self.publish_loop)

    def apply_all_parameters(self) -> None:
        self.apply_parameters(list(self.param_by_key.keys()))

    def apply_parameters(self, keys: list[str]) -> None:
        grouped = {}
        for key in keys:
            meta = self.param_by_key[key]
            try:
                value = self.read_widget_value(meta)
            except ValueError as exc:
                self.tuning_status_var.set(f"{meta.label}: {exc}")
                return
            grouped.setdefault(meta.node_name, []).append((meta, value))

        for node_name, entries in grouped.items():
            client = self.set_clients[node_name]
            if not client.service_is_ready():
                self.tuning_status_var.set(f"{node_name}: parameter service is not ready.")
                continue

            request = SetParameters.Request()
            for meta, value in entries:
                request.parameters.append(Parameter(meta.name, value=value).to_parameter_msg())

            future = client.call_async(request)
            future.add_done_callback(
                lambda future, node_name=node_name, entries=entries: self.on_apply_done(
                    node_name,
                    entries,
                    future,
                )
            )

    def on_apply_done(self, node_name: str, entries, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.tuning_status_var.set(f"{node_name}: apply failed ({exc}).")
            return

        failures = []
        for (meta, _value), result in zip(entries, response.results):
            if not result.successful:
                failures.append(f"{meta.name}: {result.reason}")
        if failures:
            self.tuning_status_var.set(f"{node_name}: " + " | ".join(failures))
            return

        applied_names = ", ".join(meta.name for meta, _value in entries)
        self.tuning_status_var.set(f"{node_name}: applied {applied_names}")

    def refresh_live_parameters(self) -> None:
        ready_nodes = []
        waiting_nodes = []
        grouped_names = {}
        for meta in TUNING_PARAMETERS:
            grouped_names.setdefault(meta.node_name, []).append(meta.name)

        for node_name, names in grouped_names.items():
            client = self.get_clients[node_name]
            if not client.service_is_ready():
                waiting_nodes.append(node_name)
                continue

            request = GetParameters.Request()
            request.names = names
            future = client.call_async(request)
            future.add_done_callback(
                lambda future, node_name=node_name, names=names: self.on_refresh_done(
                    node_name,
                    names,
                    future,
                )
            )
            ready_nodes.append(node_name)

        if ready_nodes or waiting_nodes:
            status_chunks = []
            if ready_nodes:
                status_chunks.append("ready: " + ", ".join(ready_nodes))
            if waiting_nodes:
                status_chunks.append("waiting: " + ", ".join(waiting_nodes))
            self.node_status_var.set("Live nodes: " + " | ".join(status_chunks))
        else:
            self.node_status_var.set("Live nodes: none configured")

    def on_refresh_done(self, node_name: str, names: list[str], future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.tuning_status_var.set(f"{node_name}: refresh failed ({exc}).")
            return

        for name, value_msg in zip(names, response.values):
            key = f"{node_name}:{name}"
            meta = self.param_by_key.get(key)
            if meta is None:
                continue
            value = self.parameter_value_to_python(value_msg)
            self.write_widget_value(meta, value)

        self.tuning_status_var.set(f"{node_name}: live values refreshed")

    def read_widget_value(self, meta: TunableParameter):
        key = self.param_key(meta)
        variable = self.param_vars[key]

        if meta.value_type == "bool":
            return bool(variable.get())

        raw_text = str(variable.get()).strip()
        if raw_text == "":
            raise ValueError("value is empty")

        if meta.value_type == "int":
            value = int(float(raw_text))
            if meta.odd_only and value % 2 == 0:
                raise ValueError("must be an odd integer")
        else:
            value = float(raw_text)

        if meta.min_value is not None and value < meta.min_value:
            raise ValueError(f"must be >= {meta.min_value}")
        if meta.max_value is not None and value > meta.max_value:
            raise ValueError(f"must be <= {meta.max_value}")

        if meta.name == "accumulate_from_m":
            clear_value = float(self.param_vars["traversability_to_map:clear_below_m"].get())
            full_value = float(self.param_vars["traversability_to_map:full_at_m"].get())
            if not (clear_value <= value < full_value):
                raise ValueError("must satisfy clear_below_m <= accumulate_from_m < full_at_m")
        elif meta.name == "clear_below_m":
            accumulate_value = float(self.param_vars["traversability_to_map:accumulate_from_m"].get())
            if value > accumulate_value:
                raise ValueError("must be <= accumulate_from_m")
        elif meta.name == "full_at_m":
            accumulate_value = float(self.param_vars["traversability_to_map:accumulate_from_m"].get())
            if value <= accumulate_value:
                raise ValueError("must be > accumulate_from_m")
        elif meta.name == "slope_start_deg":
            slope_max_value = float(self.param_vars["mppi_navigator:slope_max_deg"].get())
            if value >= slope_max_value:
                raise ValueError("must be < slope_max_deg")
        elif meta.name == "slope_max_deg":
            slope_start_value = float(self.param_vars["mppi_navigator:slope_start_deg"].get())
            if value <= slope_start_value:
                raise ValueError("must be > slope_start_deg")

        return value

    def write_widget_value(self, meta: TunableParameter, value) -> None:
        key = self.param_key(meta)
        variable = self.param_vars[key]
        if meta.value_type == "bool":
            variable.set(bool(value))
            return
        if meta.value_type == "int":
            variable.set(str(int(value)))
            return
        format_string = "{:." + str(meta.digits) + "f}"
        variable.set(format_string.format(float(value)))

    def load_config_values(self) -> None:
        config_path_text = self.config_path_var.get().strip()
        if not config_path_text:
            self.tuning_status_var.set("Config path is empty.")
            return
        config_path = Path(config_path_text)
        if not config_path.exists():
            self.tuning_status_var.set(f"Config not found: {config_path}")
            return

        try:
            with config_path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except Exception as exc:
            self.tuning_status_var.set(f"Failed to read config: {exc}")
            return

        loaded = 0
        for meta in TUNING_PARAMETERS:
            section = data.get(meta.config_key, {})
            params = section.get("ros__parameters", {})
            if meta.name not in params:
                continue
            self.write_widget_value(meta, params[meta.name])
            loaded += 1

        self.tuning_status_var.set(f"Loaded {loaded} values from {config_path}")

    def save_config_values(self) -> None:
        config_path_text = self.config_path_var.get().strip()
        if not config_path_text:
            self.tuning_status_var.set("Config path is empty.")
            return

        config_path = Path(config_path_text)
        data = {}
        if config_path.exists():
            try:
                with config_path.open("r", encoding="utf-8") as handle:
                    data = yaml.safe_load(handle) or {}
            except Exception as exc:
                self.tuning_status_var.set(f"Failed to read config: {exc}")
                return

        try:
            current_values = {
                self.param_key(meta): self.read_widget_value(meta)
                for meta in TUNING_PARAMETERS
            }
        except ValueError as exc:
            self.tuning_status_var.set(f"Save aborted: {exc}")
            return

        for meta in TUNING_PARAMETERS:
            section = data.setdefault(meta.config_key, {})
            ros_params = section.setdefault("ros__parameters", {})
            ros_params[meta.name] = current_values[self.param_key(meta)]

        trav_params = data.get("/traversability_to_map", {}).get("ros__parameters", {})
        trav_params.pop("gaussian_filter_size", None)
        trav_params.pop("gaussian_sigma", None)

        try:
            config_path.parent.mkdir(parents=True, exist_ok=True)
            if config_path.exists():
                backup_path = config_path.with_suffix(config_path.suffix + ".bak")
                shutil.copyfile(config_path, backup_path)
            temp_path = config_path.with_suffix(config_path.suffix + ".tmp")
            with temp_path.open("w", encoding="utf-8") as handle:
                yaml.safe_dump(
                    data,
                    handle,
                    sort_keys=False,
                    default_flow_style=False,
                    allow_unicode=False,
                )
            os.replace(temp_path, config_path)
        except Exception as exc:
            self.tuning_status_var.set(f"Failed to save config: {exc}")
            return

        self.tuning_status_var.set(f"Saved YAML to {config_path}")

    @staticmethod
    def parameter_value_to_python(value_msg):
        if value_msg.type == ParameterType.PARAMETER_BOOL:
            return value_msg.bool_value
        if value_msg.type == ParameterType.PARAMETER_INTEGER:
            return value_msg.integer_value
        if value_msg.type == ParameterType.PARAMETER_DOUBLE:
            return value_msg.double_value
        if value_msg.type == ParameterType.PARAMETER_STRING:
            return value_msg.string_value
        return None

    def on_close(self) -> None:
        try:
            if self.publish_timer is not None:
                self.root.after_cancel(self.publish_timer)
                self.publish_timer = None
        except tk.TclError:
            pass

        try:
            zero = Twist()
            self.cmd_pub.publish(zero)
        except Exception:
            pass

        try:
            self.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

        try:
            self.root.destroy()
        except tk.TclError:
            pass


def main() -> None:
    rclpy.init()
    node = TeleopGui()
    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.on_close()


if __name__ == "__main__":
    main()
