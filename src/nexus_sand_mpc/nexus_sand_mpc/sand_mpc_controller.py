from __future__ import annotations

import math
import warnings
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class SandMpcObservation:
    timestamp: float
    v: float
    w: float


class SandMpcControllerMIMO:
    """MIMO sand-slip MPC migrated from the sand_sim MATLAB controller.

    The optimization itself is delegated to do-mpc/CasADi. This class only
    defines the plant model, keeps delay queues, and replays delayed
    observations so the estimate remains time-consistent.
    """

    def __init__(
        self,
        horizon: int = 25,
        dt_nominal: float = 1e-2,
        cmd_delay: float = 0.05,
        drive_tau: float = 0.08,
        turn_tau: float = 0.08,
        q_v: float = 120.0,
        q_int_v: float = 12.0,
        r_du_v: float = 0.30,
        q_w: float = 80.0,
        q_int_w: float = 6.0,
        r_du_w: float = 0.50,
        slip_alpha: float = 0.30,
        slip_init: float = 0.50,
        correction_gain: float = 0.40,
        v_max: float = 1.50,
        w_max: float = 1.40,
    ) -> None:
        self.horizon = int(horizon)
        self.dt_nominal = float(dt_nominal)
        self.cmd_delay = max(0.0, float(cmd_delay))
        self.delay_steps = max(0, int(round(self.cmd_delay / self.dt_nominal)))
        self.drive_tau = float(drive_tau)
        self.turn_tau = float(turn_tau)
        self.q_v = float(q_v)
        self.q_int_v = float(q_int_v)
        self.r_du_v = float(r_du_v)
        self.q_w = float(q_w)
        self.q_int_w = float(q_int_w)
        self.r_du_w = float(r_du_w)
        self.slip_alpha = float(slip_alpha)
        self.slip_init = float(slip_init)
        self.correction_gain = float(np.clip(correction_gain, 0.0, 1.0))
        self.v_max = float(v_max)
        self.w_max = float(w_max)

        self.n_x = 4 + 2 * self.delay_steps
        self._tvp_v_ref = 0.0
        self._tvp_w_ref = 0.0
        self._tvp_gain = max(0.20, 1.0 - self.slip_init)
        self.model, self.mpc = self._build_mpc()
        self.reset()

    def reset(self) -> None:
        self.v_ref = 0.0
        self.w_ref = 0.0
        self.x_hat = np.zeros(self.n_x, dtype=float)
        self.current_tick = 0
        self.last_v_cmd = 0.0
        self.last_w_cmd = 0.0
        self.slip_est = self.slip_init
        self._tvp_v_ref = 0.0
        self._tvp_w_ref = 0.0
        self._tvp_gain = max(0.20, 1.0 - self.slip_init)

        self.step_cmds: list[tuple[float, float]] = []
        self.step_refs: list[tuple[float, float]] = []
        self.step_gains: list[float] = []
        self.snapshot_states: list[np.ndarray] = [self.x_hat.copy()]

        self.pending_obs: Optional[SandMpcObservation] = None

        self.mpc.reset_history()
        self.mpc.t0 = 0.0
        self.mpc.x0 = self.x_hat.reshape(-1, 1)
        self.mpc.u0 = np.array([[0.0], [0.0]])
        self.mpc.set_initial_guess()

    def set_reference(self, v_ref: float, w_ref: float) -> None:
        self.v_ref = float(np.clip(v_ref, 0.0, self.v_max))
        self.w_ref = float(np.clip(w_ref, -self.w_max, self.w_max))

    def receive_observation(self, obs: SandMpcObservation) -> None:
        self.pending_obs = obs

        cmd_at_measure = self._command_at(obs.timestamp - self.cmd_delay, "v")
        if cmd_at_measure > 0.1:
            raw_slip = 1.0 - (float(obs.v) / cmd_at_measure)
            raw_slip = min(max(raw_slip, 0.0), 0.60)
            self.slip_est += self.slip_alpha * (raw_slip - self.slip_est)

    def compute(self, t_now: float) -> tuple[float, float]:
        tick_now = self._tick_from_time(t_now)
        self._sync_to_tick(tick_now)

        if self.pending_obs is not None:
            self.x_hat = self._replay_latest_observation(tick_now)
            self.snapshot_states[self.current_tick] = self.x_hat.copy()
            self.pending_obs = None

        gain = max(0.20, 1.0 - self.slip_est)
        self._tvp_v_ref = self.v_ref
        self._tvp_w_ref = self.w_ref
        self._tvp_gain = gain

        self.mpc.t0 = float(t_now)
        self.mpc.x0 = self.x_hat.reshape(-1, 1)
        self.mpc.u0 = np.array([[self.last_v_cmd], [self.last_w_cmd]])
        u_arr = self.mpc.make_step(self.x_hat.reshape(-1, 1))
        u = np.asarray(u_arr, dtype=float).reshape(-1)

        v_cmd = float(np.clip(u[0], 0.0, self.v_max))
        w_cmd = float(np.clip(u[1], -self.w_max, self.w_max))
        self._store_interval(tick_now, v_cmd, w_cmd, self.v_ref, self.w_ref, gain)
        self.last_v_cmd = v_cmd
        self.last_w_cmd = w_cmd
        return v_cmd, w_cmd

    def diagnostics(self) -> dict[str, float]:
        return {
            "slip_est": float(self.slip_est),
            "gain": float(max(0.20, 1.0 - self.slip_est)),
            "v_hat": float(self.x_hat[0]),
            "w_hat": float(self.x_hat[1]),
            "last_v_cmd": float(self.last_v_cmd),
            "last_w_cmd": float(self.last_w_cmd),
        }

    def _build_mpc(self):
        try:
            with warnings.catch_warnings():
                warnings.filterwarnings(
                    "ignore",
                    message="The ONNX feature is not available.*",
                    category=UserWarning,
                )
                warnings.filterwarnings(
                    "ignore",
                    message="The opcua feature is not available.*",
                    category=UserWarning,
                )
                import do_mpc
        except Exception as exc:
            raise RuntimeError(
                "nexus_sand_mpc requires do-mpc and CasADi in the ROS Python environment. "
                "Install them for /usr/bin/python3, or port this package to a C++ QP backend."
            ) from exc

        model = do_mpc.model.Model("discrete")
        v = model.set_variable("_x", "v")
        w = model.set_variable("_x", "w")
        int_v = model.set_variable("_x", "int_v")
        int_w = model.set_variable("_x", "int_w")
        qv = [model.set_variable("_x", f"qv{i}") for i in range(self.delay_steps)]
        qw = [model.set_variable("_x", f"qw{i}") for i in range(self.delay_steps)]
        u_v = model.set_variable("_u", "u_v")
        u_w = model.set_variable("_u", "u_w")
        v_ref = model.set_variable("_tvp", "v_ref")
        w_ref = model.set_variable("_tvp", "w_ref")
        gain = model.set_variable("_tvp", "gain")

        dt = self.dt_nominal
        av = min(1.0, dt / max(self.drive_tau, 1e-3))
        aw = min(1.0, dt / max(self.turn_tau, 1e-3))
        eff_v = qv[0] if self.delay_steps > 0 else u_v
        eff_w = qw[0] if self.delay_steps > 0 else u_w

        model.set_rhs("v", v + av * (gain * eff_v - v))
        model.set_rhs("w", w + aw * (gain * eff_w - w))
        model.set_rhs("int_v", int_v + dt * (v - v_ref))
        model.set_rhs("int_w", int_w + dt * (w - w_ref))
        for i in range(self.delay_steps):
            model.set_rhs(f"qv{i}", qv[i + 1] if i + 1 < self.delay_steps else u_v)
            model.set_rhs(f"qw{i}", qw[i + 1] if i + 1 < self.delay_steps else u_w)
        model.setup()

        mpc = do_mpc.controller.MPC(model)
        mpc.set_param(
            n_horizon=self.horizon,
            t_step=self.dt_nominal,
            state_discretization="discrete",
            store_full_solution=False,
            nlpsol_opts={"ipopt.print_level": 0, "ipopt.sb": "yes", "print_time": 0},
        )

        lterm = (
            self.q_v * (v - v_ref) ** 2
            + self.q_w * (w - w_ref) ** 2
            + self.q_int_v * int_v**2
            + self.q_int_w * int_w**2
        )
        mterm = lterm
        mpc.set_objective(mterm=mterm, lterm=lterm)
        mpc.set_rterm(u_v=self.r_du_v, u_w=self.r_du_w)
        mpc.bounds["lower", "_u", "u_v"] = 0.0
        mpc.bounds["upper", "_u", "u_v"] = self.v_max
        mpc.bounds["lower", "_u", "u_w"] = -self.w_max
        mpc.bounds["upper", "_u", "u_w"] = self.w_max

        tvp_template = mpc.get_tvp_template()

        def tvp_fun(_t_now):
            tvp_template["_tvp", :, "v_ref"] = self._tvp_v_ref
            tvp_template["_tvp", :, "w_ref"] = self._tvp_w_ref
            tvp_template["_tvp", :, "gain"] = self._tvp_gain
            return tvp_template

        mpc.set_tvp_fun(tvp_fun)
        mpc.setup()
        return model, mpc

    def _sync_to_tick(self, tick_now: int) -> None:
        if tick_now < self.current_tick:
            self.current_tick = min(tick_now, len(self.snapshot_states) - 1)
            self.x_hat = self.snapshot_states[self.current_tick].copy()
            return

        while self.current_tick < tick_now:
            step_idx = self.current_tick
            v_cmd, w_cmd = self._interval_pair(self.step_cmds, step_idx, (self.last_v_cmd, self.last_w_cmd))
            v_ref, w_ref = self._interval_pair(self.step_refs, step_idx, (self.v_ref, self.w_ref))
            gain = self._interval_scalar(self.step_gains, step_idx, self._tvp_gain)
            self.x_hat = self._step_state(self.x_hat, v_cmd, w_cmd, gain, v_ref, w_ref)
            self.current_tick += 1
            if self.current_tick < len(self.snapshot_states):
                self.snapshot_states[self.current_tick] = self.x_hat.copy()
            else:
                self.snapshot_states.append(self.x_hat.copy())

    def _replay_latest_observation(self, tick_now: int) -> np.ndarray:
        assert self.pending_obs is not None
        base_tick = min(max(self._tick_floor(self.pending_obs.timestamp), 0), tick_now)
        x_corr = self.snapshot_states[base_tick].copy()
        cg = self.correction_gain
        x_corr[0] = max(0.0, (1.0 - cg) * x_corr[0] + cg * max(0.0, self.pending_obs.v))
        x_corr[1] = (1.0 - cg) * x_corr[1] + cg * self.pending_obs.w

        for step_idx in range(base_tick, tick_now):
            v_cmd, w_cmd = self._interval_pair(self.step_cmds, step_idx, (self.last_v_cmd, self.last_w_cmd))
            v_ref, w_ref = self._interval_pair(self.step_refs, step_idx, (self.v_ref, self.w_ref))
            gain = self._interval_scalar(self.step_gains, step_idx, self._tvp_gain)
            x_corr = self._step_state(x_corr, v_cmd, w_cmd, gain, v_ref, w_ref)
        return x_corr

    def _step_state(
        self,
        x: np.ndarray,
        v_cmd: float,
        w_cmd: float,
        gain: float,
        v_ref: float,
        w_ref: float,
    ) -> np.ndarray:
        x_next = np.array(x, copy=True)
        d = self.delay_steps
        eff_v = x[4] if d > 0 else v_cmd
        eff_w = x[4 + d] if d > 0 else w_cmd
        dt = self.dt_nominal
        av = min(1.0, dt / max(self.drive_tau, 1e-3))
        aw = min(1.0, dt / max(self.turn_tau, 1e-3))

        v = max(0.0, float(x[0]))
        x_next[0] = max(0.0, v + av * (gain * eff_v - v))
        x_next[1] = x[1] + aw * (gain * eff_w - x[1])
        x_next[2] = x[2] + dt * (v - v_ref)
        x_next[3] = x[3] + dt * (x[1] - w_ref)

        if d > 0:
            if d > 1:
                x_next[4 : 4 + d - 1] = x[5 : 4 + d]
                x_next[4 + d : 4 + 2 * d - 1] = x[5 + d : 4 + 2 * d]
            x_next[4 + d - 1] = v_cmd
            x_next[4 + 2 * d - 1] = w_cmd
        return x_next

    def _store_interval(
        self,
        tick: int,
        v_cmd: float,
        w_cmd: float,
        v_ref: float,
        w_ref: float,
        gain: float,
    ) -> None:
        self._store_pair(self.step_cmds, tick, (v_cmd, w_cmd))
        self._store_pair(self.step_refs, tick, (v_ref, w_ref))
        self._store_scalar(self.step_gains, tick, gain)

    @staticmethod
    def _store_pair(values: list[tuple[float, float]], index: int, value: tuple[float, float]) -> None:
        while len(values) < index:
            previous = values[-1] if values else (0.0, 0.0)
            values.append((float(previous[0]), float(previous[1])))
        if index == len(values):
            values.append((float(value[0]), float(value[1])))
        else:
            values[index] = (float(value[0]), float(value[1]))

    @staticmethod
    def _store_scalar(values: list[float], index: int, value: float) -> None:
        while len(values) < index:
            values.append(float(values[-1]) if values else float(value))
        if index == len(values):
            values.append(float(value))
        else:
            values[index] = float(value)

    @staticmethod
    def _interval_pair(
        values: list[tuple[float, float]],
        index: int,
        fallback: tuple[float, float],
    ) -> tuple[float, float]:
        if index < len(values):
            return float(values[index][0]), float(values[index][1])
        return float(fallback[0]), float(fallback[1])

    @staticmethod
    def _interval_scalar(values: list[float], index: int, fallback: float) -> float:
        if index < len(values):
            return float(values[index])
        return float(fallback)

    def _tick_from_time(self, t_now: float) -> int:
        return max(0, int(round(float(t_now) / self.dt_nominal)))

    def _tick_floor(self, t_now: float) -> int:
        return int(math.floor(float(t_now) / self.dt_nominal + 1e-9))

    def _command_at(self, t_query: float, channel: str) -> float:
        if not self.step_cmds:
            return 0.0
        tick = self._tick_floor(t_query)
        if tick < 0:
            return 0.0
        if tick >= len(self.step_cmds):
            cmd = self.step_cmds[-1]
        else:
            cmd = self.step_cmds[tick]
        return float(cmd[0] if channel == "v" else cmd[1])
