"""SO101BaseEnv: mujoco 内核 gymnasium 基类（workflow/plan/envs.md Phase 1）。

- 动作: Box(-1,1,(6,)) 归一化关节位置目标, 经 6 个 <position> actuator 执行
- 内环: 每控制步 n_substeps 次 mj_step (默认 50 x 2ms = 10Hz 控制 / 500Hz 仿真)
- obs: state 向量 [qpos(6) qvel(6) tcp_pos(3) tcp_quat(4)] + 任务专属段
- 相机: camera=True 时 obs 为 Dict{"state","rgb":{"fixed","wrist"}}
"""

from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "egl")

from pathlib import Path

import gymnasium
import mujoco
import numpy as np
from gymnasium import spaces

from .randomize import RandomizationConfig
from .utils import is_grasping, is_touching, is_static

ASSETS = Path(__file__).resolve().parent / "assets"

ROBOT_JOINTS = [
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper",
]
# rest pose = squint SO101 "start" keyframe (相机朝前, 夹爪微开)
REST_QPOS = np.array([0.0, 0.0, 0.0, np.pi / 2, -np.pi / 2, 60 * np.pi / 180])

RENDER_SIZE = 128
N_SUBSTEPS = 50
# squint pd_joint_target_delta_pos: 每控制步 ctrl 增量上限（rad）
DELTA_MAX = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.2])
# squint TCP = 两指尖中点（URDF finger1_tip/finger2_tip 在对应 link 系的偏移）
FINGER1_TIP_LOCAL = np.array([-0.002, 0.0, -0.092])   # gripper_link
FINGER2_TIP_LOCAL = np.array([-0.01, -0.077, 0.02])   # moving_jaw_so101_v1_link


class SO101BaseEnv(gymnasium.Env):
    """SO101 mujoco gymnasium 基类, 任务类继承并实现 4 个 hook。"""

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 10}
    SCENE_XML = None  # 子类必须指定 assets 下的场景文件名
    TERMINATE_ON_SUCCESS = True  # squint 任务不因 success 终止（lift 子类置 False）

    def __init__(self, camera=False, render_mode=None, cfg=None, n_substeps=N_SUBSTEPS,
                 viewer_ui=True, viewer_maximize=False, viewer_no_input=False,
                 domain_randomization=False, action_mode="position"):
        super().__init__()
        assert self.SCENE_XML is not None, "子类必须设置 SCENE_XML"
        assert render_mode in (None, "rgb_array", "human")
        assert action_mode in ("position", "delta"), f"action_mode 非法: {action_mode}"
        self.camera = bool(camera)
        self.render_mode = render_mode
        self.cfg = cfg if cfg is not None else RandomizationConfig()
        self.n_substeps = int(n_substeps)
        self.domain_randomization = bool(domain_randomization)
        self.action_mode = action_mode
        self._ctrl_target = None  # squint use_target=True: 控制器目标 qpos

        self.model = mujoco.MjModel.from_xml_path(str(ASSETS / self.SCENE_XML))
        self.data = mujoco.MjData(self.model)

        self._qpos_adr = np.array(
            [self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
             for n in ROBOT_JOINTS]
        )
        self._qvel_adr = np.array(
            [self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
             for n in ROBOT_JOINTS]
        )
        self._ctrlrange = np.array(
            [self.model.actuator_ctrlrange[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)]
             for n in ROBOT_JOINTS]
        )
        self._tcp_sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
        self._gripper_bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
        self._jaw_bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "moving_jaw_so101_v1")
        self._step_count = 0

        self._init_spaces()
        self._renderer = None
        self._viewer = None
        # viewer 窗口选项（human 模式, _open_viewer 惰性消费）
        self._viewer_ui = viewer_ui
        self._viewer_maximize = viewer_maximize
        self._viewer_no_input = viewer_no_input
        self._closed = False
        self._item_base_rgba = {}   # geom_name -> 基色 (DR 从基色采样, 防累积漂移)
        self._fixed_cam_base = None  # fixed_cam 基位姿 (同上)

    # ---------- 空间 ----------
    def _init_spaces(self):
        self.action_space = spaces.Box(-1.0, 1.0, shape=(6,), dtype=np.float32)
        state_dim = 19 + self.TASK_OBS_DIM
        state_box = spaces.Box(-np.inf, np.inf, shape=(state_dim,), dtype=np.float64)
        if self.camera:
            rgb_box = spaces.Box(0, 255, shape=(RENDER_SIZE, RENDER_SIZE, 3), dtype=np.uint8)
            self.observation_space = spaces.Dict(
                {"state": state_box, "rgb": spaces.Dict({"fixed": rgb_box, "wrist": rgb_box})}
            )
        else:
            self.observation_space = state_box

    TASK_OBS_DIM = 0  # 子类覆盖: 任务专属 obs 维数

    # ---------- 工具 ----------
    def denormalize_action(self, action):
        """[-1,1] 动作 -> 各关节 ctrlrange 内的目标位置 (rad)。"""
        lo = self._ctrlrange[:, 0]
        hi = self._ctrlrange[:, 1]
        return lo + (np.clip(action, -1.0, 1.0) + 1.0) / 2.0 * (hi - lo)

    def normalize_qpos(self, qpos):
        """关节位置 (rad) -> [-1,1] 动作。"""
        lo = self._ctrlrange[:, 0]
        hi = self._ctrlrange[:, 1]
        return 2.0 * (np.asarray(qpos) - lo) / (hi - lo) - 1.0

    @property
    def tcp_pose(self):
        """TCP 位姿 (pos3 + quat wxyz4): 两指尖中点 + gripper_link 朝向（squint）。"""
        return self.tcp_pos, self.data.xquat[self._gripper_bid].copy()

    @property
    def tcp_pos(self):
        """两指尖中点（随时间/夹爪角变化；squint agent.tcp_pos）。"""
        Rg = self.data.xmat[self._gripper_bid].reshape(3, 3)
        p1 = self.data.xpos[self._gripper_bid] + Rg @ FINGER1_TIP_LOCAL
        Rj = self.data.xmat[self._jaw_bid].reshape(3, 3)
        p2 = self.data.xpos[self._jaw_bid] + Rj @ FINGER2_TIP_LOCAL
        return (p1 + p2) / 2.0

    @property
    def target_qpos(self):
        """控制器目标 qpos（squint controller._target_qpos; position 模式下 = ctrl）。"""
        if self._ctrl_target is None:
            return self.data.ctrl.copy()
        return self._ctrl_target.copy()

    def _get_state_obs(self):
        qpos = self.data.qpos[self._qpos_adr].copy()
        if self.domain_randomization and self.cfg.robot_qpos_noise_std > 0:
            qpos += self.np_random.normal(0.0, self.cfg.robot_qpos_noise_std, size=6)
        qvel = self.data.qvel[self._qvel_adr]
        tcp_p, tcp_q = self.tcp_pose
        return np.concatenate([qpos, qvel, tcp_p, tcp_q]).astype(np.float64)

    # ---------- gymnasium API ----------
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        rng = self.np_random
        mujoco.mj_resetData(self.model, self.data)
        # 机器人: rest pose + 初始 qpos 噪声 (squint: rest + randn*scale, 默认开)
        q0 = REST_QPOS + rng.normal(0.0, self.cfg.initial_qpos_noise_scale, size=6)
        q0 = np.clip(q0, self._ctrlrange[:, 0], self._ctrlrange[:, 1])
        self.data.qpos[self._qpos_adr] = q0
        self.data.ctrl[:] = q0
        self._ctrl_target = q0.copy()
        self._reset_task(rng)
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)
        self._step_count = 0
        return self._get_obs(), {}

    def step(self, action):
        action = np.asarray(action, dtype=np.float64)
        if self.action_mode == "delta":
            # squint pd_joint_target_delta_pos: 归一化动作 -> ±DELTA_MAX 增量,
            # 目标积分并钳在 ctrlrange（use_target=True 语义）
            lo = self._ctrlrange[:, 0]
            hi = self._ctrlrange[:, 1]
            self._ctrl_target = np.clip(
                self._ctrl_target + np.clip(action, -1.0, 1.0) * DELTA_MAX, lo, hi)
            ctrl = self._ctrl_target
        else:
            ctrl = self.denormalize_action(action)
            self._ctrl_target = ctrl.copy()
        self.data.ctrl[:] = ctrl
        for _ in range(self.n_substeps):
            mujoco.mj_step(self.model, self.data)
        self._step_count += 1
        info = self.evaluate()
        reward = float(self.compute_dense_reward(action, info))
        terminated = bool(info.get("success", False) and self.TERMINATE_ON_SUCCESS)
        obs = self._get_obs()
        if self.render_mode == "human":
            self.render()
        return obs, reward, terminated, False, info

    def _get_obs(self):
        state = self._get_state_obs()
        if self.camera:
            return {"state": state, "rgb": self._render_cameras()}
        return state

    def render(self):
        if self.render_mode == "rgb_array":
            return self._render_camera("fixed_cam")
        if self.render_mode == "human":
            # gymnasium 约定: human 模式由 env 在 step/reset 后刷新窗口
            if self._viewer is None:
                self._open_viewer()
            self._viewer.sync()
            return None
        return None

    def _open_viewer(self):
        """惰性打开 mujoco viewer 交互窗口（render_mode="human"）。"""
        import mujoco.viewer

        self._viewer = mujoco.viewer.launch_passive(
            self.model, self.data,
            show_left_ui=self._viewer_ui, show_right_ui=self._viewer_ui,
        )
        if self._viewer_maximize:
            from .utils import maximize_window_by_name

            maximize_window_by_name("MuJoCo")
        if self._viewer_no_input:
            from .utils import set_window_no_input

            set_window_no_input("MuJoCo")

    def close(self):
        self._closed = True
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None

    # ---------- 相机渲染 (Phase 3 主用, Phase 1 仅 rgb_array) ----------
    def _get_renderer(self):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, RENDER_SIZE, RENDER_SIZE)
        return self._renderer

    def _render_camera(self, cam_name):
        r = self._get_renderer()
        cid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
        r.update_scene(self.data, camera=cid)
        return r.render().copy()

    def _render_cameras(self):
        return {"fixed": self._render_camera("fixed_cam"), "wrist": self._render_camera("wrist_cam")}

    # ---------- 自由物体工具 ----------
    def set_free_body_pose(self, body, pos, yaw=0.0):
        """设置自由物体位姿（位置 + yaw 旋转），速度清零。"""
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
        jadr = self.model.body_jntadr[bid]
        qadr = self.model.jnt_qposadr[jadr]
        vadr = self.model.jnt_dofadr[jadr]
        self.data.qpos[qadr:qadr + 3] = pos
        self.data.qpos[qadr + 3] = np.cos(yaw / 2)
        self.data.qpos[qadr + 6] = np.sin(yaw / 2)
        self.data.qvel[vadr:vadr + 6] = 0.0

    def free_body_pos(self, body):
        """自由物体当前位置 (3,)。"""
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
        qadr = self.model.jnt_qposadr[self.model.body_jntadr[bid]]
        return self.data.qpos[qadr:qadr + 3].copy()

    def free_body_pose(self, body):
        """自由物体位姿 qpos[7] (pos3 + quat wxyz4)。"""
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
        qadr = self.model.jnt_qposadr[self.model.body_jntadr[bid]]
        return self.data.qpos[qadr:qadr + 7].copy()

    # ---------- squint 物体几何/惯性（cube/can 共用） ----------
    def sample_box_geometry(self, rng, half_range, density=None):
        """采样 cube: 返回 (geom_size(3), half_z, mass, inertia(3))。DR 关取中点。"""
        density = self.cfg.item_density if density is None else density
        h = rng.uniform(*half_range) if self.domain_randomization else sum(half_range) / 2.0
        side = 2.0 * h
        mass = density * side ** 3
        inertia = np.full(3, mass * side ** 2 / 6.0)
        return np.array([h, h, h]), float(h), float(mass), inertia

    def sample_cylinder_geometry(self, rng, radius_range, half_height_range, density=None):
        """采样 can: 返回 (geom_size(2)=[r, half_h], half_z, mass, inertia(3))。"""
        density = self.cfg.item_density if density is None else density
        r = rng.uniform(*radius_range) if self.domain_randomization else sum(radius_range) / 2.0
        hh = (rng.uniform(*half_height_range)
              if self.domain_randomization else sum(half_height_range) / 2.0)
        mass = density * np.pi * r ** 2 * (2.0 * hh)
        inertia = np.array([
            mass * (3.0 * r ** 2 + (2.0 * hh) ** 2) / 12.0,
            mass * (3.0 * r ** 2 + (2.0 * hh) ** 2) / 12.0,
            mass * r ** 2 / 2.0,
        ])
        return np.array([r, hh]), float(hh), float(mass), inertia

    def apply_free_geometry(self, body, geom_name, size, mass, inertia):
        """运行期改写自由物体 geom 尺寸/质量/惯性并重算常量（mj_setConst 会重置 data）。"""
        gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        bid = int(self.model.geom_bodyid[gid])
        self.model.geom_size[gid][: len(size)] = size
        self.model.body_mass[bid] = mass
        self.model.body_inertia[bid] = inertia
        self._recompute_constants()

    def _recompute_constants(self):
        """mj_setConst 重算 rbound/invweight 等；保存/恢复 data（setConst 会重置为默认态）。"""
        qpos, qvel, ctrl, sim_time = (
            self.data.qpos.copy(), self.data.qvel.copy(),
            self.data.ctrl.copy(), self.data.time)
        mocap_pos, mocap_quat = self.data.mocap_pos.copy(), self.data.mocap_quat.copy()
        mujoco.mj_setConst(self.model, self.data)
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel
        self.data.ctrl[:] = ctrl
        self.data.time = sim_time
        self.data.mocap_pos[:] = mocap_pos
        self.data.mocap_quat[:] = mocap_quat

    # ---------- 任务 hook (子类实现) ----------
    def _reset_task(self, rng):
        raise NotImplementedError

    def evaluate(self):
        raise NotImplementedError

    def compute_dense_reward(self, action, info):
        raise NotImplementedError

    # ---------- 域随机化 (Phase 3) ----------
    def _randomize_item(self, geom_name, rng):
        """任务类出生域采样后调用; 应用 rgba/摩擦随机化 (安全可变字段)。

        squint 语义: domain_randomization=True 才按 cfg range 采样; 否则取 range 中点。
        rgba 从首次缓存的基色采样（非增量累积），保证同 seed 多次 reset 可复现。
        """
        cfg = self.cfg
        gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        if self.domain_randomization and cfg.randomize_item_color:
            if geom_name not in self._item_base_rgba:
                self._item_base_rgba[geom_name] = np.array(self.model.geom_rgba[gid][:3])
            rgb = np.clip(
                self._item_base_rgba[geom_name]
                + rng.uniform(-1, 1, 3) * cfg.item_rgba_jitter, 0.0, 1.0)
            self.model.geom_rgba[gid][:3] = rgb
        lo, hi = cfg.item_friction_range
        f = rng.uniform(lo, hi) if self.domain_randomization else (lo + hi) / 2.0
        self.model.geom_friction[gid] = [f, 0.005, 0.0001]
        self._item_friction = float(f)

    def _randomize_fixed_cam(self, rng):
        """fixed 相机位姿噪声（从基位姿采样, 防累积漂移）。"""
        if self.cfg.fixed_camera_pos_noise <= 0:
            return
        cid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "fixed_cam")
        if self._fixed_cam_base is None:
            self._fixed_cam_base = np.array(self.model.cam_pos[cid])
        self.model.cam_pos[cid] = (
            self._fixed_cam_base + rng.uniform(-1, 1, 3) * self.cfg.fixed_camera_pos_noise)
