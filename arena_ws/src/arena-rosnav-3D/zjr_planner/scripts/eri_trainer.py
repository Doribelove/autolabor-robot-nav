#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, time, json, math, collections
import rospy
import torch
import torch.nn as nn
import torch.optim as optim
from std_msgs.msg import String, Float32
from torch.distributions import Beta
from datetime import datetime

import warnings
warnings.filterwarnings(
    "ignore",
    message="CUDA initialization: The NVIDIA driver on your system is too old.*",
    category=UserWarning
)


# -----------------------
# Utilities
# -----------------------
def to_tensor(x, dtype=torch.float32):
    return torch.tensor(x, dtype=dtype)

def clamp01(v, eps=1e-6):
    return max(eps, min(1.0 - eps, v))

def three_step_returns(ep, gamma):
    """ep: list of transitions (dict) for a single episode_id"""
    n = len(ep)
    R3 = [0.0] * n
    for t in range(n):
        r0 = ep[t]["reward"]
        r1 = ep[t+1]["reward"] if t+1 < n else 0.0
        r2 = ep[t+2]["reward"] if t+2 < n else 0.0
        R3[t] = r0 + gamma * r1 + (gamma ** 2) * r2
    return R3

def iter_episodes(stream):
    """stream: list of transitions (already time-ordered)"""
    i, n = 0, len(stream)
    while i < n:
        eid = stream[i]["episode_id"]
        j = i
        while j < n and stream[j]["episode_id"] == eid:
            j += 1
        yield stream[i:j]
        i = j

# -----------------------
# Models
# -----------------------
class Actor(nn.Module):
    """Beta policy head: x=[tau, rho] -> (alpha>0, beta>0)"""
    def __init__(self, ab_max: float = 50.0):
        super().__init__()
        self.ab_max = float(ab_max)
        self.net = nn.Sequential(
            nn.Linear(2, 32), nn.ReLU(),
            nn.Linear(32, 32), nn.ReLU(),
            nn.Linear(32, 2)
        )
        self.softplus = nn.Softplus()  # for positive constraints

    def forward(self, x):  # x: (B,2)
        raw = self.net(x)
        alpha = 1.0 + self.softplus(raw[:, 0])
        beta  = 1.0 + self.softplus(raw[:, 1])
        # clamp to avoid explosion / collapse
        alpha = alpha.clamp(min=1e-3, max=self.ab_max)
        beta  = beta.clamp(min=1e-3, max=self.ab_max)
        return alpha, beta

class Critic(nn.Module):
    def __init__(self):
        super().__init__()
        self.v = nn.Sequential(
            nn.Linear(2, 32), nn.ReLU(),
            nn.Linear(32, 1)
        )
    def forward(self, x):
        return self.v(x).squeeze(-1)

# -----------------------
# Trainer Node
# -----------------------
class ERITrainerNode(object):
    def __init__(self):
        rospy.init_node("eri_trainer")

        # --- Read minimal shared params ---
        self.eri_min = float(rospy.get_param("~eri_min", rospy.get_param("/eri_min", 0.0)))
        self.eri_max = float(rospy.get_param("~eri_max", rospy.get_param("/eri_max", 10.0)))

        # --- Paths / checkpoint config ---
        base_dir = rospy.get_param("~save_dir", rospy.get_param("/save_dir", "/tmp"))
        self.save_dir = os.path.abspath(base_dir)

        # --- dirs ---
        self.latest_dir = os.path.join(self.save_dir, "latest")
        self.best_dir   = os.path.join(self.save_dir, "best")

        run_id = rospy.get_param("~run_id", datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
        self.snapshot_run_dir = os.path.join(self.save_dir, "snapshots", f"run_{run_id}")

        for d in [
            os.path.join(self.latest_dir, "checkpoints"),
            os.path.join(self.latest_dir, "exports"),
            os.path.join(self.latest_dir, "metrics"),
            os.path.join(self.best_dir, "checkpoints"),
            os.path.join(self.best_dir, "exports"),
            os.path.join(self.best_dir, "metrics"),
            self.snapshot_run_dir,
        ]:
            os.makedirs(d, exist_ok=True)

        # --- paths ---
        self.ckpt_latest_path = os.path.join(self.latest_dir, "checkpoints", "latest.pth")
        self.ckpt_best_path   = os.path.join(self.best_dir,   "checkpoints", "best.pth")

        self.ts_latest_path   = os.path.join(self.latest_dir, "exports", "eri_net_ts.pt")
        self.ts_best_path     = os.path.join(self.best_dir,   "exports", "eri_net_ts.pt")

        self.train_csv_path   = os.path.join(self.latest_dir, "metrics", "train.csv")
        self.eval_csv_path    = os.path.join(self.latest_dir, "metrics", "eval.csv")
        self.best_json_path   = os.path.join(self.best_dir,   "metrics", "best.json")

        # --- Trainer hyperparams (tweak via rosparam if needed) ---
        self.gamma       = float(rospy.get_param("~gamma", 0.9))
        self.lr          = float(rospy.get_param("~lr", 1e-3))
        self.batch_min   = int(rospy.get_param("~batch_min", 32))      # start training when >= this many
        self.window_max  = int(rospy.get_param("~window_max", 5000))    # rolling buffer for batch build
        self.save_every  = float(rospy.get_param("~save_every_sec", 60.0))
        self.lambda_reg0 = float(rospy.get_param("~lambda_reg0", 1.0))  # behavior regularizer init
        self.lambda_reg  = self.lambda_reg0
        self.lambda_decay= float(rospy.get_param("~lambda_decay", 1e-4)) # per update
        self.adv_norm    = bool(rospy.get_param("~adv_norm", True))      # advantage normalization
        self.grad_clip = float(rospy.get_param("~grad_clip", 1.0))  # set <=0 to disable

        # ---- feature normalization ----
        self.normalize_rho = bool(rospy.get_param("~normalize_rho", True))
        self.rho_max = float(rospy.get_param("~rho_max", 0.03))  # rho in [0, 0.03] by your observation

        self.snapshot_every = float(rospy.get_param("~snapshot_every", 20))
        self.max_snapshots = int(rospy.get_param("~max_snapshots", 20))
        self.snapshots = []

        # --- Models/optim ---
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.ab_max = float(rospy.get_param("~ab_max", 50.0))
        self.actor  = Actor(ab_max=self.ab_max).to(self.device)
        # self.actor  = Actor().to(self.device)
        self.critic = Critic().to(self.device)
        self.opt    = optim.Adam(list(self.actor.parameters()) + list(self.critic.parameters()), lr=self.lr)

        # --- Buffers/pubs/subs ---
        self.buffer = collections.deque(maxlen=50000)  # transitions
        rospy.Subscriber("/sderi/transition", String, self.on_transition, queue_size=2000)
        self.pub_loss = rospy.Publisher("/sderi/train/loss", Float32, queue_size=10)
        self.pub_ret  = rospy.Publisher("/sderi/train/return_ema", Float32, queue_size=10)

        # training stats
        self.ret_ema       = 0.0
        self.ema_beta      = 0.95
        self.global_step   = 0
        self.best_ret_ema  = None
        self.last_save_t   = time.time()

        # 是否自动从 latest checkpoint 恢复
        self.resume = bool(rospy.get_param("~resume", True))
        self.resume_from = str(rospy.get_param("~resume_from", "latest")).lower()

        resume_candidates = []
        if self.resume_from == "best":
            resume_candidates = [self.ckpt_best_path, self.ckpt_latest_path]
        else:
            resume_candidates = [self.ckpt_latest_path, self.ckpt_best_path]

        if self.resume:
            for p in resume_candidates:
                if os.path.exists(p):
                    try:
                        self._load_checkpoint(p)
                        rospy.loginfo("[eri_trainer] resumed from %s (global_step=%d, ret_ema=%.3f)",
                                    p, self.global_step, self.ret_ema)
                        break
                    except Exception as e:
                        rospy.logwarn("[eri_trainer] failed to resume from %s: %s", p, e)

        # 如果已经有 best checkpoint，记录它的 ret_ema，用于之后判定“是否更好”
        if os.path.exists(self.ckpt_best_path):
            try:
                st_best = torch.load(self.ckpt_best_path, map_location=self.device)
                self.best_ret_ema = float(st_best.get("ret_ema", 0.0))
                rospy.loginfo("[eri_trainer] found best checkpoint %s with ret_ema=%.3f",
                              self.ckpt_best_path, self.best_ret_ema)
            except Exception as e:
                rospy.logwarn("[eri_trainer] could not read best checkpoint %s: %s",
                              self.ckpt_best_path, e)

        rospy.loginfo("[eri_trainer] device=%s save_dir=%s", self.device, self.save_dir)


    # -------- Callbacks ----------

    def _norm_rho(self, rho_raw: float) -> float:
        if not self.normalize_rho:
            return float(rho_raw)
        denom = max(1e-8, float(self.rho_max))
        v = float(rho_raw) / denom
        return max(0.0, min(1.0, v))

    def on_transition(self, msg: String):
        try:
            tr = json.loads(msg.data)
            # sanity coercions
            tr["tau"]       = float(tr.get("tau")) if tr.get("tau") is not None else 0.0
            tr["rho"]       = float(tr.get("rho")) if tr.get("rho") is not None else 0.0
            tr["eri_rule"]  = float(tr.get("eri_rule")) if tr.get("eri_rule") is not None else 0.0
            tr["eri_act"]   = float(tr.get("eri_act")) if tr.get("eri_act") is not None else tr["eri_rule"]
            tr["reward"]    = float(tr.get("reward", 0.0))
            tr["done"]      = bool(tr.get("done", False))
            tr["episode_id"]= int(tr.get("episode_id", 0))
            tr["acted_by"]  = str(tr.get("acted_by", "teacher"))
            self.buffer.append(tr)
        except Exception as e:
            rospy.logwarn("[eri_trainer] bad transition JSON: %s", e)

    # -------- Checkpoint helpers ---------

    def _atomic_torch_save(self, obj, path: str):
        tmp = path + ".tmp"
        torch.save(obj, tmp)
        os.replace(tmp, path)

    def _atomic_ts_save(self, ts, path: str):
        tmp = path + ".tmp"
        ts.save(tmp)
        os.replace(tmp, path)

    def _checkpoint_state(self, global_step: int):
        return {
            "global_step": int(global_step),
            "actor": self.actor.state_dict(),
            "critic": self.critic.state_dict(),
            "opt": self.opt.state_dict(),
            "ret_ema": float(self.ret_ema),
            "lambda_reg": float(getattr(self, "lambda_reg", 0.0)),
            "gamma": float(self.gamma),
            "batch_min": int(self.batch_min),
            "window_max": int(self.window_max),
            "adv_norm": bool(self.adv_norm),
        }

    def _save_checkpoint(self, path: str, global_step: int, best: bool = False):
        state = self._checkpoint_state(global_step)
        self._atomic_torch_save(state, path)   # ✅ 注意这里是 self.
        rospy.loginfo("[eri_trainer] checkpoint saved to %s (%s)",
                    path, "best" if best else "latest")

    def _load_checkpoint(self, path):
        state = torch.load(path, map_location=self.device)
        self.actor.load_state_dict(state["actor"])
        self.critic.load_state_dict(state["critic"])
        if "opt" in state:
            self.opt.load_state_dict(state["opt"])
        self.ret_ema   = float(state.get("ret_ema", 0.0))
        self.lambda_reg = float(state.get("lambda_reg", getattr(self, "lambda_reg", 0.0)))
        self.global_step = int(state.get("global_step", 0))

    # -------- Training Loop ----------
    def run(self):
        rate = rospy.Rate(10)
        step = 0
        while not rospy.is_shutdown():
            if len(self.buffer) < self.batch_min:
                rate.sleep(); continue

            # Build windowed batch (time-ordered)
            window = list(self.buffer)[-self.window_max:]
            xs, es, R3s, t_eris, m_act = [], [], [], [], []

            for ep in iter_episodes(window):
                R3 = three_step_returns(ep, self.gamma)
                for t, tr in enumerate(ep):
                    # x = [float(tr["tau"]), float(tr["rho"])]
                    tau = float(tr["tau"])
                    rho_raw = float(tr["rho"])
                    rho = self._norm_rho(rho_raw)
                    x = [tau, rho]
                    # normalize action to e in [0,1] for Beta log_prob
                    e = (float(tr["eri_act"]) - self.eri_min) / max(1e-8, (self.eri_max - self.eri_min))
                    e = clamp01(e)
                    xs.append(x)
                    es.append([e])
                    R3s.append([R3[t]])
                    t_eris.append([float(tr["eri_rule"])])
                    m_act.append([1.0 if tr["acted_by"] == "student" else 0.0])

            if len(xs) == 0:
                rate.sleep(); continue

            x = to_tensor(xs).to(self.device)                        # (B,2)
            e = to_tensor(es).squeeze(1).to(self.device)             # (B,)
            R3 = to_tensor(R3s).squeeze(1).to(self.device)           # (B,)
            t_eri = to_tensor(t_eris).squeeze(1).to(self.device)     # (B,)
            m_act = to_tensor(m_act).squeeze(1).to(self.device)      # (B,)

            alpha, beta = self.actor(x)
            dist = Beta(alpha, beta)
            logp = dist.log_prob(e.clamp(1e-6, 1-1e-6))              # (B,)
            V = self.critic(x)                                       # (B,)

            with torch.no_grad():
                adv = R3 - V
                if self.adv_norm and adv.numel() > 1:
                    adv = (adv - adv.mean()) / (adv.std() + 1e-6)

            # Actor: only where student acted
            if m_act.sum() > 0:
                loss_actor = -((adv.detach() * logp) * m_act).sum() / m_act.sum()
            else:
                loss_actor = torch.zeros((), device=self.device)

            # Critic loss: all steps (teacher + student)
            loss_critic = (V - R3).pow(2).mean()

            # Behavior regularizer toward teacher ERI on the action taken
            eri_act = self.eri_min + (self.eri_max - self.eri_min) * e
            loss_reg = ((eri_act - t_eri).pow(2)).mean()

            loss = loss_actor + 0.5 * loss_critic + self.lambda_reg * loss_reg

            self.opt.zero_grad()
            loss.backward()
            # nn.utils.clip_grad_norm_(list(self.actor.parameters()) + list(self.critic.parameters()), 1.0)
            if self.grad_clip and self.grad_clip > 0:
                nn.utils.clip_grad_norm_(
                    list(self.actor.parameters()) + list(self.critic.parameters()),
                    self.grad_clip
                )
            self.opt.step()
            step += 1

            # Telemetry
            with torch.no_grad():
                ret_mean = R3.mean().item()
            self.ret_ema = self.ema_beta * self.ret_ema + (1 - self.ema_beta) * ret_mean
            self.pub_loss.publish(float(loss.item()))
            self.pub_ret.publish(float(self.ret_ema))

            self.lambda_reg = max(0.0, self.lambda_reg - self.lambda_decay)

            # 统计指标
            with torch.no_grad():
                ret_mean = R3.mean().item()
            self.ret_ema = self.ema_beta * self.ret_ema + (1.0 - self.ema_beta) * ret_mean
            self.pub_loss.publish(float(loss.item()))
            self.pub_ret.publish(float(self.ret_ema))

            # 训练 step 计数
            self.global_step += 1

            # ----- checkpoint & TS 导出逻辑 -----
            now = time.time()
            improved = (self.best_ret_ema is None) or (self.ret_ema > self.best_ret_ema)

            if improved:
                # 更新 best
                self.best_ret_ema = self.ret_ema
                self._save_checkpoint(self.ckpt_best_path, self.global_step, best=True)
                self._save_checkpoint(self.ckpt_latest_path, self.global_step, best=False)
                self.export_torchscript(step=self.global_step, is_best=True)
                self.last_save_t = now
            elif now - self.last_save_t > self.save_every:
                # 定期保存 latest（即使没有变好，也方便 resume）
                self._save_checkpoint(self.ckpt_latest_path, self.global_step, best=False)
                self.export_torchscript(step=self.global_step, is_best=False)
                self.last_save_t = now

            rate.sleep()

    def export_torchscript(self, step: int = None, is_best: bool = False):
        """
        Export actor to TorchScript.

        - always export latest -> latest/exports/eri_net_ts.pt
        - if is_best: also export best -> best/exports/eri_net_ts.pt and write best.json
        - optionally archive snapshot checkpoints to snapshots/run_xxx/step_XXXXXXX.pth
        """
        self.actor.eval()
        ex = torch.randn(1, 2, device=self.device)
        ts = torch.jit.trace(self.actor, ex)

        # 1) latest export always
        self._atomic_ts_save(ts, self.ts_latest_path)

        # 2) best export if needed
        if is_best:
            self._atomic_ts_save(ts, self.ts_best_path)
            try:
                with open(self.best_json_path, "w") as f:
                    json.dump(
                        {"global_step": int(step if step is not None else -1),
                        "ret_ema": float(self.ret_ema)},
                        f, indent=2
                    )
            except Exception as e:
                rospy.logwarn("[eri_trainer] failed to write best.json: %s", e)

            rospy.loginfo("[eri_trainer] TorchScript BEST saved to %s", self.ts_best_path)
        else:
            rospy.loginfo("[eri_trainer] TorchScript latest saved to %s", self.ts_latest_path)

        # 3) snapshot archive (pth) — align with your desired structure
        if (step is not None) and (self.snapshot_every > 0):
            if (step % self.snapshot_every) == 0 or is_best:
                snap_path = os.path.join(self.snapshot_run_dir, f"step_{step:07d}.pth")
                try:
                    self._atomic_torch_save(self._checkpoint_state(step), snap_path)
                    self.snapshots.append(snap_path)

                    # enforce max_snapshots
                    while len(self.snapshots) > self.max_snapshots:
                        old = self.snapshots.pop(0)
                        try:
                            if os.path.exists(old):
                                os.remove(old)
                        except Exception as e:
                            rospy.logwarn("[eri_trainer] failed to remove old snapshot %s: %s", old, e)

                    rospy.loginfo("[eri_trainer] snapshot archived: %s", snap_path)
                except Exception as e:
                    rospy.logwarn("[eri_trainer] failed to save snapshot %s: %s", snap_path, e)



if __name__ == "__main__":
    node = ERITrainerNode()
    node.run()