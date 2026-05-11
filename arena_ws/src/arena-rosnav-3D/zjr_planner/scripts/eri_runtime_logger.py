#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json, math, time
import rospy
from std_msgs.msg import String, Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan


def euclidean(a, b):
    dx, dy = a[0] - b[0], a[1] - b[1]
    return math.hypot(dx, dy)


class ERIRuntimeLogger(object):
    """
    Turns /sderi/choice + /odom + /scan into step-wise /sderi/transition with reward,
    and episode-wise /sderi/episode_summary.

    一条 transition 对应“上一个 choice window”（t_{k-1} -> t_k）。
    collision 不作为终止条件，只参与 reward 和 episode 统计。
    """

    def __init__(self):
        ns = rospy.get_namespace()  # e.g. "/"

        # --- reward & episode params ---
        self.goal_thresh = float(rospy.get_param("~goal_thresh", 0.5))
        self.w_progress = float(rospy.get_param("~w_progress", 1.0))
        self.w_time = float(rospy.get_param("~w_time", 0.07))
        self.w_collision = float(rospy.get_param("~w_collision", 3.0))
        self.success_bonus = float(rospy.get_param("~success_bonus", 10.0))
        self.timeout_penalty = float(rospy.get_param("~timeout_penalty", -10.0))

        # 碰撞判定阈值（建议与 model_params.yaml 中 robot_radius 保持一致）
        self.collision_distance = float(rospy.get_param("~collision_distance", 0.5))
        self.timeout_threshold_sec = float(rospy.get_param("~timeout_threshold_sec", 180.0))
        self.max_collisions = int(rospy.get_param("~max_collisions", 3))

        # stuck 检测：长时间几乎没有 progress 则认为卡住
        self.stuck_time_thresh = float(rospy.get_param("~stuck_time_thresh", 30.0))
        self.stuck_progress_eps = float(rospy.get_param("~stuck_progress_eps", 0.05))

        # goal 初始值；真正值通过 /sderi/final_goal 更新
        self.goal_xy = (0.0, 0.0)

        # --- runtime state ---
        self.episode_id = 0
        self.last_choice = None      # (stamp_sec, payload_dict, pose_xy_at_choice)
        self.last_pose_xy = None     # (x, y)

        # episode 级别统计
        self.episode_started_time = None
        self.episode_sum_reward = 0.0
        self.episode_step_count = 0
        self.episode_done = False    # 一旦 True，本 episode 内不再记录 transition

        # collision 统计（对齐 batch_generate_metrics.py 的 get_collisions 思路）
        self.collision_amount = 0            # 本 episode 总碰撞次数（上升沿计数）
        self.collision_marker_prev = False   # 上一帧是否处于碰撞状态
        self.collisions_at_prev_choice = 0   # 上一个 choice 时刻的累计 collision 数

        # stuck 检测
        self.no_progress_time = 0.0          # 最近一段时间内“几乎无 progress”的累计时长

        # --- pubs/subs ---
        self.pub_trans = rospy.Publisher("/sderi/transition", String, queue_size=1000)
        self.pub_episode = rospy.Publisher("/sderi/episode_summary", String, queue_size=100)

        rospy.Subscriber("/odom", Odometry, self.on_odom, queue_size=50)
        rospy.Subscriber("/scenario_reset", Int16, self.on_reset, queue_size=10)
        rospy.Subscriber("/sderi/choice", String, self.on_choice, queue_size=200)
        rospy.Subscriber("/sderi/final_goal", PoseStamped, self.on_final_goal, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=50)

        rospy.loginfo(
            "[eri_runtime_logger] started. goal=(%.3f, %.3f), thresh=%.2f, "
            "collision_distance=%.2f, stuck_time_thresh=%.1f, stuck_progress_eps=%.3f",
            self.goal_xy[0], self.goal_xy[1], self.goal_thresh,
            self.collision_distance, self.stuck_time_thresh, self.stuck_progress_eps
        )

    # --- Subscribers ---

    def on_scan(self, scan: LaserScan):
        """
        仅用于在线统计 collision_amount：
        - 若任意激光束距离 <= collision_distance 则认为当前帧处于碰撞状态；
        - 0->1 上升沿记一次碰撞。
        """
        if not scan.ranges:
            return

        is_collision = False
        for r in scan.ranges:
            if math.isinf(r) or math.isnan(r):
                continue
            if r <= self.collision_distance:
                is_collision = True
                break

        if is_collision and not self.collision_marker_prev:
            self.collision_amount += 1

        self.collision_marker_prev = is_collision

    def on_reset(self, msg: Int16):
        """
        Episode reset from arena（/scenario_reset）.

        若上一个 episode 仍未在 logger 内显式终止（success/stuck_timeout 等），
        则将最后一个 window 视为 timeout 步并结束该 episode（scenario_timeout）。
        然后重置所有 episode 级别的统计量，开始新的 episode。
        """
        now = rospy.Time.now().to_sec()

        # 如有未结算的 window 且 episode 尚未标记 done，则做一次 timeout 结算
        if (not self.episode_done) and (self.last_choice is not None) and (self.last_pose_xy is not None):
            self.emit_terminal_timeout(self.last_choice, now, self.last_pose_xy, reason="scenario_timeout")

        # 清空状态，开始新 episode
        self.last_choice = None
        self.last_pose_xy = None
        self.episode_id = int(msg.data)
        self.episode_started_time = None
        self.episode_sum_reward = 0.0
        self.episode_step_count = 0
        self.episode_done = False

        # 重置 collision & stuck 统计
        self.collision_amount = 0
        self.collision_marker_prev = False
        self.collisions_at_prev_choice = 0
        self.no_progress_time = 0.0

        rospy.loginfo("[eri_runtime_logger] episode reset -> %d", self.episode_id)

    def on_odom(self, od: Odometry):
        p = od.pose.pose.position
        self.last_pose_xy = (float(p.x), float(p.y))
        # 不在这里更新 episode_start_time，因为 episode 的语义以 /sderi/choice 为主

    def on_final_goal(self, msg: PoseStamped):
        self.goal_xy = (float(msg.pose.position.x), float(msg.pose.position.y))
        # rospy.loginfo(
            # "[eri_runtime_logger] final_goal updated to (%.3f, %.3f)",
            # self.goal_xy[0], self.goal_xy[1]
        # )

    def on_choice(self, msg: String):
        """
        每次 /sderi/choice：
        - 若存在 last_choice，则为 [last_choice -> now] 这一段生成一条 transition；
        - 否则生成一条中性的“首步” transition（reward=0，不参与 episode 统计）。
        """
        now = rospy.Time.now().to_sec()

        if self.episode_done:
            # 当前 episode 已经在 logger 内部终止（success/stuck/timeout），
            # 在真正收到下一次 /scenario_reset 前，忽略所有 /sderi/choice。
            return

        try:
            cur_payload = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn("[eri_runtime_logger] bad /sderi/choice JSON: %s", e)
            return

        # --- 正常 window：根据上一次 choice 计算 reward ---
        if self.last_choice is not None and self.last_pose_xy is not None:
            prev_t, prev_payload, prev_xy = self.last_choice
            dt = max(0.0, now - prev_t)

            # 初始化 episode_start_time（以第一段 window 的起始时间为 episode 起点）
            if self.episode_started_time is None:
                self.episode_started_time = prev_t

            cur_xy = self.last_pose_xy
            d_before = euclidean(prev_xy, self.goal_xy)
            d_now = euclidean(cur_xy, self.goal_xy)

            # progress：距离减小的量。负值截断为 0（不奖励后退），
            # stuck 检测用的是是否“小于 eps”。
            progress = max(0.0, d_before - d_now)

            # 窗口内新增的碰撞次数（对齐离线 get_collisions 的上升沿计数）
            collisions_delta = max(0, self.collision_amount - self.collisions_at_prev_choice)
            self.collisions_at_prev_choice = self.collision_amount

            # 奖励
            r_t = (self.w_progress * progress) \
                  - (self.w_time * dt) \
                  - (self.w_collision * collisions_delta)

            done = False
            term_reason = None

            # 1) 先判断是否到达目标
            if d_now < self.goal_thresh:
                r_t += self.success_bonus
                done = True
                term_reason = "success"
            else:
                # 2) stuck 检测：长时间几乎没有前进
                if progress < self.stuck_progress_eps:
                    self.no_progress_time += dt
                else:
                    self.no_progress_time = 0.0

                if self.no_progress_time >= self.stuck_time_thresh:
                    r_t += self.timeout_penalty
                    done = True
                    term_reason = "stuck_timeout"

            # --- 更新 episode 级别统计 ---
            self.episode_sum_reward += r_t
            self.episode_step_count += 1

            trans = {
                "episode_id": self.episode_id,
                "tau": float(prev_payload.get("tau")) if prev_payload.get("tau") is not None else None,
                "rho": float(prev_payload.get("rho")) if prev_payload.get("rho") is not None else None,
                "eri_rule": float(prev_payload.get("eri_rule")) if prev_payload.get("eri_rule") is not None else None,
                "eri_nn": float(prev_payload.get("eri_nn")) if prev_payload.get("eri_nn") is not None else None,
                "eri_act": float(prev_payload.get("eri_act")) if prev_payload.get("eri_act") is not None else None,
                "api_type": prev_payload.get("api_type"),
                "alpha": prev_payload.get("alpha"),
                "beta": prev_payload.get("beta"),
                "band": int(prev_payload.get("band")) if prev_payload.get("band") is not None else None,
                "acted_by": prev_payload.get("acted_by", "teacher"),
                "dt": float(dt),
                "progress": float(progress),
                "collision": int(collisions_delta),
                "reward": float(r_t),
                "done": bool(done),
                "term_type": term_reason if done else ""
            }
            rospy.loginfo(
                "[eri_runtime_logger] transition: ep=%d r=%.3f done=%s reason=%s",
                self.episode_id, r_t, done, term_reason
            )
            self.pub_trans.publish(String(data=json.dumps(trans)))

            if done:
                self.episode_done = True
                # 计算 episode 时长并发布 episode_summary
                self.publish_episode_summary(term_reason, now)
                # 清空 last_choice，直到下一次 /scenario_reset
                self.last_choice = None
                return

        else:
            # --- 首次 choice（或缺失 odom 时）：发一条中性 transition ---
            # 这条不参与 episode_sum_reward，不算步数，只用于让 buffer 结构完整
            trans = {
                "episode_id": self.episode_id,
                "tau": float(cur_payload.get("tau")) if cur_payload.get("tau") is not None else None,
                "rho": float(cur_payload.get("rho")) if cur_payload.get("rho") is not None else None,
                "eri_rule": float(cur_payload.get("eri_rule")) if cur_payload.get("eri_rule") is not None else None,
                "eri_nn": float(cur_payload.get("eri_nn")) if cur_payload.get("eri_nn") is not None else None,
                "eri_act": float(cur_payload.get("eri_act")) if cur_payload.get("eri_act") is not None else None,
                "api_type": cur_payload.get("api_type"),
                "alpha": cur_payload.get("alpha"),
                "beta": cur_payload.get("beta"),
                "band": int(cur_payload.get("band")) if cur_payload.get("band") is not None else None,
                "acted_by": cur_payload.get("acted_by", "teacher"),
                "dt": 0.0,
                "progress": 0.0,
                "collision": 0,
                "reward": 0.0,
                "done": False,
                "term_type": ""
            }
            rospy.loginfo("[eri_runtime_logger] publishing first neutral transition")
            self.pub_trans.publish(String(data=json.dumps(trans)))
            # episode_start_time 在下一次 window 时再设置

        # finally, arm this choice for next step window
        start_xy = self.last_pose_xy if (self.last_pose_xy is not None) else (0.0, 0.0)
        self.last_choice = (now, cur_payload, start_xy)

    # --- Helpers ---

    def classify_episode_end(self, duration_sec, collision_amount):
        if duration_sec >= self.timeout_threshold_sec:
            return "timeout"      # 对应旧的 TIMEOUT
        if collision_amount >= self.max_collisions:
            return "collision"    # 对应旧的 COLLISION
        return "success"          # 对应旧的 GOAL_REACHED

    def publish_episode_summary(self, term_type, t_end):
        if self.episode_started_time is None:
            duration = 0.0
        else:
            duration = max(0.0, t_end - self.episode_started_time)

        # 映射 term_type：
        # - stuck_timeout 在统计 SR 的意义上也属于 timeout
        # - scenario_timeout 需要用旧规则重新分类 success/collision/timeout
        if term_type == "scenario_timeout":
            mapped_type = self.classify_episode_end(duration, self.collision_amount)
        elif term_type == "stuck_timeout":
            mapped_type = "timeout"
        else:
            mapped_type = term_type

        summary = {
            "episode_id": self.episode_id,
            "term_type": mapped_type,
            "time_sec": float(duration),
            "steps": int(self.episode_step_count),
            "collisions": int(self.collision_amount),
            "sum_reward": float(self.episode_sum_reward)
        }
        rospy.loginfo("[eri_runtime_logger] episode_summary: %s", summary)
        self.pub_episode.publish(String(data=json.dumps(summary)))

    def emit_terminal_timeout(self, last_choice, now, cur_xy, reason="timeout"):
        """
        在 episode reset 时调用（场景结束但 logger 未提前结束该 episode），
        将最后一个 window 视为 timeout 步，追加 timeout_penalty，并终止 episode。
        """
        prev_t, prev_payload, prev_xy = last_choice
        dt = max(0.0, now - prev_t)

        if self.episode_started_time is None:
            self.episode_started_time = prev_t

        d_before = euclidean(prev_xy, self.goal_xy)
        d_now = euclidean(cur_xy, self.goal_xy)
        progress = max(0.0, d_before - d_now)

        collisions_delta = max(0, self.collision_amount - self.collisions_at_prev_choice)
        self.collisions_at_prev_choice = self.collision_amount

        r_t = (self.w_progress * progress) \
              - (self.w_time * dt) \
              - (self.w_collision * collisions_delta) \
              + self.timeout_penalty

        self.episode_sum_reward += r_t
        self.episode_step_count += 1

        trans = {
            "episode_id": self.episode_id,
            "tau": float(prev_payload.get("tau")) if prev_payload.get("tau") is not None else None,
            "rho": float(prev_payload.get("rho")) if prev_payload.get("rho") is not None else None,
            "eri_rule": float(prev_payload.get("eri_rule")) if prev_payload.get("eri_rule") is not None else None,
            "eri_nn": float(prev_payload.get("eri_nn")) if prev_payload.get("eri_nn") is not None else None,
            "eri_act": float(prev_payload.get("eri_act")) if prev_payload.get("eri_act") is not None else None,
            "api_type": prev_payload.get("api_type"),
            "alpha": prev_payload.get("alpha"),
            "beta": prev_payload.get("beta"),
            "band": int(prev_payload.get("band")) if prev_payload.get("band") is not None else None,
            "acted_by": prev_payload.get("acted_by", "teacher"),
            "dt": float(dt),
            "progress": float(progress),
            "collision": int(collisions_delta),
            "reward": float(r_t),
            "done": True,
            "term_type": reason
        }
        rospy.loginfo(
            "[eri_runtime_logger] terminal timeout transition: ep=%d r=%.3f reason=%s",
            self.episode_id, r_t, reason
        )
        self.pub_trans.publish(String(data=json.dumps(trans)))

        self.episode_done = True
        self.publish_episode_summary(reason, now)


if __name__ == "__main__":
    rospy.init_node("eri_runtime_logger")
    ERIRuntimeLogger()
    rospy.spin()
