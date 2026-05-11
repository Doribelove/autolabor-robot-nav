#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import argparse
import pandas as pd


def compute_metrics(df, window=None):
    """
    df: 包含 episode_id, term_type, time_sec, collisions 的 DataFrame
    window: 若不为 None，则只在最后 window 个 episode 上计算
    """
    if window is not None and len(df) > window:
        df = df.tail(window)

    total_eps = len(df)
    if total_eps == 0:
        return None

    succ = df[df["term_type"] == "success"]

    sr = len(succ) / float(total_eps)

    at_all = df["time_sec"].mean()
    at_succ = succ["time_sec"].mean() if len(succ) > 0 else float("nan")

    ac_all = df["collisions"].mean()
    ac_succ = succ["collisions"].mean() if len(succ) > 0 else float("nan")

    return {
        "episodes": total_eps,
        "success_episodes": len(succ),
        "SR": sr,
        "AT_all": at_all,
        "AT_success": at_succ,
        "AC_all": ac_all,
        "AC_success": ac_succ,
    }


def main():
    parser = argparse.ArgumentParser(
        description="从 episode_summary_*.csv 计算 SR/AT/AC（支持整体和滑动窗口）。"
    )
    parser.add_argument("csv_path", help="episode_summary_xxx.csv 的路径")
    parser.add_argument("--window", type=int, default=None,
                        help="可选，只在最近 window 个 episode 上计算（例如 50）")
    args = parser.parse_args()

    if not os.path.exists(args.csv_path):
        print(f"文件不存在: {args.csv_path}")
        return

    df = pd.read_csv(args.csv_path)

    required = {"episode_id", "term_type", "time_sec", "collisions"}
    if not required.issubset(df.columns):
        print(f"CSV 缺少必要列: {required - set(df.columns)}")
        return

    metrics = compute_metrics(df, window=args.window)
    if metrics is None:
        print("没有 episode 数据。")
        return

    print("====== ERI-NN Online Metrics ======")
    print(f"文件: {os.path.basename(args.csv_path)}")
    if args.window:
        print(f"(最近 {args.window} 个 episode)")
    print(f"总 episode 数: {metrics['episodes']}")
    print(f"成功 episode 数: {metrics['success_episodes']}")
    print(f"Success Rate (SR): {metrics['SR'] * 100:.2f}%")
    print(f"Average Time (全部 episode, AT_all): {metrics['AT_all']:.3f} s")
    print(f"Average Time (成功 episode, AT_success): {metrics['AT_success']:.3f} s")
    print(f"Average Collisions (全部, AC_all): {metrics['AC_all']:.3f}")
    print(f"Average Collisions (成功, AC_success): {metrics['AC_success']:.3f}")


if __name__ == "__main__":
    main()
