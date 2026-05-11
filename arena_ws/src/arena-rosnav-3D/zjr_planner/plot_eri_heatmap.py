#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import argparse
import numpy as np
import torch
import matplotlib.pyplot as plt


DEFAULT_OUT_DIR = "/home/robot/catkin_arena/src/zjr_planner/eri_logs/heatmaps"


def resolve_model_path(save_dir: str, runtime_tag: str) -> str:
    runtime_tag = str(runtime_tag).lower()
    if runtime_tag == "best":
        return os.path.join(save_dir, "best", "exports", "eri_net_ts.pt")
    return os.path.join(save_dir, "latest", "exports", "eri_net_ts.pt")


@torch.no_grad()
def infer_e_from_model(model, x: torch.Tensor, use_sample: bool = False) -> torch.Tensor:
    """
    x: (B,2) with [tau, rho]
    return e in [0,1], shape (B,)
    Supports:
      - scalar head: out -> (B,1) or (B,)
      - beta head: out -> (alpha, beta) as tuple/list OR out numel==2 per sample
    """
    out = model(x)

    # Case A: tuple/list (alpha, beta)
    if isinstance(out, (tuple, list)) and len(out) == 2:
        alpha = out[0].reshape(-1).clamp_min(1e-3)
        beta  = out[1].reshape(-1).clamp_min(1e-3)
        if use_sample:
            dist = torch.distributions.Beta(alpha, beta)
            e = dist.sample().clamp(1e-6, 1 - 1e-6)
        else:
            e = (alpha / (alpha + beta)).clamp(1e-6, 1 - 1e-6)
        return e

    # Case B: tensor with 2 params per sample
    if torch.is_tensor(out):
        # If out is (B,2): alpha,beta
        if out.ndim == 2 and out.shape[1] == 2:
            alpha = out[:, 0].reshape(-1).clamp_min(1e-3)
            beta  = out[:, 1].reshape(-1).clamp_min(1e-3)
            if use_sample:
                dist = torch.distributions.Beta(alpha, beta)
                e = dist.sample().clamp(1e-6, 1 - 1e-6)
            else:
                e = (alpha / (alpha + beta)).clamp(1e-6, 1 - 1e-6)
            return e

        # Scalar model: out in [0,1] (sigmoid)
        e = out.reshape(-1).float()
        e = torch.clamp(e, 0.0, 1.0)
        return e

    raise RuntimeError(f"Unknown model output type: {type(out)}")


def main():
    parser = argparse.ArgumentParser(description="Plot ERI heatmap over (tau, rho) by calling TorchScript model.")
    parser.add_argument("--save_dir", default="/home/robot/catkin_arena/src/zjr_planner/imilearning_model",
                        help="Root dir that contains latest/ and best/")
    parser.add_argument("--runtime_tag", choices=["latest", "best"], default="latest",
                        help="Which exported model to use: latest or best")
    parser.add_argument("--out_dir", default=DEFAULT_OUT_DIR,
                        help=f"Output directory (default: {DEFAULT_OUT_DIR})")
    parser.add_argument("--out_name", required=True,
                        help="Output filename, e.g. latest_heatmap.png")
    parser.add_argument("--tau_min", type=float, default=0.02)
    parser.add_argument("--tau_max", type=float, default=0.06)
    parser.add_argument("--rho_min", type=float, default=0.0)
    parser.add_argument("--rho_max", type=float, default=0.03)
    parser.add_argument("--n_tau", type=int, default=101, help="Grid resolution along tau")
    parser.add_argument("--n_rho", type=int, default=101, help="Grid resolution along rho")
    parser.add_argument("--batch", type=int, default=4096, help="Batch size for inference")
    parser.add_argument("--use_sample", action="store_true",
                        help="If set, use Beta.sample() (stochastic). Default is expectation (deterministic).")
    parser.add_argument("--normalize_rho", action="store_true",
                    help="If set, feed rho_norm=rho/rho_scale_max to model, but keep axis as raw rho.")
    parser.add_argument("--rho_scale_max", type=float, default=0.03,
                    help="Denominator for rho normalization when --normalize_rho is set.")


    # For mapping to [0,1] colorbar.
    # If your eri_min/max are 0/10 in runtime, normalized e is still [0,1].
    # We'll plot normalized e directly to match your requested colorbar 0.2~0.8.
    parser.add_argument("--cmin", type=float, default=0.2, help="Colorbar min (display)")
    parser.add_argument("--cmax", type=float, default=0.8, help="Colorbar max (display)")

    args = parser.parse_args()

    model_path = resolve_model_path(args.save_dir, args.runtime_tag)
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model not found: {model_path}")

    os.makedirs(args.out_dir, exist_ok=True)
    out_path = os.path.join(args.out_dir, args.out_name)

    # Load TorchScript model
    model = torch.jit.load(model_path, map_location="cpu")
    model.eval()

    # Build grid
    tau_vals = np.linspace(args.tau_min, args.tau_max, args.n_tau, dtype=np.float32)
    rho_vals = np.linspace(args.rho_min, args.rho_max, args.n_rho, dtype=np.float32)

    # We'll fill heatmap[rho_idx, tau_idx]
    heat = np.zeros((args.n_rho, args.n_tau), dtype=np.float32)

    # Vectorize grid points
    TT, RR_raw = np.meshgrid(tau_vals, rho_vals)  # RR_raw used for axis display
    RR = RR_raw
    if args.normalize_rho:
        denom = max(1e-8, float(args.rho_scale_max))
        RR = np.clip(RR_raw / denom, 0.0, 1.0)

    pts = np.stack([TT.reshape(-1), RR.reshape(-1)], axis=1).astype(np.float32)
    x_all = torch.from_numpy(pts)

    # Batch inference
    N = x_all.shape[0]
    out_e = torch.empty((N,), dtype=torch.float32)

    with torch.no_grad():
        for i in range(0, N, args.batch):
            xb = x_all[i:i + args.batch]
            eb = infer_e_from_model(model, xb, use_sample=args.use_sample)
            out_e[i:i + args.batch] = eb.cpu()

    heat[:, :] = out_e.numpy().reshape(args.n_rho, args.n_tau)

    # Plot
    plt.figure()
    im = plt.imshow(
        heat,
        origin="lower",
        aspect="auto",
        extent=[args.tau_min, args.tau_max, args.rho_min, args.rho_max],
        vmin=args.cmin,
        vmax=args.cmax,
        interpolation="nearest"
    )
    title = f"Predicted ERI: {args.runtime_tag.capitalize()}"
    plt.title(title)
    plt.xlabel("tau")
    plt.ylabel("rho")
    cbar = plt.colorbar(im)
    cbar.set_label("ERI Value")

    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    print(f"[OK] Saved heatmap to: {out_path}")
    print(f"[INFO] model: {model_path}")
    print(f"[INFO] grid: tau({args.n_tau}), rho({args.n_rho}), sample={args.use_sample}")
    print("[DEBUG] heat min/max/mean/std:",
      float(np.min(heat)), float(np.max(heat)),
      float(np.mean(heat)), float(np.std(heat)))



if __name__ == "__main__":
    main()
