#!/usr/bin/env python3
"""Process NACA0025 wind tunnel pressure data (ck70 experiment section 5)."""
import math
import os
import glob

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

DESKTOP = os.path.join(os.environ["USERPROFILE"], "Desktop")
OUT_DIR = os.path.join(os.path.dirname(__file__), "output")
os.makedirs(OUT_DIR, exist_ok=True)

# NACA 00xx thickness distribution (t=0.25 for NACA0025)
T = 0.25


def naca00_yt(x):
    return (T / 0.2) * (
        0.2969 * np.sqrt(np.clip(x, 1e-9, None))
        - 0.1260 * x
        - 0.3516 * x**2
        + 0.2843 * x**3
        - 0.1015 * x**4
    )


def naca00_dyt_dx(x):
    x = np.clip(x, 1e-9, None)
    return (T / 0.2) * (
        0.2969 * 0.5 / np.sqrt(x)
        - 0.1260
        - 2 * 0.3516 * x
        + 3 * 0.2843 * x**2
        - 4 * 0.1015 * x**3
    )


def load_case(path):
    df = pd.read_excel(path, sheet_name="temp", header=None)
    alpha_str = os.path.basename(path).replace("V20.xlsx", "").replace("A", "")
    alpha = float(alpha_str)
    xc = df.iloc[1, 1:].astype(float).values
    cp_up = df.iloc[2, 1:].astype(float).values
    cp_lo = df.iloc[3, 1:].astype(float).values
    return alpha, xc, cp_up, cp_lo


def integrate_coefficients(xc, cp_up, cp_lo, alpha_deg, ca_x_start=0.025):
    """Trapezoidal integration for Cn and Ca (body-axis), then wind-axis Cl, Cd."""
    alpha = math.radians(alpha_deg)
    dyt = naca00_dyt_dx(xc)
    dy_up = dyt
    dy_lo = -dyt

    cn = np.trapezoid(cp_lo - cp_up, xc)
    # Skip leading-edge singularity in Ca integral (lab standard practice)
    mask = xc >= ca_x_start
    ca = np.trapezoid(cp_lo[mask] * dy_lo[mask] - cp_up[mask] * dy_up[mask], xc[mask])

    cl = cn * math.cos(alpha) - ca * math.sin(alpha)
    cd = cn * math.sin(alpha) + ca * math.cos(alpha)
    return cn, ca, cl, cd


def main():
    files = sorted(glob.glob(os.path.join(DESKTOP, "A*V20.xlsx")))
    files = [f for f in files if "~$" not in f]

    results = []
    cases = {}
    for path in files:
        alpha, xc, cp_up, cp_lo = load_case(path)
        cn, ca, cl, cd = integrate_coefficients(xc, cp_up, cp_lo, alpha)
        results.append(
            {
                "alpha_deg": alpha,
                "Cn": cn,
                "Ca": ca,
                "Cl": cl,
                "Cd": cd,
                "L_D": cl / cd if abs(cd) > 1e-6 else float("nan"),
            }
        )
        cases[alpha] = (xc, cp_up, cp_lo)

    df = pd.DataFrame(results).sort_values("alpha_deg")
    df.to_csv(os.path.join(OUT_DIR, "coefficients.csv"), index=False, float_format="%.6f")
    df.to_csv(os.path.join(DESKTOP, "翼型实验_系数表.csv"), index=False, float_format="%.6f")
    print(df.to_string(index=False))

    # --- Fig 1: Cp distribution at alpha=8 deg ---
    plot_alpha = 8.0
    if plot_alpha not in cases and len(cases):
        plot_alpha = sorted(cases.keys(), key=lambda a: abs(a - 8))[0]
    xc, cp_up, cp_lo = cases[plot_alpha]

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(xc, cp_up, "b-o", markersize=4, label="上表面 Cp")
    ax.plot(xc, cp_lo, "r-s", markersize=4, label="下表面 Cp")
    ax.axhline(0, color="k", linewidth=0.8)
    ax.invert_yaxis()
    ax.set_xlabel("x/c")
    ax.set_ylabel("Cp")
    ax.set_title(f"NACA0025 翼型压强分布 (α={plot_alpha:g}°, V=20 m/s)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"Cp_distribution_alpha{plot_alpha:g}.png"), dpi=200)
    fig.savefig(os.path.join(DESKTOP, f"图1_压强分布_alpha{plot_alpha:g}.png"), dpi=200)
    plt.close(fig)

    # --- Fig 2: Cp at multiple angles (subplot style) ---
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    axes = axes.flatten()
    for ax, alpha in zip(axes, sorted(cases.keys())):
        xc, cp_up, cp_lo = cases[alpha]
        ax.plot(xc, cp_up, "b-", linewidth=1.2, label="上表面")
        ax.plot(xc, cp_lo, "r-", linewidth=1.2, label="下表面")
        ax.axhline(0, color="k", linewidth=0.5)
        ax.invert_yaxis()
        ax.set_title(f"α={alpha:g}°")
        ax.set_xlabel("x/c")
        ax.set_ylabel("Cp")
        ax.grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.suptitle("NACA0025 各攻角压强系数分布 (V=20 m/s)", fontsize=13)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "Cp_all_angles.png"), dpi=200)
    fig.savefig(os.path.join(DESKTOP, "图2_各攻角压强分布.png"), dpi=200)
    plt.close(fig)

    # --- Fig 3: Cl vs alpha ---
    alphas = df["alpha_deg"].values
    cls = df["Cl"].values
    cds = df["Cd"].values

    fig, ax1 = plt.subplots(figsize=(8, 5))
    ax1.plot(alphas, cls, "b-o", linewidth=2, label=r"$C_L$")
    ax1.set_xlabel("攻角 α (°)")
    ax1.set_ylabel(r"升力系数 $C_L$", color="b")
    ax1.tick_params(axis="y", labelcolor="b")
    ax1.grid(True, alpha=0.3)

    ax2 = ax1.twinx()
    ax2.plot(alphas, cds, "r-s", linewidth=2, label=r"$C_D$")
    ax2.set_ylabel(r"阻力系数 $C_D$", color="r")
    ax2.tick_params(axis="y", labelcolor="r")

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper left")
    ax1.set_title("NACA0025 升阻特性曲线 (V=20 m/s, Re≈const)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "Cl_Cd_vs_alpha.png"), dpi=200)
    fig.savefig(os.path.join(DESKTOP, "图3_升阻特性曲线.png"), dpi=200)
    plt.close(fig)

    # --- Fig 4: Polar (Cl vs Cd) ---
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(cds, cls, "g-o", linewidth=2)
    for a, cl, cd in zip(alphas, cls, cds):
        ax.annotate(f"{a:g}°", (cd, cl), textcoords="offset points", xytext=(5, 5), fontsize=9)
    ax.set_xlabel(r"阻力系数 $C_D$")
    ax.set_ylabel(r"升力系数 $C_L$")
    ax.set_title("升力-阻力极曲线")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "polar_curve.png"), dpi=200)
    fig.savefig(os.path.join(DESKTOP, "图4_升力阻力极曲线.png"), dpi=200)
    plt.close(fig)

    # --- Fig 5: L/D ratio ---
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(alphas, df["L_D"], "m-d", linewidth=2)
    ax.set_xlabel("攻角 α (°)")
    ax.set_ylabel("升阻比 L/D")
    ax.set_title("升阻比随攻角变化")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "L_over_D.png"), dpi=200)
    fig.savefig(os.path.join(DESKTOP, "图5_升阻比曲线.png"), dpi=200)
    plt.close(fig)

    print(f"\nFigures saved to: {OUT_DIR}")


if __name__ == "__main__":
    main()
