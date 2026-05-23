#!/usr/bin/env python3
"""
IMU noise calibration via Allan variance analysis.

Reads a static IMU data file (7-column format: t gx gy gz ax ay az)
and extracts white noise density and bias random walk parameters
suitable for FAST-LIVO2 YAML configs.

Usage:
    python3 imu_calibrate.py --input imu_static.txt
    python3 imu_calibrate.py --input imu_static.txt --plot
    python3 imu_calibrate.py --input imu_static.txt --update-config config/odin.yaml
"""

import argparse
import sys

import numpy as np


def load_imu_data(path: str):
    """Load 7-column IMU data: t gx gy gz ax ay az. Returns (dt, gyro, accel)."""
    data = np.loadtxt(path)
    if data.ndim != 2 or data.shape[1] != 7:
        raise ValueError(
            f"Expected 7 columns (t gx gy gz ax ay az), got {data.shape[1]}"
        )
    timestamps = data[:, 0]
    if len(timestamps) < 2:
        raise ValueError("Need at least 2 IMU samples")
    dt = float(np.median(np.diff(timestamps)))
    gyro = data[:, 1:4]   # rad/s
    accel = data[:, 4:7]  # m/s^2
    return dt, gyro, accel


def allan_variance(signal: np.ndarray, dt: float):
    """
    Compute overlapping Allan variance for a 1D signal.

    Returns (taus, sigmas) where taus are cluster times in seconds
    and sigmas are the corresponding Allan deviations.
    """
    n = len(signal)
    max_clusters = n // 4
    if max_clusters < 2:
        return np.array([]), np.array([])

    # Logarithmically-spaced cluster sizes
    m_vals = np.unique(
        np.logspace(0, np.log10(max_clusters), num=100, dtype=int)
    )
    m_vals = m_vals[m_vals >= 1]

    taus = []
    sigmas = []
    for m in m_vals:
        k = n // m
        if k < 2:
            continue
        clusters = signal[: k * m].reshape(k, m)
        means = clusters.mean(axis=1)
        sigma_sq = np.sum(np.diff(means) ** 2) / (2.0 * (k - 1))
        taus.append(m * dt)
        sigmas.append(np.sqrt(sigma_sq))

    return np.array(taus), np.array(sigmas)


def fit_white_noise(taus: np.ndarray, sigmas: np.ndarray):
    """
    Fit white noise model on log-log Allan deviation.
    White noise has slope -1/2: sigma(tau) = N / sqrt(tau).

    Returns N such that sigma(tau=1) = N.
    """
    log_tau = np.log(taus)
    log_sigma = np.log(sigmas)

    # Find the white noise region: slope between -0.7 and -0.3
    slopes = np.diff(log_sigma) / np.diff(log_tau)
    wn_mask = np.zeros(len(taus), dtype=bool)
    for i in range(1, len(taus)):
        s = slopes[i - 1]
        if -0.7 <= s <= -0.3:
            wn_mask[i] = True
            wn_mask[i - 1] = True

    if wn_mask.sum() < 3:
        # Fallback: use first 25% of taus where slope is typically -1/2
        n_use = max(len(taus) // 4, 2)
        wn_mask[:n_use] = True

    # Fit: log(sigma) = log(N) - 0.5 * log(tau)
    x = log_tau[wn_mask]
    y = log_sigma[wn_mask]
    slope, intercept = np.polyfit(x, y, 1)
    # Force slope to -0.5 for the estimate
    log_N = np.mean(y + 0.5 * x)
    return np.exp(log_N)


def fit_bias_random_walk(taus: np.ndarray, sigmas: np.ndarray):
    """
    Fit bias random walk model on log-log Allan deviation.
    BRW has slope +1/2: sigma(tau) = K * sqrt(tau/3).

    Returns K such that sigma(tau) = K * sqrt(tau/3).
    """
    log_tau = np.log(taus)
    log_sigma = np.log(sigmas)

    slopes = np.diff(log_sigma) / np.diff(log_tau)
    brw_mask = np.zeros(len(taus), dtype=bool)
    for i in range(1, len(taus)):
        s = slopes[i - 1]
        if 0.3 <= s <= 0.7:
            brw_mask[i] = True
            brw_mask[i - 1] = True

    if brw_mask.sum() < 3:
        return None, False

    x = log_tau[brw_mask]
    y = log_sigma[brw_mask]
    # Fit: log(sigma) = log(K) + 0.5 * log(tau) - 0.5 * log(3)
    # → log(K) = mean(y - 0.5 * x + 0.5 * log(3))
    log_K = np.mean(y - 0.5 * x + 0.5 * np.log(3))
    return np.exp(log_K), True


def calibrate_axis(taus, sigmas, label):
    """Calibrate one IMU axis: extract N and K from Allan deviation."""
    N = fit_white_noise(taus, sigmas)
    K, brw_ok = fit_bias_random_walk(taus, sigmas)
    return N, K, brw_ok


def plot_allan(gyro_data, accel_data, dt, output_path):
    """Generate and save Allan deviation log-log plots."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("Warning: matplotlib not available, skipping plot.")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    labels = ["X", "Y", "Z"]
    colors = ["#e74c3c", "#2ecc71", "#3498db"]

    for i, (label, color) in enumerate(zip(labels, colors)):
        taus, sigmas = allan_variance(gyro_data[:, i], dt)
        if len(taus) > 0:
            ax1.loglog(taus, sigmas, color=color, alpha=0.6, linewidth=0.5)
            ax1.loglog(taus, sigmas, ".", color=color, markersize=2, label=f"Gyr {label}")

    ax1.set_xlabel("Cluster time τ (s)")
    ax1.set_ylabel("Allan deviation σ(τ) (rad/s)")
    ax1.set_title("Gyroscope Allan Deviation")
    ax1.legend()
    ax1.grid(True, alpha=0.3, which="both")

    for i, (label, color) in enumerate(zip(labels, colors)):
        taus, sigmas = allan_variance(accel_data[:, i], dt)
        if len(taus) > 0:
            ax2.loglog(taus, sigmas, color=color, alpha=0.6, linewidth=0.5)
            ax2.loglog(taus, sigmas, ".", color=color, markersize=2, label=f"Acc {label}")

    ax2.set_xlabel("Cluster time τ (s)")
    ax2.set_ylabel("Allan deviation σ(τ) (m/s²)")
    ax2.set_title("Accelerometer Allan Deviation")
    ax2.legend()
    ax2.grid(True, alpha=0.3, which="both")

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    print(f"Plot saved to {output_path}")


def update_yaml_config(config_path, gyr_cov, acc_cov, b_gyr_cov, b_acc_cov):
    """Write calibrated IMU parameters into a FAST-LIVO2 YAML config file."""
    try:
        import yaml
    except ImportError:
        print("Error: PyYAML is required for --update-config. Install with: pip3 install PyYAML")
        sys.exit(1)

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    if "imu" not in config:
        config["imu"] = {}

    config["imu"]["acc_cov"] = float(acc_cov)
    config["imu"]["gyr_cov"] = float(gyr_cov)
    config["imu"]["b_acc_cov"] = float(b_acc_cov)
    config["imu"]["b_gyr_cov"] = float(b_gyr_cov)

    with open(config_path, "w") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    print(f"Updated {config_path} with calibrated IMU parameters.")


def main():
    parser = argparse.ArgumentParser(
        description="IMU noise calibration via Allan variance for FAST-LIVO2"
    )
    parser.add_argument(
        "--input", "-i", required=True,
        help="Path to static IMU data file (7 columns: t gx gy gz ax ay az)"
    )
    parser.add_argument(
        "--plot", "-p", action="store_true",
        help="Generate Allan deviation log-log plots"
    )
    parser.add_argument(
        "--plot-output", default="allan_deviation.png",
        help="Output path for plot (default: allan_deviation.png)"
    )
    parser.add_argument(
        "--update-config",
        help="Path to FAST-LIVO2 YAML config to update with calibrated values"
    )
    args = parser.parse_args()

    # Load data
    print(f"Loading IMU data from: {args.input}")
    dt, gyro_data, accel_data = load_imu_data(args.input)
    fs = 1.0 / dt
    n_samples = len(gyro_data)
    duration = n_samples * dt

    print(f"  Samples: {n_samples}")
    print(f"  Sample period: {dt*1000:.2f} ms ({fs:.1f} Hz)")
    print(f"  Duration: {duration:.1f} s ({duration/3600:.2f} hours)")

    if duration < 300:
        print("  Warning: Short recording (< 5 min). White noise estimates may be "
              "usable, but bias random walk will likely be unresolved. "
              "Record 1+ hours for reliable BRW results.")

    axis_labels = ["X", "Y", "Z"]
    gyro_N = np.zeros(3)
    gyro_K = np.zeros(3)
    gyro_brw_ok = [False, False, False]
    accel_N = np.zeros(3)
    accel_K = np.zeros(3)
    accel_brw_ok = [False, False, False]

    print("\nComputing Allan variance...")
    for i, label in enumerate(axis_labels):
        taus_g, sigmas_g = allan_variance(gyro_data[:, i], dt)
        taus_a, sigmas_a = allan_variance(accel_data[:, i], dt)

        if len(taus_g) >= 3:
            gyro_N[i], gyro_K[i], gyro_brw_ok[i] = calibrate_axis(taus_g, sigmas_g, f"Gyr {label}")
        else:
            gyro_N[i] = gyro_K[i] = 0.0

        if len(taus_a) >= 3:
            accel_N[i], accel_K[i], accel_brw_ok[i] = calibrate_axis(taus_a, sigmas_a, f"Acc {label}")
        else:
            accel_N[i] = accel_K[i] = 0.0

    # Mean values across axes
    N_gyr_mean = float(np.mean(gyro_N))
    N_acc_mean = float(np.mean(accel_N))
    K_gyr_mean = float(np.mean(gyro_K)) if any(gyro_brw_ok) else None
    K_acc_mean = float(np.mean(accel_K)) if any(accel_brw_ok) else None

    # Map to FAST-LIVO2 config: cov_config = N² * fs (see plan for derivation)
    gyr_cov = N_gyr_mean ** 2 * fs
    acc_cov = N_acc_mean ** 2 * fs
    b_gyr_cov = (K_gyr_mean ** 2 * fs) if K_gyr_mean is not None else None
    b_acc_cov = (K_acc_mean ** 2 * fs) if K_acc_mean is not None else None

    # Report
    print("\n" + "=" * 64)
    print("                IMU CALIBRATION RESULTS")
    print("=" * 64)

    print("\nGyroscope (angular velocity):")
    print(f"  {'Axis':>6s}  {'ARW (rad/s/√Hz)':>18s}  {'BRW (rad/s²/√Hz)':>18s}")
    print(f"  {'-'*6}  {'-'*18}  {'-'*18}")
    for i, label in enumerate(axis_labels):
        brw_str = f"{gyro_K[i]:.6e}" if gyro_brw_ok[i] else "unresolved"
        print(f"  {label:>6s}  {gyro_N[i]:18.6e}  {brw_str:>18s}")
    print(f"  {'Mean':>6s}  {N_gyr_mean:18.6e}  ", end="")
    if K_gyr_mean is not None:
        print(f"{K_gyr_mean:18.6e}")
    else:
        print("unresolved")

    print("\nAccelerometer (linear acceleration):")
    print(f"  {'Axis':>6s}  {'VRW (m/s²/√Hz)':>18s}  {'BRW (m/s³/√Hz)':>18s}")
    print(f"  {'-'*6}  {'-'*18}  {'-'*18}")
    for i, label in enumerate(axis_labels):
        brw_str = f"{accel_K[i]:.6e}" if accel_brw_ok[i] else "unresolved"
        print(f"  {label:>6s}  {accel_N[i]:18.6e}  {brw_str:>18s}")
    print(f"  {'Mean':>6s}  {N_acc_mean:18.6e}  ", end="")
    if K_acc_mean is not None:
        print(f"{K_acc_mean:18.6e}")
    else:
        print("unresolved")

    # FAST-LIVO2 config mapping
    print("\n" + "-" * 64)
    print("FAST-LIVO2 Config Values  (cov = N² × fs,  fs = {:.1f} Hz)".format(fs))
    print("-" * 64)
    print(f"  acc_cov:    {acc_cov:.6e}")
    print(f"  gyr_cov:    {gyr_cov:.6e}")
    if b_acc_cov is not None:
        print(f"  b_acc_cov:  {b_acc_cov:.6e}")
    else:
        print(f"  b_acc_cov:  unresolved (keep current config value)")
    if b_gyr_cov is not None:
        print(f"  b_gyr_cov:  {b_gyr_cov:.6e}")
    else:
        print(f"  b_gyr_cov:  unresolved (keep current config value)")

    print("\n" + "-" * 64)
    print("Recommended odin.yaml / avia.yaml imu section:")
    print("-" * 64)
    print(f"  acc_cov: {acc_cov:.6e}")
    print(f"  gyr_cov: {gyr_cov:.6e}")
    print(f"  b_acc_cov: {b_acc_cov:.6e}" if b_acc_cov is not None else "  b_acc_cov: <unresolved>")
    print(f"  b_gyr_cov: {b_gyr_cov:.6e}" if b_gyr_cov is not None else "  b_gyr_cov: <unresolved>")
    print("=" * 64)

    if any(not ok for ok in [*gyro_brw_ok, *accel_brw_ok]):
        print("\nNote: Some bias random walk parameters are unresolved.")
        print("Collect a longer static recording (2+ hours) for reliable BRW values.")

    # Plot
    if args.plot:
        plot_allan(gyro_data, accel_data, dt, args.plot_output)

    # Update config
    if args.update_config:
        if b_gyr_cov is None or b_acc_cov is None:
            print("Error: Cannot update config with unresolved BRW parameters.")
            print("Use a longer recording or set values manually.")
            sys.exit(1)
        update_yaml_config(args.update_config, gyr_cov, acc_cov, b_gyr_cov, b_acc_cov)


if __name__ == "__main__":
    main()
