import numpy as np

from static_detector import StaticDetector


def build_synthetic_imu(n_samples=400, seed=7):
    rng = np.random.default_rng(seed)

    acc = np.tile(np.array([0.0, 0.0, 9.81]), (n_samples, 1))
    gyr = np.zeros((n_samples, 3), dtype=float)
    mag = np.tile(np.array([25.0, 0.0, -40.0]), (n_samples, 1))

    # Low-noise static baseline.
    acc += rng.normal(0.0, 0.01, size=acc.shape)
    gyr += rng.normal(0.0, 0.002, size=gyr.shape)
    mag += rng.normal(0.0, 0.02, size=mag.shape)

    # Inject dynamic intervals to force variance ratios > 1.
    motion_ranges = [(80, 130), (220, 280)]
    for start, end in motion_ranges:
        acc[start:end] += rng.normal(0.0, 0.8, size=(end - start, 3))
        gyr[start:end] += rng.normal(0.0, 0.2, size=(end - start, 3))
        mag[start:end] += rng.normal(0.0, 0.8, size=(end - start, 3))

    return acc, gyr, mag


def sanity_check_variance_vs_static():
    acc, gyr, mag = build_synthetic_imu()

    detector_for_variance = StaticDetector(
        acc_threshold=0.08,
        gyr_threshold=0.03,
        mag_threshold=0.10,
        window_size=5,
        block_forward_steps=0,
    )
    var_ratios = detector_for_variance.varianceBatch(acc, gyr, mag)

    detector_for_flags = StaticDetector(
        acc_threshold=0.08,
        gyr_threshold=0.03,
        mag_threshold=0.10,
        window_size=5,
        block_forward_steps=0,
    )
    static_flags = detector_for_flags.updateBatch(acc, gyr, mag)

    high_variance_mask = np.any(var_ratios > 1.0, axis=0)
    violations = np.where(high_variance_mask & static_flags)[0]

    assert np.any(high_variance_mask), "Sanity test is weak: no sample exceeded normalized variance of 1.0"
    assert violations.size == 0, (
        "Sanity check failed: is_static() returned True while at least one normalized variance "
        f"component was > 1. Violation indices (first 20): {violations[:20].tolist()}"
    )

    print("Sanity check passed")
    print(f"Samples with any normalized variance > 1: {int(np.sum(high_variance_mask))}")
    print(f"Samples flagged static: {int(np.sum(static_flags))}")


if __name__ == "__main__":
    sanity_check_variance_vs_static()