# UWB noise provenance

Fitted `2026-09-01` by `perception/uwb_sim/uwb_noise_model/fit_uwb_noise.py`.

| Parameter | Value | Dataset | Condition |
|---|---|---|---|
| `sigma_boresight_deg` | 4.795 | ETH-PBL DualAntenna AoA | static LOS lab rotation |
| `angle_sigma_slope_deg_per_deg` | 0.1649 | ETH-PBL | static LOS, binned |θ| |
| `aoa_fov_deg` | 90.0 | ETH-PBL | max reliable front hemisphere (paper ±45°) |
| ETH static `range_sigma_m` (not sim LOS if UTIL present) | 0.0873 | ETH-PBL TWR dist_mm demeaned | static LOS |
| `sigma_range_los_m` | 0.1087 | UTIAS UTIL TDOA /√2 | flight LOS if present else ident LOS |
| `range_bias_m` | -0.0008 | UTIL | same |
| `nlos_bias_mean_m` | 0.0870 | UTIL identification | NLOS materials |
| `nlos_bias_sigma_m` | 0.1273 | UTIL identification | NLOS |
| `nlos_sigma_mult` | 1.683 | UTIL | σ_NLOS / σ_LOS (TDOA) |
| `p_dropout_nlos` | 0.0883 | UTIL | frac \|e\| > 3 σ_LOS |
| flight inflation vs ident | 2.032 | UTIL | motion vs static TDOA |
| flight inflation vs ETH static | 1.245 | UTIL vs ETH-PBL | different radios |
| flight LOS σ used? | True | UTIL | UTIL measures TDOA d12=d2-d1, not TWR. range_sigma_m = sigma_tdoa/sqrt(2). LOS sigma from robust flight TDOA (0.1537 m). |

## Sanity vs papers

- ETH-PBL (Margiani et al. 2023 TIM): ~2.4° mean angular accuracy within ±45°. This fit: σ(|θ|≤45°)=4.79°, MAE=2.40°, boresight σ=4.79°.
- UTIL (Zhao et al. 2024 IJRR): TDOA dataset, DWM1000, identification LOS/NLOS + ~150 min flight. Paper Table 4 obstacle-free positioning RMSE ~10 cm (ESKF/batch) — that is **localization** RMSE, not raw TDOA σ. Raw TDOA σ from this fitter is the quantity mapped to `sigma_range_los_m` via 1/√2.

## Do not use

- ETH-PBL for flight dynamics, NLOS, or tunnel multipath.
- UTIL for angle/AoA (TDOA-only, no PDoA).
- Either dataset as Crazyflie-nano Phase-11 bench of *this* airframe.

**Angle noise has no flight validation.** Carry that into any paper limitations section.

## Re-run

```
python3 perception/uwb_sim/uwb_noise_model/fit_uwb_noise.py \
  --dataset <ETH-PBL rotation_*.log.gz dir> \
  --util <UTIL tree with identification-dataset/ and flight-dataset/> \
  --write-config
```

Download UTIL from [utiasdsl.github.io/util-uwb-dataset](https://utiasdsl.github.io/util-uwb-dataset/) ([github.com/learnsyslab/util-uwb-dataset](https://github.com/learnsyslab/util-uwb-dataset)). Place extracted CSVs under `perception/uwb_sim/uwb_noise_model/dataset/util-uwb-dataset/` (gitignored). The GitHub repo is parsers only; the CSVs are downloaded from the dataset site.
