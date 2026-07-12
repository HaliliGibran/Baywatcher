# Independent test audit

All metrics are conditional on already-successful ROI images.

| model | major single | major adaptive | worst scene adaptive | adaptive coverage | high-conf errors | high-conf error rate | worst-scene high-conf errors |
|---|---:|---:|---:|---:|---:|---:|---:|
| candidate | 92.75% | 92.96% | 92.60% | 98.87% | 28 | 1.8301% | 13 |
| current baseline | 83.92% | 85.82% | 75.22% | 97.85% | 78 | 6.3158% | 47 |

## Hard gates

- `adaptive_accuracy_not_lower`: PASS
- `worst_scene_adaptive_within_tolerance`: PASS
- `high_conf_error_rate_not_higher`: PASS
- `worst_scene_high_conf_error_count_not_higher`: PASS

Overall gate: **PASS**

## Runtime parity

- PyTorch / ONNX / manual MLP parity: **PASS**
- Maximum logit absolute differences: torch-onnx=5.7220459e-06, torch-manual=6.6757202e-06, onnx-manual=6.6757202e-06

The rolling two-frame audit excludes a low-confidence final frame when no successor exists. It cannot measure second-frame ROI extraction failure, target disappearance, search miss, or wait timeout.
