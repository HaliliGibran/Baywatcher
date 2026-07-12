# RGB32 Manual MLP Deployment Conditions

## Model

- Architecture: `RGB 32x32 -> Linear(3072,128) -> ReLU -> Linear(128,8)`
- Training chain: `61_train_boardroi_png_grouped.py`
- Selected checkpoint: seed `42`, epoch `19`
- Runtime: compiled manual MLP weights
- ONNX: retained only for optional parity comparison

## Input

The extracted BGR ROI is converted to RGB planar `float32` in NCHW order and
normalized with:

```text
R = (R / 255 - 0.485) / 0.229
G = (G / 255 - 0.456) / 0.224
B = (B / 255 - 0.406) / 0.225
```

## Output Classes

```text
0 急救包
1 急救包（空白）
2 急救车
3 手枪
4 望远镜
5 步枪
6 炸药包
7 装甲车
```

The board combines subclass probabilities before making a decision:

```text
weapon  = 手枪 + 步枪 + 炸药包
supply  = 急救包 + 急救包（空白） + 望远镜
vehicle = 急救车 + 装甲车
```

## Calibration

- Temperature: `0.7896584472651295`
- Logit bias: all zero
- Decision top1 threshold: `0.30`
- Decision margin threshold: `0.01`
- Single-frame high-confidence thresholds remain compile-time values in
  `common.h`: top1 `0.90`, margin `0.70`

## Required Files

- `Code/User/inc/recognition_mlp_weights.h`
- `Code/User/src/recognition_mlp_weights.cc`
- `class_names.json`
- `deploy_calibration.json`
- `cls.onnx` when `BW_RECOG_MANUAL_MLP_COMPARE_ONNX=1`

See `TEST_AUDIT.md` for the frozen independent-test result and runtime parity.
