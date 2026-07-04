# `mlp_rgb_128_s32_trainval_mild_seed42` Deployment Conditions

- Source run dir: `D:\aaa走马观碑代码\yolo\project_root\out_torch_boardroi_transfer_balanced_n300\mlp_rgb_128_s32_trainval_mild_seed42`
- Training/scoring ROI root: `D:\aaa走马观碑代码\yolo\灵眼pro320\板端实拍传输ROI_六大类均衡_n300`
- Rank: `1`
- Score accuracy: `99.53%`
- Test accuracy: `97.64%`
- Predicted board time: `12.62 ms`
- Total score: `86.20`

## Input

- ROI size: `32x32`
- Input tensor shape: `1x3x32x32`
- Feature mode: `rgb`

## Frontend Preprocessing

Starting from the extracted ROI BGR image:

1. Resize ROI to `32x32`.
2. Convert BGR to RGB channel order.
3. Convert to float in `[0, 1]`.
4. Normalize each channel with:

```text
R = (R - 0.485) / 0.229
G = (G - 0.456) / 0.224
B = (B - 0.406) / 0.225
```

5. Feed as `float32` NCHW blob: `1x3x32x32`.

## Output Classes

Output classes are 8 subclasses in this exact order:

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

## Board Postprocess Grouping

The board runtime must accumulate the 8 subclass probabilities into 3 strategy classes by class name:

```text
weapon  = 手枪 + 步枪 + 炸药包
supply  = 急救包 + 急救包（空白） + 望远镜
vehicle = 急救车 + 装甲车
```

## Board Code Requirements

- `BW_RECOG_MODEL_VARIANT` must be `BW_RECOG_MODEL_VARIANT_RGB32_SUBCLASS`.
- Active model root is `./model_boardroi_transfer_mlp_rgb_128_s32_rank1`.
- `class_names.json` must be deployed with this exact model because class order differs from the earlier 2026-06-20 package.

## Calibration

- `deploy_calibration.json` uses neutral defaults: temperature `1.0`, all logit bias values `0.0`.
- Decision thresholds remain low for probability accumulation: top1 average `0.30`, margin `0.01`.
