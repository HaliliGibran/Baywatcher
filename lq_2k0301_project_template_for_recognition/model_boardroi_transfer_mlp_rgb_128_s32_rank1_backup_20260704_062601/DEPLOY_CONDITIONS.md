# `mlp_rgb_128_s32_trainval_noaug_seed7` Deployment Conditions

- Source run dir: `D:\aaa走马观碑代码\yolo\project_root\out_torch_boardroi_transfer\mlp_rgb_128_s32_trainval_noaug_seed7`
- Accuracy score standard: full `灵眼pro320\板端实拍传输ROI` ROI dataset
- Rank: `1`
- Score accuracy: `97.20%`
- Test accuracy: `86.05%`
- Predicted board time: `12.43 ms`
- Total score: `80.56`

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
3 望远镜
4 手枪
5 步枪
6 炸药包
7 装甲车
```

## Board Postprocess Grouping

The board runtime must accumulate the 8 subclass probabilities into 3 strategy classes:

```text
weapon  = 手枪 + 步枪 + 炸药包
supply  = 急救包 + 急救包（空白） + 望远镜
vehicle = 急救车 + 装甲车
```

## Calibration

- `deploy_calibration.json` uses neutral defaults: temperature `1.0`, all logit bias values `0.0`.
- Keep `BW_RECOG_MODEL_VARIANT` set to `BW_RECOG_MODEL_VARIANT_RGB32_SUBCLASS`.
