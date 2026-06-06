# `mlp_rgb_256_128` Deployment Conditions

- Source run dir: `D:\aaa走马观碑代码\yolo\project_root\out_torch_subclass320\mlp_rgb_256_128`
- Rank: `3`
- Test accuracy on unaugmented real ROI: `99.01%`
- Predicted board time: `61.89 ms`
- Total score: `47.52`

## Input

- ROI size: `32x32`
- Input tensor shape: `1x3x32x32`
- Feature mode: `rgb`

## Frontend preprocessing

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

## Output

- Output classes are six subclasses in this exact order:

```text
0 急救包
1 望远镜
2 救护车
3 装甲车
4 枪支
5 炸药包
```

- Board-side postprocess must accept `6` logits instead of the previous `3`.
- No `deploy_calibration.json` is provided for this model.

## Notes

- This model keeps standard RGB input.
- Accuracy is high, but predicted board time is much worse than the rank-1 grayscale model.
