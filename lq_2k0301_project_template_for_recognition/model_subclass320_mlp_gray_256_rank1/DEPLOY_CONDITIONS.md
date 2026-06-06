# `mlp_gray_256` Deployment Conditions

- Source run dir: `D:\aaa走马观碑代码\yolo\project_root\out_torch_subclass320\mlp_gray_256`
- Rank: `1`
- Test accuracy on unaugmented real ROI: `93.38%`
- Predicted board time: `11.55 ms`
- Total score: `71.89`

## Input

- ROI size: `32x32`
- Input tensor shape: `1x1x32x32`
- Feature mode: `gray`

## Frontend preprocessing

Starting from the extracted ROI BGR image:

1. Resize ROI to `32x32`.
2. Convert to grayscale.
3. Convert to float in `[0, 1]`.
4. Normalize with:

```text
gray_norm = (gray - 0.449) / 0.226
```

5. Feed as `float32` NCHW blob: `1x1x32x32`.

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

- This model expects grayscale input only.
- Do not keep the old RGB `blobFromImage` path when switching to this model.
