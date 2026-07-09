#!/usr/bin/env python3
"""Export the current two-layer ONNX MLP to C++ arrays.

The generated arrays use PyTorch Linear layout:
    out[o] = bias[o] + sum_i input[i] * weight[o][i]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import numpy as np
import onnx
from onnx import numpy_helper


def _attr_map(node: onnx.NodeProto) -> Dict[str, object]:
    return {attr.name: onnx.helper.get_attribute_value(attr) for attr in node.attribute}


def _float_list(values: np.ndarray, per_line: int = 8) -> Iterable[str]:
    flat = values.astype(np.float32, copy=False).reshape(-1)
    for start in range(0, flat.size, per_line):
        chunk = flat[start:start + per_line]
        yield "    " + ", ".join(f"{float(v):.9g}f" for v in chunk) + ","


def _write_lf_text(path: Path, text: str) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as fout:
        fout.write(text)


def _find_two_gemms(model: onnx.ModelProto) -> List[onnx.NodeProto]:
    gemms = [node for node in model.graph.node if node.op_type == "Gemm"]
    if len(gemms) != 2:
        raise RuntimeError(f"expected exactly 2 Gemm nodes, found {len(gemms)}")
    return gemms


def _load_initializer_map(model: onnx.ModelProto) -> Dict[str, np.ndarray]:
    return {
        init.name: numpy_helper.to_array(init).astype(np.float32, copy=False)
        for init in model.graph.initializer
    }


def _extract_gemm_weights(
    gemm: onnx.NodeProto,
    initializers: Dict[str, np.ndarray],
) -> Tuple[np.ndarray, np.ndarray]:
    attrs = _attr_map(gemm)
    alpha = float(attrs.get("alpha", 1.0))
    beta = float(attrs.get("beta", 1.0))
    trans_a = int(attrs.get("transA", 0))
    trans_b = int(attrs.get("transB", 0))
    if alpha != 1.0 or beta != 1.0 or trans_a != 0:
        raise RuntimeError(
            f"unsupported Gemm attrs for {gemm.name}: alpha={alpha}, beta={beta}, transA={trans_a}"
        )
    if trans_b != 1:
        raise RuntimeError(f"unsupported Gemm attrs for {gemm.name}: transB={trans_b}, expected 1")
    if len(gemm.input) < 3:
        raise RuntimeError(f"Gemm {gemm.name} has no explicit weight+bias")

    weight = initializers[gemm.input[1]]
    bias = initializers[gemm.input[2]]
    if weight.ndim != 2 or bias.ndim != 1:
        raise RuntimeError(f"bad Gemm initializer rank for {gemm.name}")
    if weight.shape[0] != bias.shape[0]:
        raise RuntimeError(
            f"bad Gemm shape for {gemm.name}: weight={weight.shape}, bias={bias.shape}"
        )
    return weight, bias


def _read_metadata(model_dir: Path) -> Tuple[int, int, List[float], List[float]]:
    cfg_path = model_dir / "cls.json"
    if not cfg_path.exists():
        raise RuntimeError(f"missing metadata: {cfg_path}")
    cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
    input_size = int(cfg["image_size"])
    channels = int(cfg["input_channels"])
    mean = [float(v) for v in cfg["mean"]]
    std = [float(v) for v in cfg["std"]]
    if channels != len(mean) or channels != len(std):
        raise RuntimeError("mean/std length does not match channels")
    return input_size, channels, mean, std


def export_weights(model_dir: Path, header_path: Path, source_path: Path) -> None:
    onnx_path = model_dir / "cls.onnx"
    model = onnx.load(str(onnx_path))
    gemm1, gemm2 = _find_two_gemms(model)
    initializers = _load_initializer_map(model)
    fc1_weight, fc1_bias = _extract_gemm_weights(gemm1, initializers)
    fc2_weight, fc2_bias = _extract_gemm_weights(gemm2, initializers)

    input_size, channels, mean, std = _read_metadata(model_dir)
    input_elements = input_size * input_size * channels
    hidden = int(fc1_weight.shape[0])
    classes = int(fc2_weight.shape[0])

    if fc1_weight.shape != (hidden, input_elements):
        raise RuntimeError(f"bad fc1 shape: {fc1_weight.shape}, expected ({hidden}, {input_elements})")
    if fc2_weight.shape != (classes, hidden):
        raise RuntimeError(f"bad fc2 shape: {fc2_weight.shape}, expected ({classes}, {hidden})")

    header_path.parent.mkdir(parents=True, exist_ok=True)
    source_path.parent.mkdir(parents=True, exist_ok=True)

    header = f"""#pragma once

#include <cstddef>

namespace recognition_mlp_weights {{

constexpr int kInputSize = {input_size};
constexpr int kChannels = {channels};
constexpr int kInputElements = {input_elements};
constexpr int kHiddenUnits = {hidden};
constexpr int kClassCount = {classes};

extern const float kMean[kChannels];
extern const float kStd[kChannels];
extern const float kFc1Weight[kHiddenUnits * kInputElements];
extern const float kFc1Bias[kHiddenUnits];
extern const float kFc2Weight[kClassCount * kHiddenUnits];
extern const float kFc2Bias[kClassCount];

}} // namespace recognition_mlp_weights
"""
    _write_lf_text(header_path, header)

    lines = [
        '#include "recognition_mlp_weights.h"',
        "",
        "namespace recognition_mlp_weights {",
        "",
        "const float kMean[kChannels] = {",
        "    " + ", ".join(f"{v:.9g}f" for v in mean) + ",",
        "};",
        "",
        "const float kStd[kChannels] = {",
        "    " + ", ".join(f"{v:.9g}f" for v in std) + ",",
        "};",
        "",
        "const float kFc1Weight[kHiddenUnits * kInputElements] = {",
    ]
    lines.extend(_float_list(fc1_weight))
    lines.extend([
        "};",
        "",
        "const float kFc1Bias[kHiddenUnits] = {",
    ])
    lines.extend(_float_list(fc1_bias))
    lines.extend([
        "};",
        "",
        "const float kFc2Weight[kClassCount * kHiddenUnits] = {",
    ])
    lines.extend(_float_list(fc2_weight))
    lines.extend([
        "};",
        "",
        "const float kFc2Bias[kClassCount] = {",
    ])
    lines.extend(_float_list(fc2_bias))
    lines.extend([
        "};",
        "",
        "} // namespace recognition_mlp_weights",
        "",
    ])
    _write_lf_text(source_path, "\n".join(lines))

    print(f"exported {onnx_path}")
    print(f"  input={channels}x{input_size}x{input_size}, hidden={hidden}, classes={classes}")
    print(f"  header={header_path}")
    print(f"  source={source_path}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model-dir",
        default="lq_2k0301_project_template_for_recognition/model_boardroi_transfer_mlp_rgb_128_s32_rank1",
    )
    parser.add_argument(
        "--header",
        default="lq_2k0301_project_template_for_recognition/Code/User/inc/recognition_mlp_weights.h",
    )
    parser.add_argument(
        "--source",
        default="lq_2k0301_project_template_for_recognition/Code/User/src/recognition_mlp_weights.cc",
    )
    args = parser.parse_args()
    export_weights(Path(args.model_dir), Path(args.header), Path(args.source))


if __name__ == "__main__":
    main()
