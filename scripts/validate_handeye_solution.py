#!/usr/bin/env python3
"""Evaluate new, separately captured samples against a frozen hand-eye solution."""
import argparse
import json
import os
import sys

import numpy as np

from solve_handeye_checkerboard import metrics, sample_transforms


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", required=True)
    parser.add_argument("--solution", default="outputs/handeye/solution-candidate.json")
    parser.add_argument("--method", default="park")
    args = parser.parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sample_path = args.samples if os.path.isabs(args.samples) else os.path.join(root, args.samples)
    solution_path = args.solution if os.path.isabs(args.solution) else os.path.join(root, args.solution)
    with open(sample_path, encoding="utf-8") as stream:
        sample_data = json.load(stream)
    with open(solution_path, encoding="utf-8") as stream:
        solution = json.load(stream)
    method = solution["methods"][args.method]
    tool_from_camera = np.asarray(method["tool0_from_camera"]["matrix_4x4"])
    reference = np.asarray(method["base_from_target_training_mean"]["matrix_4x4"])
    accepted = [sample for sample in sample_data["samples"] if sample.get("accepted")]
    if not accepted:
        raise RuntimeError("validation dataset has no accepted samples")
    closures = []
    for sample in accepted:
        base_from_tool, camera_from_target = sample_transforms(sample)
        closures.append(base_from_tool @ tool_from_camera @ camera_from_target)
    result = metrics(closures, reference)
    print(json.dumps({"method": args.method,
                      "sample_ids": [x["sample_id"] for x in accepted],
                      "against_frozen_training_target": result},
                     indent=2, ensure_ascii=False))
    passed = (result["translation_mm"]["max"] <= 5.0 and
              result["rotation_deg"]["max"] <= 2.0)
    print("独立新增样本门禁（<=5 mm / <=2 deg）：%s" %
          ("通过" if passed else "未通过"))
    sys.exit(0 if passed else 2)


if __name__ == "__main__":
    main()
