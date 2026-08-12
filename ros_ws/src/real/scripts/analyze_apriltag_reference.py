#!/usr/bin/env python3
"""Estimate a stable AprilTag reference pose from a burst of still images."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def load_calibration(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    camera_matrix = np.asarray(
        data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(
        data["distortion_coefficients"]["data"], dtype=np.float64)
    return camera_matrix, distortion


def mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    """Project the arithmetic matrix mean back onto SO(3)."""
    u, _, vt = np.linalg.svd(np.mean(rotations, axis=0))
    result = u @ vt
    if np.linalg.det(result) < 0:
        u[:, -1] *= -1
        result = u @ vt
    return result


def rotation_delta_deg(reference: np.ndarray, sample: np.ndarray) -> float:
    relative = reference.T @ sample
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("images", nargs="+", type=Path)
    parser.add_argument("--camera-yaml", required=True, type=Path)
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--tag-size", type=float, default=0.120)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--annotated", type=Path)
    parser.add_argument("--max-reprojection-error-px", type=float, default=2.0)
    args = parser.parse_args()

    camera_matrix, distortion = load_calibration(args.camera_yaml)
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    half = args.tag_size / 2.0
    # OpenCV ArUco corners are TL, TR, BR, BL. IPPE_SQUARE requires this
    # object-point order with +Y pointing toward the printed tag's top edge.
    object_points = np.asarray([
        [-half, half, 0.0],
        [half, half, 0.0],
        [half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float64)

    samples = []
    last_annotation = None
    for path in args.images:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            samples.append({"image": str(path), "valid": False,
                            "reason": "image_read_failed"})
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        id_values = [] if ids is None else ids.reshape(-1).tolist()
        matches = [index for index, value in enumerate(id_values)
                   if value == args.tag_id]
        if len(matches) != 1:
            samples.append({
                "image": str(path),
                "valid": False,
                "reason": "tag_not_found_once",
                "detected_ids": id_values,
            })
            continue

        image_points = corners[matches[0]].reshape(4, 2).astype(np.float64)
        success, rotation_vector, translation_vector = cv2.solvePnP(
            object_points, image_points, camera_matrix, distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not success:
            samples.append({"image": str(path), "valid": False,
                            "reason": "pose_estimation_failed"})
            continue

        projected, _ = cv2.projectPoints(
            object_points, rotation_vector, translation_vector,
            camera_matrix, distortion)
        residual = projected.reshape(4, 2) - image_points
        reprojection_error = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))
        translation = translation_vector.reshape(3)
        rotation, _ = cv2.Rodrigues(rotation_vector)
        center = np.mean(image_points, axis=0)
        side_lengths = [
            float(np.linalg.norm(image_points[(index + 1) % 4] - image_points[index]))
            for index in range(4)
        ]
        valid = (translation[2] > 0 and
                 reprojection_error <= args.max_reprojection_error_px)
        samples.append({
            "image": str(path),
            "valid": valid,
            "reason": None if valid else "pose_quality_rejected",
            "detected_ids": id_values,
            "center_px": center.tolist(),
            "mean_side_px": float(np.mean(side_lengths)),
            "reprojection_error_px": reprojection_error,
            "translation_camera_xyz_m": translation.tolist(),
            "rotation_vector_tag_to_camera": rotation_vector.reshape(3).tolist(),
            "rotation_matrix_tag_to_camera": rotation.tolist(),
        })
        if valid:
            annotation = image.copy()
            cv2.aruco.drawDetectedMarkers(
                annotation, [corners[matches[0]]],
                np.asarray([[args.tag_id]], dtype=np.int32))
            cv2.drawFrameAxes(
                annotation, camera_matrix, distortion,
                rotation_vector, translation_vector, args.tag_size / 2.0, 2)
            last_annotation = annotation

    candidates = [sample for sample in samples if sample["valid"]]
    if len(candidates) < 3:
        raise RuntimeError(
            f"Only {len(candidates)} valid Tag {args.tag_id} observations; need at least 3")

    translations = np.asarray(
        [sample["translation_camera_xyz_m"] for sample in candidates])
    median_translation = np.median(translations, axis=0)
    deviations_m = np.linalg.norm(translations - median_translation, axis=1)
    median_deviation = float(np.median(deviations_m))
    mad = float(np.median(np.abs(deviations_m - median_deviation)))
    outlier_limit_m = max(0.005, median_deviation + 3.0 * 1.4826 * mad)
    used = [sample for sample, deviation in zip(candidates, deviations_m)
            if deviation <= outlier_limit_m]
    if len(used) < 3:
        raise RuntimeError("Translation outlier filtering left fewer than 3 samples")

    translations = np.asarray(
        [sample["translation_camera_xyz_m"] for sample in used])
    rotations = [np.asarray(sample["rotation_matrix_tag_to_camera"])
                 for sample in used]
    translation_mean = np.mean(translations, axis=0)
    rotation_mean = mean_rotation(rotations)
    rotation_vector_mean, _ = cv2.Rodrigues(rotation_mean)
    rotation_deltas = [rotation_delta_deg(rotation_mean, rotation)
                       for rotation in rotations]
    centers = np.asarray([sample["center_px"] for sample in used])
    side_lengths = np.asarray([sample["mean_side_px"] for sample in used])
    reprojection_errors = np.asarray(
        [sample["reprojection_error_px"] for sample in used])

    result = {
        "schema_version": 1,
        "family": "tag36h11",
        "tag_id": args.tag_id,
        "tag_size_m": args.tag_size,
        "tag_size_definition": "black outer edge / detection-corner edge",
        "camera_calibration": str(args.camera_yaml),
        "coordinate_convention": {
            "camera_x": "right in image",
            "camera_y": "down in image",
            "camera_z": "forward from lens",
            "rotation": "tag frame to camera frame",
        },
        "quality": {
            "input_count": len(args.images),
            "detected_and_pose_valid_count": len(candidates),
            "used_count": len(used),
            "translation_outlier_limit_mm": outlier_limit_m * 1000.0,
            "translation_std_xyz_mm": (np.std(translations, axis=0) * 1000.0).tolist(),
            "rotation_delta_mean_deg": float(np.mean(rotation_deltas)),
            "rotation_delta_max_deg": float(np.max(rotation_deltas)),
            "reprojection_error_mean_px": float(np.mean(reprojection_errors)),
            "reprojection_error_max_px": float(np.max(reprojection_errors)),
        },
        "reference": {
            "translation_camera_xyz_m": translation_mean.tolist(),
            "range_m": float(np.linalg.norm(translation_mean)),
            "horizontal_bearing_deg": math.degrees(math.atan2(
                translation_mean[0], translation_mean[2])),
            "vertical_bearing_deg": math.degrees(math.atan2(
                translation_mean[1], translation_mean[2])),
            "rotation_vector_tag_to_camera": rotation_vector_mean.reshape(3).tolist(),
            "rotation_matrix_tag_to_camera": rotation_mean.tolist(),
            "tag_center_mean_px": np.mean(centers, axis=0).tolist(),
            "tag_center_std_px": np.std(centers, axis=0).tolist(),
            "tag_mean_side_px": float(np.mean(side_lengths)),
            "tag_side_std_px": float(np.std(side_lengths)),
        },
        "samples": samples,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    if args.annotated is not None and last_annotation is not None:
        args.annotated.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.annotated), last_annotation):
            raise RuntimeError(f"Could not write {args.annotated}")

    print(json.dumps({
        "output": str(args.output),
        "annotated": None if args.annotated is None else str(args.annotated),
        "used_count": len(used),
        "translation_camera_xyz_m": translation_mean.tolist(),
        "range_m": result["reference"]["range_m"],
        "horizontal_bearing_deg": result["reference"]["horizontal_bearing_deg"],
        "rotation_delta_max_deg": result["quality"]["rotation_delta_max_deg"],
        "translation_std_xyz_mm": result["quality"]["translation_std_xyz_mm"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
