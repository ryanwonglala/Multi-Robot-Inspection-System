from __future__ import annotations

import math

from sensor_msgs.msg import LaserScan


def summarize_scan(scan: LaserScan | None) -> dict:
    if scan is None:
        return {
            'scan_received': False,
            'valid_ranges': 0,
            'min_range': None,
            'max_range': None,
            'mean_range': None,
        }

    values = [
        float(value)
        for value in scan.ranges
        if math.isfinite(value) and scan.range_min <= value <= scan.range_max
    ]
    if not values:
        return {
            'scan_received': True,
            'valid_ranges': 0,
            'min_range': None,
            'max_range': None,
            'mean_range': None,
        }

    return {
        'scan_received': True,
        'valid_ranges': len(values),
        'min_range': round(min(values), 4),
        'max_range': round(max(values), 4),
        'mean_range': round(sum(values) / len(values), 4),
    }


def aggregate_scan_summaries(samples: list[dict]) -> dict:
    received = [sample for sample in samples if sample.get('scan_received')]
    min_values = [
        sample['min_range']
        for sample in received
        if sample.get('min_range') is not None
    ]
    images = [sample.get('image_capture') or {} for sample in samples]
    captured_images = [
        image for image in images
        if image.get('status') == 'captured'
    ]
    return {
        'scan_samples': len(samples),
        'scan_samples_received': len(received),
        'overall_min_range': min(min_values) if min_values else None,
        'image_samples': len(images),
        'images_captured': len(captured_images),
    }
