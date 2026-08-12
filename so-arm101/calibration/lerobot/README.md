# LeRobot Servo Calibration (`main_arm.json`)

Device-specific encoder ranges and zero positions for the original SO-ARM101
follower arm. The file covers six STS3215 servos and includes the validated
`shoulder_lift` lower-range adjustment.

> **Hardware binding:** use this file only with the physical arm from which it
> was recorded. A replacement arm or servo must be calibrated independently.

## Validate the file

From `so-arm101/`:

```bash
python -m json.tool calibration/lerobot/main_arm.json >/dev/null
```

## Install for the original arm

LeRobot expects the calibration in its user cache:

```bash
mkdir -p ~/.cache/huggingface/lerobot/calibration/robots/so_follower
cp -i calibration/lerobot/main_arm.json \
  ~/.cache/huggingface/lerobot/calibration/robots/so_follower/main_arm.json
```

`cp -i` prevents silent replacement of an existing local calibration. Back up
and compare any existing file before choosing which one belongs to the
connected hardware.

Without a valid matching file, `connect()` may request recalibration or use
incorrect encoder limits. Stop immediately if the arm moves toward a hard
limit, reports an unexpected zero position, or does not match the recorded
pose. Recalibrate rather than modifying encoder values by trial and error.
