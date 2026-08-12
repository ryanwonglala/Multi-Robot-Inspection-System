#!/usr/bin/env python3
from pathlib import Path
import shutil


DRIVER = Path.home() / "ros2_ws/src/wave_rover_driver/wave_rover_driver/wave_rover_node.py"
BACKUP = DRIVER.with_suffix(".py.before_feedback_patch")
MARKER = "# UGV02 feedback startup retry"


def main():
    if not DRIVER.exists():
        raise SystemExit(f"Driver not found: {DRIVER}")

    source = DRIVER.read_text(encoding="utf-8")
    if MARKER in source:
        print("Feedback patch is already installed.")
        return

    target = '        self.send_json({"T": 131, "cmd": 1})'
    if target not in source:
        raise SystemExit(
            "Could not find the T131 startup line. The driver was not changed."
        )

    if "\nimport time\n" not in f"\n{source}":
        lines = source.splitlines()
        insert_at = 1 if lines and lines[0].startswith("#!") else 0
        lines.insert(insert_at, "import time")
        source = "\n".join(lines) + ("\n" if source.endswith("\n") else "")

    replacement = "\n".join(
        [
            f"        {MARKER}",
            "        self.ser.reset_input_buffer()",
            "        time.sleep(1.0)",
            "        for _ in range(3):",
            '            self.send_json({"T": 131, "cmd": 1})',
            "            time.sleep(0.2)",
            '        self.send_json({"T": 130})',
        ]
    )

    patched = source.replace(target, replacement, 1)
    compile(patched, str(DRIVER), "exec")

    if not BACKUP.exists():
        shutil.copy2(DRIVER, BACKUP)
    DRIVER.write_text(patched, encoding="utf-8")

    print(f"Patched: {DRIVER}")
    print(f"Backup:  {BACKUP}")
    print("Next: rebuild wave_rover_driver with colcon.")


if __name__ == "__main__":
    main()
