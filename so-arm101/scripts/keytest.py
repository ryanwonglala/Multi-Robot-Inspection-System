"""按键识别测试（不连机械臂）：验证方向键/WASD 能否被正确捕获。

在终端运行: .venv/bin/python scripts/keytest.py
按方向键或 WASD（可长按测试连击频率），每次识别会回显；q 退出。
"""

import select
import sys
import termios
import time
import tty

ARROWS = {"[A": "↑", "[B": "↓", "[C": "→", "[D": "←",
          "OA": "↑", "OB": "↓", "OC": "→", "OD": "←"}
WASD = {"w": "↑", "s": "↓", "d": "→", "a": "←"}

fd = sys.stdin.fileno()
old = termios.tcgetattr(fd)
tty.setcbreak(fd)
print("按方向键或 WASD（可长按），q 退出\n")
count, last_t, last_key = 0, 0.0, None
try:
    while True:
        r, _, _ = select.select([sys.stdin], [], [], 0.05)
        if not r:
            continue
        ch = sys.stdin.read(1)
        key = None
        raw = repr(ch)
        if ch == "\x1b":
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r:
                seq = sys.stdin.read(2)
                raw = repr("\x1b" + seq)
                key = ARROWS.get(seq)
        elif ch.lower() in WASD:
            key = WASD[ch.lower()]
        elif ch == "q":
            break

        now = time.time()
        rate = f" 连击 {1/(now-last_t):.0f}次/秒" if key == last_key and now - last_t < 0.5 else ""
        last_t, last_key = now, key
        count += 1
        if key:
            print(f"#{count:03d} 识别: {key}{rate}")
        else:
            print(f"#{count:03d} 未识别的输入: {raw}")
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
print("退出")
