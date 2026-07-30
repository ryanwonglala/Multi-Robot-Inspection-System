"""attempts_log.jsonl 分析：成功率-样本数曲线（论文图）+ 分段统计。

用法:
    .venv/bin/python scripts/17_analyze_log.py                 # 全量
    .venv/bin/python scripts/17_analyze_log.py --since 1785380000  # 只看某时间戳后(如今天)
输出: calibration/success_curve.png + 终端统计
"""

import argparse
import json
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).parent.parent
LOG = ROOT / "calibration" / "attempts_log.jsonl"

parser = argparse.ArgumentParser()
parser.add_argument("--since", type=float, default=0.0, help="只统计该 unix 时间戳之后的记录")
parser.add_argument("--window", type=int, default=10, help="滑动窗口大小(次)")
args = parser.parse_args()

rows = [json.loads(x) for x in LOG.read_text().splitlines() if x.strip()]
rows = [r for r in rows if r.get("t", 0) >= args.since]
if not rows:
    raise SystemExit("无记录")

by = Counter(r["result"] for r in rows)
n = len(rows)
print(f"共 {n} 次试抓: " + "  ".join(f"{k}={v}({v/n:.0%})" for k, v in by.most_common()))

# 首潜命中率(每个目标的第一次试探即成功的比例) —— 深度学习成熟度指标
first = {}
for r in rows:
    key = (r["pixel"][0] // 30, r["pixel"][1] // 30, r["t"] // 300)
    first.setdefault(key, r["result"])
fn = len(first)
fh = sum(1 for v in first.values() if v == "success")
print(f"目标轮次 {fn} 个, 首潜即中 {fh} ({fh/fn:.0%})")

# 滑动窗口成功率曲线(x=累计试抓次数)
ok = [1 if r["result"] == "success" else 0 for r in rows]
w = args.window
xs, ys = [], []
for i in range(len(ok)):
    lo = max(0, i - w + 1)
    xs.append(i + 1)
    ys.append(sum(ok[lo : i + 1]) / (i - lo + 1))

fig, ax = plt.subplots(figsize=(8, 4.5))
ax.plot(xs, ys, lw=2)
ax.set_xlabel(f"cumulative grasp attempts (window={w})")
ax.set_ylabel("success rate")
ax.set_ylim(0, 1.05)
ax.grid(alpha=0.3)
ns = [r.get("n_samples", 0) for r in rows]
ax2 = ax.twinx()
ax2.plot(xs, ns, lw=1, ls="--", color="tab:orange")
ax2.set_ylabel("samples in library", color="tab:orange")
fig.tight_layout()
out = ROOT / "calibration" / "success_curve.png"
fig.savefig(out, dpi=150)
print(f"曲线已保存: {out}")
