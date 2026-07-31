# lerobot 舵机标定文件（main_arm.json）

六个 STS3215 舵机的编码器行程与零位（含 shoulder_lift range_min=780 的放宽修改）。
lerobot 需要它在缓存目录里，新机器上安装：

```bash
mkdir -p ~/.cache/huggingface/lerobot/calibration/robots/so_follower
cp main_arm.json ~/.cache/huggingface/lerobot/calibration/robots/so_follower/
```

没有这份文件, connect() 会要求重新标定或用错零位（臂走错位置）。
它绑定这台臂的硬件（编码器零位），换臂必须重新标定而不是复用。
