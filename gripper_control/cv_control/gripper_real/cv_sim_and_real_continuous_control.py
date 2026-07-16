#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo + 真机视觉触觉夹爪连续控制（无曲线版）。

该文件作为旧命令的并行控制入口：CV目标同时发布给
MuJoCo和真机独立线程，并复用已验证的底层组件，从而保证：

- 夹爪启动限速归零和 POS_VEL 安全控制；
- 归一化手势映射和丢手保持；
- FlexiTac 基线、滤波、接触锚点和位置退让；
- rebotarm_sim_transfer_cube.xml 的 gripper 执行器映射；
- 独立 100 Hz MuJoCo 物理与夹爪更新；
- 左右 8x16 MuJoCo 触觉和安全退出。

运行：

  python3 cv_sim_and_real_continuous_control.py

本入口默认关闭 Matplotlib 曲线窗口，其他参数与新实现一致。
"""

from __future__ import annotations

import sys

from cv_sim2real_continuous_control_plot import main


def run() -> int:
    """以无曲线、CV并行模式调用共享控制组件。"""
    if "--no-plot" not in sys.argv:
        sys.argv.append("--no-plot")
    return main(coupling="parallel")


if __name__ == "__main__":
    sys.exit(run())
