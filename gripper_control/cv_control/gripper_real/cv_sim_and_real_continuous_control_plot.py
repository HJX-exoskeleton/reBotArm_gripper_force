#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo + 真机并行视觉触觉夹爪控制（带曲线版）。

数据流：

                    +-> MuJoCo 100 Hz 仿真线程
  CV 手势目标 -----|
                    +-> 真机 100 Hz POS_VEL + FlexiTac 融合线程

MuJoCo 和真机同时接收CV目标，两条控制链并行运行；仿真物理
位置不作为真机的前置输入，FlexiTac只修正真机命令。本入口
默认显示位置、触觉力和电机力矩曲线。
"""

from __future__ import annotations

import sys

from cv_sim2real_continuous_control_plot import main


def run() -> int:
    """以CV并行、默认开启曲线的模式启动共享控制程序。"""
    return main(coupling="parallel")


if __name__ == "__main__":
    sys.exit(run())
