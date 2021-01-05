# 反铲单斗液压挖掘机工作装置设计

## 自动调参程序

自动调参使用方式：

1. 在main_running.m中设置各参数的范围、步长。
2. 运行main_running，每100000次迭代输出一次运行进度，当前最优损失。

其中，best_origin为最终取值。

## 绘制包络图动画

1. 加载best_origin.mat
2. 命令行输入： opt.draw = 1
3. 运行：Design(opt)

![](.\包络图.png)

