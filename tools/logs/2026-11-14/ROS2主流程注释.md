# ROS 2 主流程注释

2026-11-14 19:13（UTC+8）

- 对照原工程的 `MyUART`、`GetObjs` 和 `InitDisributor`，给现在的 C++ 节点、Python 任务和视觉入口、启动配置及 Qt 客户端补了中文注释。
- 搜 `callback` 可找到关键入口。读动作流程时从 `TaskNode.on_frame()` 跳到 `Task.process_objects()`，再沿 `MyCMD` 跟到 `SerialNode`。
- 用 Python 的 `compile()` 和 `ast` 核对语法及执行语句，逐文件确认 C++、YAML 只加了注释，运行了 `git diff --check`。
