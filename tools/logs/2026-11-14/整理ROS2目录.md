# 整理 ROS 2 目录

- 视觉节点入口移到 vision_pkg/scripts，原视觉算法放在 modules，仍按 vision_pkg 导入。
- 启动和 robot.yaml 移到 gongchuang_bringup。单项靠近继续由 tests_pkg 提供，模型移到根目录 models。
- WSL 执行 bash tools/build.sh，七个包编译通过。bash tools/run.sh 验证无相机时只启动预览。bash tools/run.sh approach 验证 ONNX 推理、感知、HTTP 预览和靠近输出。
- 用固定检测结果核对靠近选最近目标和像素误差，用模拟命令对象验证 Task 到位停车。未连接实车串口，未部署或提交。
