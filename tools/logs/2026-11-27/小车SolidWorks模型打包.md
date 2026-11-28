# 小车 SolidWorks 模型打包

2026-11-27。以 19 齿对称爪整车为源，把 Pack and Go 的装配体、零件和 STEP 归入 `tools/car_model/SolidWorks`。在包内重指派原工作盘的零件引用，将舵机嵌入装配并解除车轮和直流电机对下载目录的导入链接。源总装沿用原保存状态。

将模型目录复制到新的临时路径，用 SolidWorks 2025 打开主总装并重建。核对 46 个组件实例，包括两个默认压缩的旧爪组件。包外依赖、丢失引用、配合错误均为 0。主要操作使用 Pack and Go、ReplaceReferencedDocument、BreakLink、OpenDoc6 和 ForceRebuild3。

主装配体改名为「小车1127」，整理包内文件名与引用，把两组舵机子装配保存成随包文件。将整包复制到新路径，在原打包路径不可用时重新打开并复查组件和配合。
