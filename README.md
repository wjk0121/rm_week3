# 第三周：装甲板位姿与整车状态估计

C++17 / CMake。默认 NN 检测，也支持第二周的传统灯条检测。滤波器使用 11 维状态：
`[cx,vx,cy,vy,za,vz,theta,omega,r,dr,dh]`。`dr`、`dh` 分别是奇数板槽与偶数板槽的半径差、高度差。
`ekf9.hpp/cpp` 保留旧文件名以兼容第二周/原测试，实际类为 `VehicleEkf`，状态维数是 11。

## 依赖和构建

Ubuntu/WSL：

```bash
sudo apt update
sudo apt install build-essential cmake libopencv-dev python3-numpy python3-matplotlib
```

另外安装 Linux x86_64 ONNX Runtime C/C++ release，并将解压目录传给 CMake：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DONNXRUNTIME_ROOT="$HOME/onnxruntime"
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

已验证环境：GCC 15.2.0、OpenCV 4.10.0、ONNX Runtime 1.20.1、Python 3.14.4。
需要以下输入文件：`data/test.avi`、`data/imu.txt`、`models/Infantry-v8n.onnx`，传统方案还需 `models/mlp.onnx`。
模型与原第二周作业一致。大文件未纳入源码提交；重新 clone 后需按此目录结构放入题目提供的文件。
本数据视频 SHA-256：`864f7d83000885beee606c2dec2025da5bfe6cf6b91350b77d34d33b04716c3f`；
IMU SHA-256：`950b2695188565207fc606c201ca013125f9e632623fb6a6f10a26a49b6c9c00`。

缺失应用依赖会使配置失败。只运行不依赖 OpenCV/ONNX Runtime 的核心测试时，必须明确选择：

```bash
cmake -S . -B build-core -DBUILD_APP=OFF -DBUILD_VISION_TESTS=OFF
cmake --build build-core -j4
ctest --test-dir build-core --output-on-failure
```

## 数据准备

仓库不包含课程提供的实拍素材，请把它们放到下列路径后再运行：

| 路径 | 内容 |
| --- | --- |
| `data/test.avi` | 531 帧、1440×1080 实拍视频 |
| `data/imu.txt` | 531 条 IMU，`t qw qx qy qz`，平均 24.086 Hz |

模型随仓库提交：`models/Infantry-v8n.onnx` 供 NN 检测，`models/mlp.onnx` 供传统检测器的数字分类。

`--detection-cache` 指向的文件不存在时会自动重新推理并写出，不需要预先准备。

## 处理视频与输出

```bash
./build/rm_week3 data/test.avi --imu data/imu.txt --camera camera.yaml \
  --method nn --time-mode imu-index \
  --detection-cache results/revision/nn_detections.cache \
  --out results/revision/submission_nn.avi --csv results/revision/submission_nn.csv
python3 tools/plot_states.py results/revision/submission_nn.csv
python3 tools/analyze_tracking.py results/revision/submission_nn.csv
```

视频右上角显示全部 11 项状态，四种颜色表示四个固定板槽；细线中包含不可见面的模型预测。
`--debug-detections` 可另画检测和单板解算信息，默认只画整车重投影。
CSV 同时记录更新前/后的重投影、NIS、量测/融合数量、初始化/重捕、IMU 边界保持以及 P 的对角项。
曲线包含所有状态及模型条件下的 ±2σ，重捕时分段，不把不同参考板的角度强接起来。
每次运行生成同名 `*.csv.meta.json`，保存输入指纹、时间来源、噪声先验、运行条件和外参假设。

可用 `--method trad` 切换传统检测；`--no-video` 只输出 CSV，`--max-frames N` 用于短片检查。
传统检测单独采用 `traditional_corner_sigma_px=3.0`、位置下限 0.02 m、yaw 下限 5°；
NN 默认对应为 1.5 px、0.01 m、3°。这些是与检测方式对应的噪声先验，不是实测精度结论。
`--imu-offset` 和 `--plate-radius` 可覆盖配置，半径限制与滤波器使用相同的配置范围。
缓存只保存原始检测，不保存 PnP 或滤波结果；视频/模型/方法/颜色不匹配时拒绝复用。
修改检测器代码或阈值后应删除旧缓存或更换缓存文件名。

## 时间和坐标约定

IMU：每行 `timestamp_s qw qx qy qz`，假定四元数为 body→world 主动旋转；
格式错误、重复/倒序时间、零四元数都会报错，不排序或跳过记录来掩盖序号错误。
相机轴为右/下/前，IMU 轴假定前/左/上，`R_ci` 实际表示 camera→IMU 的轴向映射。
世界点为 `R_iw * (R_ci * p_camera + t_ic)`，长度单位为米。
本数据未提供自身平移；当前参考系假定 IMU 原点不平移，不能解释为具备场地绝对定位。

统一查询式：`t_query = t_image + img_imu_offset`，题目给定 `img_imu_offset=-0.009 s`。
在 IMU 两侧样本间进行最短弧 SLERP，不能对四元数分量直接线性插值或对欧拉角直接相减。

本 AVI 是 531 帧、标称 30 fps；531 条 IMU 覆盖约 22.00 s（24.086 Hz）。
默认 `imu-index` 仅采用“图像 i 的时间等于第 i 条 IMU 时间”的未验证假设，不代表录制对应关系已经证实。
该模式首帧查询早于第一条 IMU 9 ms，只能保持第一条姿态；CSV/元数据明确记录了这 1 帧。
其他超出 IMU 覆盖范围的帧报错。输出视频按平均采集间隔写入，CSV 保留各帧实际使用的非均匀时间。

获得真实图像时间戳后，推荐提供：

```csv
frame,t_image_s
0,0.520
1,0.562
2,0.603
```

以上只是格式示例，实际文件必须覆盖全部帧。运行 `--frame-times data/frame_times.csv` 会自动选择 `timestamps` 模式。
对于具有可信容器时间的数据，可选择 `--time-mode video --video-start-time <首帧在IMU时钟上的时间>`。
任何模式都不会由程序自行宣称绝对同步已经验证。

## 位姿、协方差与整车模型

先取 IPPE 两组候选，检查四角正深度和内法线，再分别做带安装姿态软先验的 LM 拟合。
普通步兵装甲板内法线 pitch 的机械先验为 -15°，pitch/滚转先验标准差均为 5°，允许未标定外参和小幅车体倾斜。
该角度来自普通装甲板安装模型，不是对本车做出的实测标定；可在 YAML 调整，或关闭 `constrain_plate_pitch`。
由白化像素/先验残差的 Jacobian 得到位姿参数协方差，变换为完整 `[x,y,z,yaw]` 协方差，保留交叉项。
噪声下限和 IMU 姿态标准差是可配置先验，不能当成已标定的实际误差。

车中心、高度、朝向采用短时恒速/恒角速度模型；半径与两组几何差缓慢变化。
每帧对四个固定板槽做几何门控、马氏距离排序和一对一匹配，再将同帧量测联合更新。
观测之间共享的 IMU 姿态误差进入 R 的非对角块，避免把同一份姿态当成多份独立证据。
新息中的角度差折回一个周期。P 用 Joseph 形式更新，检查对称正定性；拒绝非法协方差和非物理半径更新。
普通切板不重置整车状态；失配时预测，连续 12 帧无融合后重捕，超过 0.2 s 的帧间断重新建立轨迹。

## 验证与交付

```bash
python3 tools/check_cli.py
```

该集成检查要求完整检测缓存已生成，验证参数拒绝、真实时间戳入口、覆盖边界和缓存/原始检测结果一致。
四项 CTest 验证可见板跟踪、动态 IMU/SLERP、两组几何、相关量测联合更新、合成 NIS/NEES、位姿双解及像素协方差传播。

修改前后对比及限制见 [results/revision/validation.md](results/revision/validation.md)。
报告草稿见 [视觉第三周作业_姓名.md](视觉第三周作业_姓名.md)，提交前填写姓名与真实仓库/PR/commit 信息。
公开方案来源及许可见 [docs/OPEN_SOURCE_REFERENCES.md](docs/OPEN_SOURCE_REFERENCES.md)。
