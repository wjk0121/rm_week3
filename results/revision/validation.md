# 2026-10-30 修改与实跑验证

输入为题目提供的完整 531 帧视频及 531 条 IMU。原始源码已备份至
`/home/wang/.cache/rm_week3_revision_20261030/before`，包含 SHA-256 清单。
旧程序从该备份独立编译并重新处理视频，输出 `baseline_nn.csv`；新程序输出 `submission_nn.csv`。
检测器源码/模型保持相同，缓存保存全部 794 次原始检测，后续比较复用相同角点。

## 修改

- 11 维整车状态，增加交替半径差 dr 和高度差 dh；保留四个固定板槽，普通切板不重置车心。
- IPPE 双候选 + 带不确定性的安装 pitch/滚转先验，不再只取单个平面解。
- 像素/先验 Jacobian 传播完整位置/yaw 协方差，初始化也保留位置/角度相关性。
- 同帧多板联合融合，共享 IMU 姿态噪声通过 R 的交叉块进入更新。
- 更新前 NIS、Joseph 协方差更新、正定检查、非法输入拒绝；统一半径机械范围。
- 真实逐帧时间戳入口，显式记录按序号配对的假设及首帧姿态保持。
- 全部状态、±2σ、运行元数据、独立原始角点评价器和回归比较脚本。

## 实拍对比

跳过前 30 帧，在相同的 501 帧上比较：

| 指标 | 修改前 | 修改后 |
| --- | ---: | ---: |
| 匹配帧 | 463/501 | 463/501 |
| 全部原始检测的模型重投影 RMS 中位数 | 29.88 px | 22.42 px |
| 全部原始检测的模型重投影 RMS 90% 分位 | 65.73 px | 55.54 px |
| 更新前主匹配板 RMS 中位数 | 23.78 px | 20.07 px |
| 更新前主匹配板 RMS 90% 分位 | 78.61 px | 60.61 px |

全部原始检测指标对每块检测板选择最近的朝前预测板，不用 EKF 是否接纳该量测筛选，避免只评价“容易被接受”的帧。
两版覆盖相同的 501 帧，检测及评价数量逐帧相同。中位数减少约 24.95%，90% 分位减少约 15.50%。
这仍是模型与检测的一致性，不是位置/速度的真实误差，也不是带真实板号的身份匹配精度。
两个版本都匹配成功的共同 433 帧上，更新前主匹配 RMS 中位数 22.42→20.30 px，90% 分位 69.25→61.18 px。

新版本完整处理 531 帧、794 次有效位姿量测，零位姿质量拒绝；初始化 1 次，实际重捕 0 次，531/531 帧协方差检查通过。
末段 5 秒估计中位数：偶数组半径约 0.2569 m，dr 约 -0.0078 m，dh 约 +0.0215 m。
这些值依赖当前模型与外参，不当作机械尺寸真值。

传统方案也做完整旧/新复跑。传统与 NN 对应角点的 RMS 差异中位数约 4.00 px（478 对、差异小于 20 px 的样本），
该比较不是角点真值标定。传统的膨胀与矩形端点存在更大误差，采用单独的 3 px 角点、0.02 m 位置和 5° yaw 噪声先验后，
跳过前 30 帧的匹配由 322/501 提升到 382/501，重捕由 1 次降为 0 次。
传统方案存在少量检测/板型错误，质量门仍会拒绝位姿；不以无限增大量测噪声来追求全部接纳。
主交付采用 NN，传统结果保存为 `submission_trad.avi/csv/png` 供对照。

## 合成与工程检查

- 完整构建与 CTest：4/4 通过；显式核心构建：3/3 通过。
- 40 个独立 Monte Carlo 试验：更新前 NIS/自由度 1.0031，末帧平均 NEES 10.8871（11 维）。
- 100 次像素噪声试验：全部解算成功，位置/yaw 四维平均 NEES 4.2301。
- 残余滚转回归：安装软先验允许数度滚转，避免硬 pitch 约束拒绝真实量测。
- 参数非法值、半径范围、时间源选择、IMU 覆盖、缓存/原始检测等集成检查通过。
- 输出视频 1440×1080、24.086 fps，完整解码确认 531 帧；抽查第 30、180、360 帧的四板与全部状态叠加。

初次尝试的硬安装约束在实拍上造成大量量测拒绝，因此未作为最终版本；最终软先验版本重新跑过完整视频。
合成统计只验证已设噪声/几何假设下的实现，不代表实拍协方差已经标定。

## 复跑

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build --output-on-failure
./build/rm_week3 --detection-cache results/revision/nn_detections.cache \
  --out results/revision/submission_nn.avi --csv results/revision/submission_nn.csv
python3 tools/plot_states.py results/revision/submission_nn.csv
python3 tools/check_cli.py

./build/evaluate_reprojection results/revision/baseline_nn.csv \
  results/revision/nn_detections.cache camera.yaml data/imu.txt results/revision/baseline_reprojection.csv 0
./build/evaluate_reprojection results/revision/submission_nn.csv \
  results/revision/nn_detections.cache camera.yaml data/imu.txt results/revision/updated_reprojection.csv
python3 tools/compare_runs.py results/revision/baseline_nn.csv results/revision/submission_nn.csv \
  results/revision/baseline_reprojection.csv results/revision/updated_reprojection.csv \
  --output results/revision/comparison.json
```

## 仍需外部资料确认的条件

逐帧图像时间未提供，保留“图像 i 时间取 IMU 行 i”的未验证假设与题目给定 -9 ms；首帧有一次姿态边界保持。
相机到 IMU 的精细安装外参、自身平移以及目标实测几何均未提供。程序增加了真实时间戳入口和可配置先验，无法凭软件修改补出这些真值。
实拍重投影还有残差，尤其在斜视或两块板量测不一致时，不能宣称达到像素级贴合或绝对空间精度。
当前运行的通过标准是：无跟踪覆盖回退、无重捕、协方差有效，且同一批全部原始检测上的中位数及尾部残差均改善；这些回归标准均通过。
