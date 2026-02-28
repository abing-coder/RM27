# 手眼标定程序使用说明

手眼标定用于求**相机相对于云台**的位姿：`R_camera2gimbal`、`t_camera2gimbal`。标定结果需写入机器人配置（如 `demo.yaml`），供自瞄解算使用。

---

## 一、前提条件

1. **已完成相机内参标定**  
   配置里需有 `camera_matrix` 和 `distort_coeffs`。若尚未标定，先做内参标定并把这两项写入 `configs/calibration.yaml`（见下文）。

2. **标定板**  
   - **calibrate_handeye**：使用**圆点网格**（对称圆点），与内参标定相同。  
   - 在 `configs/calibration.yaml` 中需与实物一致：
     - `pattern_cols`、`pattern_rows`：列数、行数（如 10×7）
     - `center_distance_mm`：圆点中心间距（毫米）

3. **硬件**  
   - 相机、云台已固定好（相机随云台转动）。  
   - **采集数据时**需要 CAN 连接下位机（云台姿态由 IMU 四元数给出）。

---

## 二、配置 calibration.yaml

`configs/calibration.yaml` 需包含手眼标定与采集所需项，例如：

```yaml
# 标定板规格（与实物一致）
pattern_cols: 10
pattern_rows: 7
center_distance_mm: 40

# 云台系与 IMU 机体系的旋转（与你的 C 板/IMU 安装一致）
R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]

# 相机与 C 板（采集时用）
camera_name: "hikrobot"
exposure_ms: 3
gain: 10.0
vid_pid: "2bdf:0001"
quaternion_canid: 0x01
bullet_speed_canid: 0x110
send_canid: 0xff
can_interface: "can0"

# 以下两项来自「相机内参标定」结果，手眼标定前必须已有
camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distort_coeffs: [k1, k2, p1, p2, k3]
```

- 若还没有 `camera_matrix`、`distort_coeffs`，先做**内参标定**（见第三节），把输出的两段 YAML 复制进 `calibration.yaml`。

---

## 三、相机内参标定（若尚未做过）

1. 用**同一块圆点标定板**拍多张不同角度、不同位置的图（标定板尽量铺满画面、多角度）。
2. 图片命名为 `1.jpg`, `2.jpg`, ... 放到同一文件夹，如 `assets/img_calib/`。
3. 运行内参标定（使用包含 `pattern_cols/rows`、`center_distance_mm` 的配置，且该配置里可无 `camera_matrix`，程序只读棋盘格参数）：
   ```bash
   ./build/calibrate_camera assets/img_calib -c configs/calibration.yaml
   ```
4. 程序会逐张显示圆点检测结果，按任意键继续；最后在终端打印 `camera_matrix` 和 `distort_coeffs`。
5. 将这两段**复制到 `configs/calibration.yaml`**，保存。

---

## 四、采集“图像 + 云台四元数”数据

手眼标定需要多组「同一时刻的图像 + 该时刻云台姿态（四元数）」数据。用 **capture** 程序完成。

1. **连接**：相机 + 下位机 CAN（或能提供 IMU 四元数的设备）。
2. 运行：
   ```bash
   ./build/capture configs/calibration.yaml -o assets/img_with_q
   ```
   - 默认输出目录为 `assets/img_with_q`，可改为其他路径。
3. 操作：
   - 手持或固定**标定板**，让云台**多次转动**到不同姿态，保证标定板在画面内且圆点清晰。
   - 每换一个云台姿态，在窗口下按 **`s`** 保存当前帧图像和对应四元数，生成 `1.jpg`+`1.txt`、`2.jpg`+`2.txt`、…
   - 建议**至少 10～20 组**，且姿态差异尽量大（yaw/pitch/roll 都有变化），标定更稳。
   - 按 **`q`** 退出。
4. 说明：
   - `*.txt` 中为**四元数 w x y z**（一行四个数）。
   - 若没有 CAN/IMU，无法用 `capture`，需要自己实现“同步保存图像+云台姿态”的采集。

---

## 五、运行手眼标定

### 1. calibrate_handeye（标定板在云台原点附近）

适用：标定板可近似看作在“世界原点”附近（例如正对云台正前方、距离固定）。

```bash
./build/calibrate_handeye assets/img_with_q -c configs/calibration.yaml
```

- 第一个参数：存放 `1.jpg`、`1.txt`、`2.jpg`、`2.txt`、… 的**文件夹**（与 capture 的 `-o` 一致）。
- `-c`：使用的配置，需含 `pattern_cols/rows`、`center_distance_mm`、`R_gimbal2imubody`、`camera_matrix`、`distort_coeffs`。

程序会：

- 逐张读入图像和对应四元数；
- 检测圆点并显示，**按任意键**进入下一张；
- 最后在终端打印 YAML 格式结果，包含：
  - `R_camera2gimbal`
  - `t_camera2gimbal`

把这两段**复制到你的机器人配置**（如 `configs/demo.yaml`）中，与 `camera_matrix`、`distort_coeffs` 一起使用。

### 2. calibrate_robotworld_handeye（标定板位置任意）

适用：标定板固定在世界系中某一位置（例如地面、墙上），不在云台原点。

```bash
./build/calibrate_robotworld_handeye assets/img_with_q -c configs/calibration.yaml
```

参数含义同上。除 `R_camera2gimbal`、`t_camera2gimbal` 外，还会输出标定板在世界系下的位姿等信息，按需使用。

---

## 六、结果写入配置

将终端里打印的 YAML 中：

- `R_camera2gimbal`
- `t_camera2gimbal`

原样复制到**机器人配置**（如 `configs/demo.yaml` 或 `configs/standard4.yaml`）中，与已有的 `camera_matrix`、`distort_coeffs` 一起保存。自瞄/解算会从该配置读取手眼结果。

---

## 七、校验手眼结果（可选）

用 **handeye_test** 可粗略检查标定是否合理：程序根据当前手眼与内参，把一组世界坐标点投影到图像上并显示。

- 需要能跑通相机 + CBoard（CAN + 四元数）。
- 使用**带完整手眼与内参的配置**（如复制手眼结果后的 `demo.yaml`），且该配置需包含 handeye_test 所需的项（如 `height`、`grid_num`、`grid_size`、`delay` 等，见 `tests/handeye_test.cpp`）。若当前没有 `configs/handeye.yaml`，可复制一份机器人配置并加上这些项再运行。

---

## 八、流程小结

| 步骤 | 程序 | 说明 |
|------|------|------|
| 1 | 内参标定（若缺） | `calibrate_camera`，得到 `camera_matrix`、`distort_coeffs`，写入 `calibration.yaml` |
| 2 | 准备配置 | 在 `calibration.yaml` 中写好标定板参数、`R_gimbal2imubody`、相机/CAN 配置、内参 |
| 3 | 采集数据 | `capture`，多姿态下按 `s` 保存，得到 `img_with_q` 下多组 jpg+txt |
| 4 | 手眼标定 | `calibrate_handeye` 或 `calibrate_robotworld_handeye`，输入数据目录和 `calibration.yaml` |
| 5 | 写回配置 | 把输出的 `R_camera2gimbal`、`t_camera2gimbal` 写入机器人 yaml |

命令行帮助：

```bash
./build/capture --help
./build/calibrate_handeye --help
./build/calibrate_robotworld_handeye --help
```
