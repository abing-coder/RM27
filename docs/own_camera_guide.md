# 使用自己的相机运行本项目

## 一、支持的相机类型

| 类型 | camera_name | 说明 |
|------|-------------|------|
| 迈德威视工业相机 | `mindvision` | 需安装 MindVision SDK，配置 vid_pid |
| 海康工业相机 | `hikrobot` | 需安装 HikRobot MVS SDK，配置 vid_pid |
| **普通 USB 摄像头** | `usb` | 使用 OpenCV V4L2，设备如 `/dev/video0` |

主程序（如 `standard`、`uav`、`auto_aim_debug_mpc`）通过 **同一份 YAML 配置** 中的 `camera_name` 选择相机类型。

---

## 二、需要修改的配置项

### 1. 工业相机（MindVision / HikRobot）

在 `configs/` 下复制一份现有配置（如 `demo.yaml`），按你的相机修改：

```yaml
# 二选一
camera_name: "mindvision"
exposure_ms: 2
gamma: 0.5
vid_pid: "f622:d13a"   # lsusb 看到的 ID，格式 厂商:产品

# 或
camera_name: "hikrobot"
exposure_ms: 2
gain: 16
vid_pid: "2bdf:0001"
```

- **vid_pid**：在终端执行 `lsusb`，找到你的相机对应行，格式为 `idVendor:idProduct`（如 `2bdf:0001`）。

### 2. USB 摄像头

在同一份 YAML 中设置：

```yaml
camera_name: "usb"
usb_device: "video0"   # 不写则默认为 video0，多摄像头可为 video2 等

# 以下为 USB 相机专用，必须与相机分辨率一致
image_width: 1280
image_height: 720
usb_frame_rate: 60
usb_exposure: 250      # 部分相机支持 1–80000
usb_gamma: 160
usb_gain: 10
```

- **设备名**：`ls /dev/video*` 查看，一般为 `video0`。
- **分辨率/帧率**：按你的摄像头实际支持填写，否则可能打不开或报错。

---

## 三、标定（必做）

换相机或换镜头后，**必须重新标定**，否则解算位姿会错。需要得到并写入同一 YAML 的 4 项：

| 配置项 | 含义 |
|--------|------|
| `camera_matrix` | 相机内参 3×3（fx, 0, cx, 0, fy, cy, 0, 0, 1） |
| `distort_coeffs` | 畸变系数（k1, k2, p1, p2, k3） |
| `R_camera2gimbal` | 手眼标定：相机到云台的旋转矩阵（行优先 9 个数） |
| `t_camera2gimbal` | 手眼标定：相机到云台的平移向量（3 个数） |

### 步骤概览

1. **内参标定**（得到 `camera_matrix`、`distort_coeffs`）  
   - 打印棋盘格，用 `capture` 采集多张不同角度的图。  
   - 运行 `calibrate_camera`，把输出的 `camera_matrix` 和 `distort_coeffs` 写入你的 YAML。

2. **手眼标定**（得到 `R_camera2gimbal`、`t_camera2gimbal`）  
   - 需要云台/机器人能提供当前位姿。  
   - 使用 `calibrate_handeye` 或 `calibrate_robotworld_handeye`，将输出的矩阵和向量写入 YAML。

标定程序用法见仓库内 `calibration/` 目录和 README。  
若暂时**没有云台、只做识别**，可先用任意一组内参（如 demo 里的）跑通流程，但**角度/距离解算会不准**。

---

## 四、怎么运行

### 1. 编译

```bash
cd /home/ubuntu/桌面/sp_vision_25
cmake -B build
make -C build -j$(nproc)
```

### 2. 不接相机、不接下位机时（用视频跑自瞄）

用自带视频测试（不依赖相机与 CAN）：

```bash
./build/auto_aim_test assets/demo/1
```

- 会查找 `assets/demo/1.avi` / `1.mp4` / `1.mov` 等；  
- 默认使用 `configs/demo.yaml`，可通过 `-c` 指定配置，例如：  
  `./build/auto_aim_test assets/demo/1 -c configs/your_config.yaml`

### 3. 接相机但无 CAN（仅测视觉，不发下位机）

若只有相机、没有 CAN/下位机，可在配置中关闭 CAN，用默认云台姿态跑自瞄（不发送控制量）：

在 YAML 的 cboard 段添加或取消注释：

```yaml
can_disabled: true
```

然后照常运行：

```bash
./build/standard configs/你的配置.yaml
```

- 不会出现 “Error getting interface index!” 或卡在 “Waiting for q...”；云台姿态用单位四元数，不向 CAN 发数据。

### 4. 接相机 + 下位机（实机自瞄）

使用你改好的配置运行主程序，例如步兵标准自瞄：

```bash
./build/standard configs/你的配置.yaml
```

或带 MPC 的调试程序（仍从同一配置读相机类型和标定）：

```bash
./build/auto_aim_debug_mpc configs/你的配置.yaml
```

- 确保：  
  - 配置文件里 `camera_name`、工业相机的 `vid_pid` 或 USB 的 `usb_device`/分辨率等已按上面改好；  
  - 同一 YAML 里已写好该相机的 `camera_matrix`、`distort_coeffs`、`R_camera2gimbal`、`t_camera2gimbal`。

### 5. 仅测试相机是否正常

- 工业相机：  
  `./build/camera_test configs/你的配置.yaml`  
- USB 相机：  
  `./build/usbcamera_test -n video0`（会从默认或指定 config 读 USB 相关参数，若报错缺 key，在对应 YAML 中补全 `image_width`、`image_height`、`usb_*` 等）。

---

## 五、小结：你要动的“值”和顺序

1. **选相机类型**：在 YAML 里设 `camera_name` 为 `mindvision` / `hikrobot` / `usb`。  
2. **填该类型的参数**：工业相机填 `vid_pid`、曝光等；USB 填 `usb_device`、`image_width`、`image_height`、`usb_*`。  
3. **标定**：用标定程序得到并写入 `camera_matrix`、`distort_coeffs`、`R_camera2gimbal`、`t_camera2gimbal`。  
4. **运行**：  
   - 无实机：`./build/auto_aim_test 视频路径 [-c 你的配置.yaml]`  
   - 有实机：`./build/standard configs/你的配置.yaml`（或其它 main 程序 + 同一配置）。

按上述修改并标定后，即可用你自己的相机跑通本项目。
