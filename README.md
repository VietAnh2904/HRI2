# UR3e Lập kế hoạch kỹ năng bằng LLM

Mô phỏng robot **UR3e trên ROS 2 Humble**, trong đó robot nhận yêu cầu bằng ngôn ngữ tự nhiên, gửi yêu cầu đến một **LLM tương thích OpenAI** để tạo kế hoạch kỹ năng, kiểm tra tính hợp lệ của kế hoạch, sau đó thực thi thông qua **MoveIt 2** và **Gazebo Classic**.

* **ROS package:** `ur3_llm_control`
* Cấu hình bài tập của sinh viên tại `src/ur3_llm_control/config/student_config.yaml`
* Với **MSSV `23020722`**:

  * Zone A ← xanh dương
  * Zone B ← đỏ
  * Zone C ← vàng
* [Video demo](https://drive.google.com/file/d/1_IpHvAtT0lvKuvCsBJHan8WBQUIoCb5-/view?usp=sharing)
* [GitHub repository](https://github.com/VietAnh2904/HRI2)

## Cấu trúc repository

```text
HRI2/
├── README.md
├── scripts/
│   ├── generate_world.py      # Tạo thế giới Gazebo từ scene.yaml
│   ├── launch.sh              # Khởi động Gazebo, các controller và MoveIt
│   ├── run.sh                 # Chạy kiểm tra hoặc thực hiện tác vụ ngôn ngữ tự nhiên
│   └── record_demo.py         # Ghi hình camera và tiến trình thực hiện tác vụ thành MP4
├── src/ur3_llm_control/
│   ├── config/                # Cấu hình scene, bài tập sinh viên và controller
│   ├── launch/                # File launch cho mô phỏng ROS 2
│   ├── prompts/               # Các hướng dẫn gửi cho LLM để lập kế hoạch
│   ├── urdf/                  # Mô hình UR3e và đầu hút chân không
│   ├── worlds/                # Thế giới Gazebo
│   ├── src/                  # Plugin Gazebo cho thao tác gắp/thả
│   └── ur3_llm_control/       # Planner, validator, executor và các kỹ năng robot
└── tests/                     # Unit test cho validator và các plan mẫu
```

## Clone repository

```bash
git clone https://github.com/VietAnh2904/HRI2.git ~/HRI2
cd ~/HRI2
```

## Yêu cầu hệ thống

Sử dụng **Ubuntu 22.04** với các thành phần sau:

* ROS 2 Humble
* Gazebo Classic
* MoveIt 2

Các dependency của ROS được khai báo trong package manifest. Cài đặt bằng `rosdep` và cài thêm các công cụ cần thiết cho việc ghi hình bằng `apt`:

```bash
sudo apt update
sudo apt install -y python3-rosdep python3-pil ffmpeg

sudo rosdep init
# Chạy lệnh này một lần duy nhất.
# Bỏ qua nếu rosdep đã được khởi tạo trước đó.

rosdep update

source /opt/ros/humble/setup.bash

cd ~/HRI2
rosdep install --from-paths src --ignore-src -r -y
```

## Build project

```bash
cd ~/HRI2

source /opt/ros/humble/setup.bash

python3 scripts/generate_world.py

colcon build --packages-select ur3_llm_control --executor sequential
```

## Kết nối 9Router

Mô phỏng có thể được kiểm tra mà **không cần LLM**.

Để chạy tác vụ bằng ngôn ngữ tự nhiên, trước tiên khởi động **9Router** trong cùng môi trường Ubuntu/WSL và kết nối một provider khả dụng trong dashboard tại:

`http://127.0.0.1:20128`

Sau đó lưu **9Router Default Key** vào máy. Key được `scripts/run.sh` đọc từ:

```text
.runtime/nine-api-key
```

Thư mục `.runtime/` đã được Git bỏ qua nên key sẽ không được đưa lên repository.

```bash
cd ~/HRI2

mkdir -p .runtime results

read -rs -p 'Paste the 9Router Default Key: ' ROUTER_KEY
echo

printf '%s' "$ROUTER_KEY" > .runtime/nine-api-key

chmod 600 .runtime/nine-api-key

unset ROUTER_KEY
```

Model đã được kiểm thử:

```text
oc/muse-spark-1.3-contributor-free
```

Nếu model trên không khả dụng, hãy chọn một model đang được bật trong 9Router và đặt biến môi trường `NINEROUTER_MODEL` thành **model ID** tương ứng trước khi chạy tác vụ.

## Chạy project

### Terminal 1 – Khởi động mô phỏng

Mở **Terminal 1**:

```bash
cd ~/HRI2

bash scripts/launch.sh gui:=true
```

Lệnh này sẽ khởi động Gazebo, controller và MoveIt.

### Terminal 2 – Kiểm tra hệ thống

Mở **Terminal 2**:

```bash
cd ~/HRI2

source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p results

bash scripts/run.sh --inspect --result results/inspect.json
```

Tiếp tục khi output kết thúc bằng:

```text
TASK SUCCESS
```

### Chạy tác vụ bằng ngôn ngữ tự nhiên

Sau khi mô phỏng đã sẵn sàng, chạy:

```bash
cd ~/HRI2

source /opt/ros/humble/setup.bash
source install/setup.bash

export NINEROUTER_MODEL='oc/muse-spark-1.3-contributor-free'

mkdir -p results

set -o pipefail

bash scripts/run.sh \
  --command 'Move the blue cube to zone A, the red cube to zone B, and the yellow cube to zone C, then return the robot home.' \
  --result results/demo.json \
  2>&1 | tee results/demo.log
```

Trong ví dụ trên, robot thực hiện yêu cầu:

> Đưa khối lập phương màu xanh dương đến Zone A, khối màu đỏ đến Zone B và khối màu vàng đến Zone C, sau đó đưa robot về vị trí Home.

Nếu chạy thành công, terminal sẽ hiển thị:

```text
TASK SUCCESS
```

Đồng thời hệ thống sẽ tạo file JSON chứa:

* Chế độ chạy (`llm`)
* Kế hoạch được LLM sinh ra
* Các bước thực thi
* Vị trí cuối cùng của các vật thể

Các file sinh ra trong thư mục `results/` chỉ dùng cho local và đã được loại khỏi Git.

## Ghi video demo (tùy chọn)

Mở **Terminal 3** và khởi động chương trình ghi hình **trước khi chạy tác vụ**.

```bash
cd ~/HRI2

source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p results

python3 scripts/record_demo.py \
  --output results/demo.mp4 \
  --log results/demo.log \
  --stop-file /tmp/ur3e_demo.stop \
  --fps 6
```

Chờ terminal hiển thị:

```text
RECORDER_READY
```

Sau đó chạy tác vụ ở **Terminal 2**.

Khi terminal chạy tác vụ hiển thị:

```text
TASK SUCCESS
```

dừng recorder bằng:

```bash
touch /tmp/ur3e_demo.stop
```

Chờ chương trình ghi hình hoàn tất. Các file được tạo gồm:

```text
results/demo.mp4
results/demo.capture.json
```

## Chạy các bài kiểm thử Validator

Để chạy toàn bộ unit test của validator:

```bash
cd ~/HRI2

source /opt/ros/humble/setup.bash

PYTHONPATH=src/ur3_llm_control \
python3 -m unittest discover -s tests -v
```

Nếu các test đều chạy thành công, kết quả sẽ được hiển thị trực tiếp trong terminal.
