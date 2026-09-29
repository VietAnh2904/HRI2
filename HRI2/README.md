# UR3e LLM Skill Planning

ROS 2 Humble simulation of a UR3e robot that receives a natural-language task, asks an OpenAI-compatible LLM for a skill plan, validates the plan, then executes it through MoveIt 2 and Gazebo Classic.

- ROS package: `ur3_llm_control`
- Student assignment configured in `src/ur3_llm_control/config/student_config.yaml`
- For MSSV `23020722`: Zone A ← blue, Zone B ← red, Zone C ← yellow
- [Demo video](https://drive.google.com/file/d/1_IpHvAtT0lvKuvCsBJHan8WBQUIoCb5-/view?usp=sharing) · [GitHub repository](https://github.com/VietAnh2904/HRI2)

## Repository layout

```text
HRI2/
├── README.md
├── scripts/
│   ├── generate_world.py      # Generate the Gazebo world from scene.yaml
│   ├── launch.sh              # Start Gazebo, controllers and MoveIt
│   ├── run.sh                 # Run inspection or a natural-language task
│   └── record_demo.py         # Record the camera and task progress to MP4
├── src/ur3_llm_control/
│   ├── config/                # Scene, student assignment and controllers
│   ├── launch/                # ROS 2 simulation launch file
│   ├── prompts/               # LLM planning instructions
│   ├── urdf/                  # UR3e and suction tool model
│   ├── worlds/                # Gazebo world
│   ├── src/                   # Gazebo grasp/release plugin
│   └── ur3_llm_control/       # Planner, validator, executor and robot skills
└── tests/                     # Validator unit tests and sample plans
```

## Clone

```bash
git clone https://github.com/VietAnh2904/HRI2.git ~/HRI2
cd ~/HRI2
```

## Requirements

Use Ubuntu 22.04 with ROS 2 Humble, Gazebo Classic and MoveIt 2 installed. The package manifest lists ROS dependencies. Install them with `rosdep`; install the extra tools used for recording with `apt`:

```bash
sudo apt update
sudo apt install -y python3-rosdep python3-pil ffmpeg
sudo rosdep init  # Run once only; skip if rosdep is already initialized
rosdep update
source /opt/ros/humble/setup.bash
cd ~/HRI2
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
cd ~/HRI2
source /opt/ros/humble/setup.bash
python3 scripts/generate_world.py
colcon build --packages-select ur3_llm_control --executor sequential
```

## Connect 9Router

The simulator can be inspected without an LLM. To run a language command, first start 9Router in the same Ubuntu/WSL environment and connect an available provider in its dashboard at `http://127.0.0.1:20128`.

Save the 9Router **Default Key** locally. The key is read by `scripts/run.sh` from `.runtime/nine-api-key`; `.runtime/` is ignored by Git.

```bash
cd ~/HRI2
mkdir -p .runtime results
read -rs -p 'Paste the 9Router Default Key: ' ROUTER_KEY
echo
printf '%s' "$ROUTER_KEY" > .runtime/nine-api-key
chmod 600 .runtime/nine-api-key
unset ROUTER_KEY
```

The verified model is `oc/muse-spark-1.3-contributor-free`. If it is unavailable, select a model enabled in 9Router and set `NINEROUTER_MODEL` to its model ID in the terminal before running the task.

## Run the project

Open **Terminal 1** and start the simulator:

```bash
cd ~/HRI2
bash scripts/launch.sh gui:=true
```

Open **Terminal 2** and check that ROS, controllers and the scene are ready:

```bash
cd ~/HRI2
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p results
bash scripts/run.sh --inspect --result results/inspect.json
```

Continue when the output ends with `TASK SUCCESS`. To run the natural-language demonstration:

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

A successful run prints `TASK SUCCESS` and writes a JSON result with mode `llm`, the generated plan, execution steps and final object positions. Output files under `results/` are local and excluded from Git.

## Record a video (optional)

Start the recorder in **Terminal 3 before** running the task. It records the live Gazebo camera and reads the task log from Terminal 2:

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

Wait for `RECORDER_READY`, run the task in Terminal 2, then stop the recorder after `TASK SUCCESS`:

```bash
touch /tmp/ur3e_demo.stop
```

Wait for the recorder to finish writing `demo.mp4` and `demo.capture.json`.

## Run validator tests

```bash
cd ~/HRI2
source /opt/ros/humble/setup.bash
PYTHONPATH=src/ur3_llm_control python3 -m unittest discover -s tests -v
```
