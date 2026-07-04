# Remote GraspNet Baseline 6D Inference

This setup keeps ROS Noetic, MoveIt, the arm driver, and tactile control in the Ubuntu VM, while WSL2 on Windows runs GPU inference.

## WSL2 GPU Side

Start the server from the same Robot-Controller checkout that contains `tools/graspnet_baseline_server.py`:

```bash
conda activate grasp6d118
cd ~/grasp6d_ws/Robot-Controller

python tools/graspnet_baseline_server.py \
  --baseline-root /home/lv/grasp6d_ws/graspnet-baseline \
  --checkpoint /home/lv/grasp6d_ws/checkpoints/checkpoint-rs.tar \
  --host 0.0.0.0 \
  --port 8000 \
  --device cuda:0 \
  --warmup
```

For network-only testing without loading the model:

```bash
python tools/graspnet_baseline_server.py --host 0.0.0.0 --port 8000 --mock
```

Check the service inside WSL2:

```bash
curl http://127.0.0.1:8000/health
```

## Windows To Ubuntu VM Network

First try the WSL2 address:

```bash
hostname -I
```

From the Ubuntu VM:

```bash
curl http://<WSL2_IP>:8000/health
```

If the VM cannot reach WSL2 directly, expose the port through Windows PowerShell as Administrator:

```powershell
netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=8000 connectaddress=<WSL2_IP> connectport=8000
netsh advfirewall firewall add rule name="Alicia Grasp6D 8000" dir=in action=allow protocol=TCP localport=8000
```

Then use the Windows host IP from the Ubuntu VM:

```bash
curl http://<WINDOWS_HOST_IP>:8000/health
```

## Ubuntu VM ROS Side

The remote node is the default 6D path. Start the full system with the WSL2/Windows URL:

```bash
source devel/setup.bash
roslaunch alicia_flexible_grasp_supervisor full_system.launch \
  start_real_arm:=false \
  start_camera:=true \
  start_tactile:=true \
  start_gui:=true \
  use_remote_grasp6d:=true \
  remote_grasp6d_url:=http://<WINDOWS_HOST_IP_OR_WSL2_IP>:8000
```

To fall back to the older local `alicia_d_grasp_6d` backend:

```bash
roslaunch alicia_flexible_grasp_supervisor full_system.launch use_remote_grasp6d:=false
```

The ROS side publishes the returned 6D grasp sequence on `/grasp_6d/plan`, which is consumed by `grasp_task_node.py`.
