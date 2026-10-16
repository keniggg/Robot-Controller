# WSL GraspNet / MuJoCo unified startup

`tools/start_wsl_grasp_stack.py` preserves the operator's existing GraspNet
startup and adds the separately verified MuJoCo deployment. Both processes run
inside WSL. ROS uses port 8000 for inference and port 8001 for simulation.

After activating `grasp6d118`, run:

```bash
python tools/start_wsl_grasp_stack.py
```

The transferred standalone installation can instead be started from any directory:

```bash
conda activate grasp6d118 && \
python ~/.local/share/alicia-grasp-stack/start_wsl_grasp_stack.py
```

The default repository is `~/grasp6d_ws/Robot-Controller-v3`. Override it with
`--repo-root` or `ALICIA_WSL_REPO`. The existing `GRASPNET_BASELINE_ROOT`,
`GRASPNET_CHECKPOINT`, `GRASPNET_DEVICE`, and `MUJOCO_ALICIA_MODEL_XML`
environment variables remain supported, with the operator's original defaults.

Port 8000 runs the original `tools/start_mujoco_digital_twin_wsl.sh` with
`--pass-score 80 --min-lift-success-m 0.015 --max-snapshot-age-sec 120.0 --warmup`.
Its bundled simulation endpoint is not selected by ROS.

Port 8001 runs the verified deployment in
`~/.local/share/alicia-mujoco-20260923-6047f9a74046`. The launcher checks its
manifest, source fingerprint, MuJoCo version, initial IK tolerance, force and
lift contracts. Its health response intentionally has top-level `ok: false`
because GraspNet is not configured in that process; `digital_twin.ok: true`
is the relevant status. The original model environment variable for port 8000
does not replace the model in the isolated port 8001 deployment.

Existing healthy matching services are reused. An occupied port with mismatched
health causes an error and is not replaced. The launcher prints
`READY_WSL_GRASP_STACK` after both checks pass. Keep its terminal open. Ctrl-C
stops only services started by that launcher; a reused service stays owned by
its original terminal. After a full shutdown, one launcher can start both.
The launcher sends no robot commands and does not itself initiate a grasp.

Use `--check-only` to validate both services without starting processes.
Validation: six tests cover the simulation-only health contract, wrong versions
and physics settings, GraspNet warmup/checkpoint mismatch, service reuse, and
cleanup of owned process groups without stopping unrelated processes.

Migration verified: the operator installed the unified launcher and obtained
`READY_WSL_GRASP_STACK`. Both WSL services were then independently checked.
ROS runtime and the persistent session launch use WSL port 8001 for simulation.
The same historical replay took 1.6403 seconds on WSL versus 13.0615 seconds
locally, with an identical response. This compares only that offline replay.

The operator subsequently closed all hardware interfaces. A read-only check
found no surviving recorded-grasp or grasp-runner process; both WSL endpoints
were unreachable at that check. No hardware was restarted. Successful service
migration is not evidence of successful physical grasping.
