---
name: nerfstudio-trainer
description: Launch nerfstudio splatfacto (3D Gaussian Splatting) training with project-specific parameters. Use when the user asks to start/launch/run 3DGS training, retrain with new parameters, or resume training. Handles data path resolution, parameter configuration, background execution, and log file management.
---

# Nerfstudio Trainer

## Overview

This skill launches nerfstudio splatfacto (3D Gaussian Splatting) training with parameters tuned for LiDAR-camera fusion datasets. It manages the full launch workflow: parameter configuration, background execution, and log file setup.

## When to Use This Skill

Use this skill when the user requests to:
- "学習を開始して" / "trainを走らせて"
- "新しいパラメータで再学習"
- "splatfactoを起動"
- "3DGS training を開始"
- "v{N}で学習開始"

## How to Use This Skill

### Step 1: Determine Parameters

Ask or infer the following from user context:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `data_dir` | (required) | COLMAP sparse model directory (containing `cameras.txt`, `images.txt`, `points3D.txt`) |
| `output_dir` | `{data_dir}/../outputs` | Training output directory |
| `log_version` | auto-increment | Log file version suffix (e.g., `train_lidar_v8.log`) |
| `reset_alpha_every` | `200` | Alpha reset interval (critical for large datasets) |
| `densify_grad_thresh` | `0.0002` | Gradient threshold for densification |
| `camera_optimizer` | `SO3xR3` | Camera pose optimization mode |
| `max_num_iterations` | `30000` | Total training steps |
| `load_3d_points` | `True` | Initialize Gaussians from points3D.txt |

### Step 2: Determine Log Version

Check existing log files to auto-increment:

```bash
ls {project_dir}/train_lidar_v*.log 2>/dev/null | sort -V | tail -1
```

### Step 3: Build and Execute Command

#### Base Command Template

```bash
ns-train splatfacto \
  --data {data_dir} \
  --output-dir {output_dir} \
  --max-num-iterations {max_num_iterations} \
  --pipeline.model.camera-optimizer.mode {camera_optimizer} \
  --pipeline.model.reset-alpha-every {reset_alpha_every} \
  --pipeline.model.densify-grad-thresh {densify_grad_thresh} \
  --pipeline.datamanager.dataparser.orientation-method none \
  --pipeline.datamanager.dataparser.center-method poses \
  --pipeline.datamanager.dataparser.auto-scale-poses False \
  --pipeline.datamanager.dataparser.assume-colmap-world-coordinate-convention False \
  --pipeline.datamanager.dataparser.load-3D-points {load_3d_points} \
  > {project_dir}/train_lidar_v{N}.log 2>&1 &
```

#### Important Notes on Parameters

**Densification fix for large datasets (>3000 images):**
- nerfstudio has a known issue (GitHub #2927, #3743) where `pause_refine_after_reset = num_train_data + refine_every`
- Default `reset_alpha_every=30` → `reset_every=3000`, which is smaller than `pause_refine_after_reset` for large datasets
- **Must use `reset_alpha_every=200`** (or higher) so that `reset_every=20000 > pause_refine_after_reset`
- Without this fix, Gaussian count stays constant (no densification)

**LiDAR-based datasets:**
- `orientation-method none`: LiDAR world coordinates are already aligned
- `center-method poses`: Center scene at camera pose centroid
- `auto-scale-poses False`: Preserve real-world scale from LiDAR SLAM
- `assume-colmap-world-coordinate-convention False`: Using LiDAR world frame, not COLMAP convention

### Step 4: Verify Launch

After launching:
1. Confirm PID is running: `ps aux | grep ns-train`
2. Check initial log output: `tail -5 {log_file}`
3. Report the viewer URL (typically `http://localhost:7007`)
4. Report the output directory path

### Step 5: Report to User

Provide:
- PID
- Log file path
- Output directory
- Viewer URL
- Key parameters used
- Expected densification window (when Gaussians should start growing)

## Parameter Presets

### Default (LiDAR + large dataset)
```
reset_alpha_every=200
densify_grad_thresh=0.0002
camera_optimizer=SO3xR3
max_num_iterations=30000
```

### Fast test (quick validation)
```
reset_alpha_every=200
densify_grad_thresh=0.0002
camera_optimizer=SO3xR3
max_num_iterations=5000
```

### High quality (longer training)
```
reset_alpha_every=200
densify_grad_thresh=0.0001
camera_optimizer=SO3xR3
max_num_iterations=50000
```

## Troubleshooting

**NaN error at startup:**
- Check `points3D.txt` for NaN/Inf values
- Filter with: `np.isfinite(xyz).all(axis=1)`

**Gaussian count not growing:**
- Verify `reset_alpha_every` is large enough: `reset_alpha_every × refine_every > num_train_data + refine_every`
- Check if training step has passed `pause_refine_after_reset`

**CUDA out of memory:**
- Reduce initial point count in `points3D.txt`
- Lower `max-num-iterations` for testing
