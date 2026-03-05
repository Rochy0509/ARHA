# ARHA VLA - Vision-Language-Action Model

> **Warning:** This project is under active development. Code, configs, and training procedures may change without notice.

ARHA VLA is the vision-language-action training pipeline for the ARHA bimanual robot. It fine-tunes a quantized Qwen 2B language model with LoRA adapters to predict 16-DOF robot actions from camera observations, joint states, and language instructions.

The model takes a camera image + current joint state as input and outputs an 8-step action chunk (8 future timesteps x 16 joints) for smooth trajectory execution.

---

## Project Structure

```
arha_vla/
├── arha_vla/              # core training code
│   ├── dataset.py         # data loading, ALOHA-to-ARHA joint mapping, collation
│   ├── model.py           # VLA model architecture (Qwen backbone + LoRA + action head)
│   └── train.py           # training loop, logging, checkpointing, early stopping
├── config/
│   └── train_stage1.yaml  # training hyperparameters and paths
├── scripts/
│   └── live_plot.py       # real-time loss curve visualization during training
├── data/                  # downloaded LeRobot ALOHA datasets (not tracked in git)
├── checkpoints/           # saved model weights per training stage
├── logs/                  # training metrics in JSON format
└── README.md
```

## How It Works

### Dataset (`dataset.py`)

Loads demonstration data from 6 LeRobot ALOHA datasets (bimanual manipulation tasks like opening cups, wiping, washing). Each sample contains a camera frame, 14-DOF ALOHA joint states, and actions.

The data pipeline:
- Maps ALOHA 14-DOF joint vectors to ARHA 16-DOF format (head pan/tilt joints are zeroed)
- Builds episode-safe 8-frame action chunks using parquet metadata (no video decoding at index time)
- Shuffles across all datasets so training sees interleaved tasks, not sequential blocks
- Formats each sample as a VLM chat message with image + text instruction for the Qwen processor

### Model (`model.py`)

The `ArhaQwenVLA` model has three components:

1. **Backbone** - Qwen 2B language model, 4-bit quantized (NF4), with LoRA adapters on all attention and MLP layers. Only LoRA weights are trained (~0.6% of total parameters).
2. **State Encoder** - small MLP that encodes the 16-DOF joint state into a 128-dim embedding.
3. **Action Head** - MLP that takes the fused scene embedding (last hidden state from backbone) + state embedding and predicts an action chunk of shape [8, 16] (8 timesteps x 16 joints).

Loss is MSE between predicted and ground truth action chunks.

### Training (`train.py`)

The training loop handles:
- Mixed precision (bfloat16) with gradient scaling
- Gradient accumulation across multiple micro-batches
- Cosine annealing learning rate schedule
- Periodic checkpointing (saves LoRA weights, state encoder, action head)
- JSON logging to disk for live monitoring
- Early stopping based on median loss (robust to occasional spikes), with a configurable minimum step threshold

### Live Plot (`scripts/live_plot.py`)

Reads the training log JSON and displays a real-time loss curve (log scale) that refreshes every 15 seconds. Run it in a separate terminal alongside training.

## Training Stages

> **Note:** Only Stage 1 is currently implemented. Stage 2 (fine-tuning on ARHA-specific data) will follow once Stage 1 training completes and the robot collects its own demonstrations.

**Stage 1** - Pre-train on public ALOHA datasets to learn general bimanual manipulation behavior. Uses 200K samples across 6 datasets with interleaved shuffling.

**Stage 2** (planned) - Fine-tune the Stage 1 checkpoint on ARHA robot demonstrations collected in simulation or on hardware.

## Configuration

All training hyperparameters are in `config/train_stage1.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `batch_size` | 2 | samples per forward pass |
| `grad_accum` | 16 | accumulation steps (effective batch = 32) |
| `num_epochs` | 3 | training epochs |
| `lr` | 1e-4 | learning rate |
| `chunk_size` | 8 | action prediction horizon |
| `save_every` | 500 | checkpoint interval (steps) |
| `early_stop_min_steps` | 2000 | minimum steps before early stopping can trigger |

## Usage

### Train

```bash
conda activate arha_vla
cd arha_vla
python arha_vla/train.py
```

### Monitor

In a separate terminal:

```bash
conda activate arha_vla
cd arha_vla/scripts
python live_plot.py
```

## Requirements

- Python 3.10+
- PyTorch 2.x with CUDA
- transformers, peft, bitsandbytes
- lerobot
- NVIDIA GPU with 12GB+ VRAM (tested on RTX 3080 Ti)
