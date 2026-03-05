import os
import json
import torch
import numpy as np
import yaml
from pathlib import Path
from torch.utils.data import DataLoader
from torch.optim import AdamW
from torch.optim.lr_scheduler import CosineAnnealingLR
from tqdm import tqdm

from dataset import AlohaStreamDataset, ACTION_DIM
from model import ArhaQwenVLA, loss_function

os.environ["TOKENIZERS_PARALLELISM"] = "false"

def load_config(path: str) -> dict:
    if not os.path.exists(path):
        # fallback defaults if yaml not found
        return {
            "model_id": "Qwen/Qwen3.5-2B",
            "max_samples": 200000,
            "data_root": "/home/grasp/Documents/ARHA/arha_vla/data",
            "batch_size": 2,
            "grad_accum": 16,
            "chunk_size": 8,
            "load_in_4bit": True,
            "lr": 1e-4,
            "weight_decay": 0.01,
            "num_epochs": 3,
            "max_grad_norm": 1.0,
            "log_every": 10,
            "save_every": 500,
            "early_stop_patience": 10,
            "early_stop_window": 50,
            "early_stop_min_steps": 500,
            "checkpoint_dir": "/home/grasp/Documents/ARHA/arha_vla/checkpoints/stage1",
            "log_path": "/home/grasp/Documents/ARHA/arha_vla/logs/train_stage1.json"
        }
    with open(path) as f:
        return yaml.safe_load(f)
    
CFG = load_config("config/train_stage1.yaml")

class Logger:
    def __init__(self, path: str):
        self.path = path
        self.logs = {"steps": [], "train_loss": [], "lr": []}
        Path(path).parent.mkdir(parents=True, exist_ok=True)

    def log(self, step: int, metrics: dict):
        self.logs["steps"].append(step)
        for k, v in metrics.items():
            if k in self.logs: self.logs[k].append(v)

    def save(self):
        with open(self.path, "w") as f:
            json.dump(self.logs, f, indent=2)

def save_checkpoint(model, step, loss, cfg):
    ckpt_dir = Path(cfg["checkpoint_dir"])
    ckpt_dir.mkdir(parents=True, exist_ok=True)
    path = ckpt_dir / f"step_{step:06d}_loss_{loss:.4f}.pt"
    
    # only save lora weights, state encoder, action head
    lora_state = {k: v for k, v in model.backbone.named_parameters() if "lora" in k}
    
    torch.save({
        "step": step,
        "loss": loss,
        "state_encoder": model.state_encoder.state_dict(),
        "action_head": model.action_head.state_dict(),
        "lora_weights": lora_state,
        "cfg": cfg,
    }, path)
    print(f"[Checkpoint] Saved: {path}")

def train(cfg: dict):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    train_ds = AlohaStreamDataset(max_samples=cfg["max_samples"], download=False)
    train_loader = DataLoader(
        train_ds, batch_size=cfg["batch_size"], 
        collate_fn=AlohaStreamDataset.collate_fn, num_workers=2,
        pin_memory=True
    )
    
    model = ArhaQwenVLA(model_id=cfg["model_id"], load_in_4bit=cfg["load_in_4bit"]).to(device)
    
    trainable_params = [p for p in model.parameters() if p.requires_grad]
    optimizer = AdamW(trainable_params, lr=cfg["lr"], weight_decay=cfg["weight_decay"])
    
    steps_per_epoch = cfg["max_samples"] // (cfg["batch_size"] * cfg["grad_accum"])
    total_steps = cfg["num_epochs"] * steps_per_epoch
    scheduler = CosineAnnealingLR(optimizer, T_max=total_steps, eta_min=cfg["lr"] * 0.1)

    logger = Logger(cfg["log_path"])
    scaler = torch.amp.GradScaler('cuda')

    # early stopping config
    es_window = cfg.get("early_stop_window", 50)
    es_patience = cfg.get("early_stop_patience", 10)
    es_min_steps = cfg.get("early_stop_min_steps", 2000)
    loss_history = []
    es_no_improve = 0
    best_avg_loss = float('inf')

    global_step = 0
    early_stopped = False
    for epoch in range(cfg["num_epochs"]):
        if early_stopped:
            break
        model.train()
        optimizer.zero_grad()
        accum_loss = 0.0
        pbar = tqdm(train_loader, desc=f"Epoch {epoch+1}")

        for batch_idx, batch in enumerate(pbar):
            # forward pass with bf16 autocast
            with torch.amp.autocast('cuda', dtype=torch.bfloat16):
                pred = model(
                    input_ids=batch["input_ids"].to(device),
                    attention_mask=batch["attention_mask"].to(device),
                    pixel_values=batch["pixel_values"].to(device),
                    state=batch["state"].to(device).float()
                )
                loss = loss_function(pred, batch["action"].to(device))
                loss = loss / cfg["grad_accum"]

            scaler.scale(loss).backward()
            accum_loss += loss.item()

            # gradient accumulation step
            if (batch_idx + 1) % cfg["grad_accum"] == 0:
                scaler.unscale_(optimizer)
                torch.nn.utils.clip_grad_norm_(trainable_params, cfg["max_grad_norm"])
                scaler.step(optimizer)
                scaler.update()
                scheduler.step()
                optimizer.zero_grad()
                global_step += 1

                # avg loss across all micro-batches in this window
                avg_loss = accum_loss
                accum_loss = 0.0

                # log to disk for live_plot
                if global_step % cfg["log_every"] == 0:
                    current_lr = scheduler.get_last_lr()[0]
                    pbar.set_postfix({"loss": f"{avg_loss:.2e}", "step": global_step})
                    logger.log(global_step, {"train_loss": avg_loss, "lr": current_lr})
                    logger.save()
                
                if global_step % cfg["save_every"] == 0:
                    save_checkpoint(model, global_step, avg_loss, cfg)

                # early stopping using median to ignore spikes
                if global_step % cfg["log_every"] == 0:
                    loss_history.append(avg_loss)
                    if global_step >= es_min_steps and len(loss_history) >= es_window * 2:
                        old_median = float(np.median(loss_history[-es_window*2:-es_window]))
                        new_median = float(np.median(loss_history[-es_window:]))
                        # require 0.1% improvement
                        if new_median < best_avg_loss * 0.999:
                            best_avg_loss = new_median
                            es_no_improve = 0
                        else:
                            es_no_improve += 1
                        if es_no_improve >= es_patience:
                            print(f"\n[Early Stop] No improvement for {es_patience} checks "
                                  f"(after step {global_step}). "
                                  f"Best median loss: {best_avg_loss:.4e}, Current: {new_median:.4e}")
                            save_checkpoint(model, global_step, new_median, cfg)
                            early_stopped = True
                            break

    logger.save()
    print("Training Complete!")

if __name__ == "__main__":
    train(CFG)