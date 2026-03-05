import os
import json
import numpy as np
from pathlib import Path

import torch
from torch.utils.data import IterableDataset, DataLoader
import torchvision.transforms.functional as F
from huggingface_hub import snapshot_download
from transformers import AutoProcessor
from tqdm import tqdm

from transformers import Qwen3VLProcessor


ACTION_DIM = 16

# ARHA 16-DOF joint index mapping
IDX = {
    "left_arm":      slice(0, 6),
    "right_arm":     slice(6, 12),
    "left_gripper":  12,
    "right_gripper": 13,
    "head_pan":      14,
    "head_tilt":     15,
}

# ALOHA 14-DOF joint index mapping
ALOHA_IDX = {
    "left_arm":      slice(0, 6),
    "left_gripper":  6,
    "right_arm":     slice(7, 13),
    "right_gripper": 13,
}

MODEL_ID = "Qwen/Qwen3.5-2B"

PROCESSOR = Qwen3VLProcessor.from_pretrained(MODEL_ID, trust_remote_code=True)

ALOHA_DATASETS = [
    "lerobot/aloha_static_cups_open",
    "lerobot/aloha_static_towel",
    "lerobot/aloha_mobile_cabinet",
    "lerobot/aloha_mobile_elevator",
    "lerobot/aloha_mobile_wash_pan",
    "lerobot/aloha_mobile_wipe_wine",
]

CAM_HIGH       = "observation.images.cam_high"
CAM_LEFT_WRIST = "observation.images.cam_left_wrist"

DATA_ROOT = Path("/home/grasp/Documents/ARHA/arha_vla/data")


def download_dataset(dataset_id: str, data_root: Path = DATA_ROOT, include_videos: bool = True) -> Path:
    local_dir = data_root / dataset_id.replace("/", os.sep)
    
    has_parquet = any((local_dir / "data").rglob("*.parquet")) if (local_dir / "data").exists() else False
    has_videos = any(local_dir.rglob("*.mp4")) if local_dir.exists() else False
    
    # skip download if data already exists locally
    if local_dir.exists() and has_parquet and (has_videos or not include_videos):
        return local_dir

    print(f"[Download] Downloading {dataset_id} -> {local_dir}")
    local_dir.mkdir(parents=True, exist_ok=True)
    ignore = [] if include_videos else ["videos/*", "*.mp4", "*.mkv"]
    snapshot_download(repo_id=dataset_id, repo_type="dataset", local_dir=str(local_dir), ignore_patterns=ignore)
    return local_dir


def ensure_datasets_downloaded(dataset_ids: list, data_root: Path = DATA_ROOT, include_videos: bool = True) -> dict:
    return {d: download_dataset(d, data_root, include_videos) for d in dataset_ids}


class AlohaStreamDataset(IterableDataset):
    def __init__(self, dataset_ids: list = None, max_samples: int = 5_000_000, 
                 data_root: Path = DATA_ROOT, download: bool = True, include_videos: bool = True):
        self.dataset_ids    = dataset_ids or ALOHA_DATASETS
        self.max_samples    = max_samples
        self.data_root      = Path(data_root)
        self.include_videos = include_videos

        if download:
            self.local_paths = ensure_datasets_downloaded(self.dataset_ids, self.data_root, include_videos)
        else:
            self.local_paths = {d: self.data_root / d.replace("/", os.sep) for d in self.dataset_ids}

    def _map_action(self, raw: np.ndarray) -> np.ndarray:
        # map ALOHA 14-DOF to ARHA 16-DOF, head joints zeroed
        action = np.zeros(ACTION_DIM, dtype=np.float32)
        action[IDX["left_arm"]]      = raw[ALOHA_IDX["left_arm"]]
        action[IDX["right_arm"]]     = raw[ALOHA_IDX["right_arm"]]
        action[IDX["left_gripper"]]  = raw[ALOHA_IDX["left_gripper"]]
        action[IDX["right_gripper"]] = raw[ALOHA_IDX["right_gripper"]]
        action[IDX["head_pan"]]      = 0.0
        action[IDX["head_tilt"]]     = 0.0
        return action

    def _parse_lerobot(self, sample: dict) -> dict | None:
        try:
            raw_action = sample.get("action")
            raw_state = sample.get("observation.state")
            
            if raw_action is None or raw_state is None: 
                return None

            raw_action = raw_action.numpy().astype(np.float32)
            raw_state = raw_state.numpy().astype(np.float32)

            # ALOHA data must be 14-DOF
            if len(raw_action) != 14 or len(raw_state) != 14: 
                return None

            action = self._map_action(raw_action)
            state = self._map_action(raw_state)

            result = {
                "state":       torch.tensor(state, dtype=torch.float32),
                "action":      torch.tensor(action, dtype=torch.float32),
                "instruction": "bimanual manipulation task",
            }
            if self.include_videos:
                result["image_head"]  = sample[CAM_HIGH]
            return result
        except Exception:
            return None

    def __iter__(self):
        import random
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        count = 0
        skipped = 0
        chunk_size = 8

        # build episode-safe chunk indices from metadata, no video decoding
        all_sources = []
        for dataset_id in self.dataset_ids:
            ds = LeRobotDataset(
                repo_id=dataset_id, 
                root=str(self.local_paths[dataset_id]), 
                video_backend='pyav', 
                download_videos=False
            )
            # get episode boundaries from parquet metadata
            for ep_idx in range(len(ds.meta.episodes)):
                ep = ds.meta.episodes[ep_idx]
                from_idx = ep["dataset_from_index"]
                to_idx = ep["dataset_to_index"]
                for start in range(from_idx, to_idx - chunk_size + 1):
                    all_sources.append((ds, start))

        print(f"[Dataset] Built {len(all_sources)} valid chunks across {len(self.dataset_ids)} datasets")

        # interleave across all datasets
        random.shuffle(all_sources)

        for ds, i in all_sources:
            if count >= self.max_samples: return
            
            try:
                samples = [ds[j] for j in range(i, i + chunk_size)]
                current = samples[0]
                
                state = self._map_action(current["observation.state"].numpy())
                actions = np.stack([self._map_action(s["action"].numpy()) for s in samples])

                yield {
                    "state":       torch.tensor(state, dtype=torch.float32),
                    "action":      torch.tensor(actions, dtype=torch.float32),
                    "image_head":  current[CAM_HIGH],
                    "instruction": "bimanual manipulation task",
                }
                count += 1
            except Exception as e:
                skipped += 1
                if skipped % 100 == 0:
                    print(f"[Dataset] Skipped {skipped} bad samples (last: {e})")
                continue

    def __len__(self):
        return self.max_samples

    @staticmethod
    def collate_fn(batch: list) -> dict:
        states = torch.stack([s["state"] for s in batch])
        actions = torch.stack([s["action"] for s in batch])
        images = [F.to_pil_image(s["image_head"]) for s in batch]
        
        # build chat-style messages for the VLM processor
        messages_list = [
            [{
                "role": "user",
                "content": [
                    {"type": "image"},
                    {"type": "text", "text": f"Task: {s['instruction']}. Predict the next 16-DOF robot action."}
                ]
            }]
            for s in batch
        ]
        
        texts = [PROCESSOR.apply_chat_template(msg, tokenize=False, add_generation_prompt=True) for msg in messages_list]
        
        inputs = PROCESSOR(
            text=texts,
            images=images,
            padding=True,
            return_tensors="pt"
        )
        
        inputs["state"] = states
        inputs["action"] = actions
        
        return inputs
