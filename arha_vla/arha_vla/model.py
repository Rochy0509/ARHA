import torch
import torch.nn as nn
from transformers import AutoModelForCausalLM, BitsAndBytesConfig
from peft import get_peft_model, LoraConfig, TaskType, prepare_model_for_kbit_training

MODEL_ID = "Qwen/Qwen3.5-2B"
HIDDEN_DIM = 2048
STATE_DIM = 16
STATE_EMBED_DIM = 128
ACTION_DIM = 16
CHUNK_SIZE = 8

def get_bnb_config() -> BitsAndBytesConfig:
    return BitsAndBytesConfig(
        load_in_4bit=True,
        bnb_4bit_use_double_quant=True,
        bnb_4bit_quant_type="nf4",
        bnb_4bit_compute_dtype=torch.bfloat16
    )

def get_lora_config() -> LoraConfig:
    return LoraConfig(
        r=16,
        lora_alpha=32,
        lora_dropout=0.05,
        bias="none",
        target_modules=["q_proj", "k_proj", "v_proj", "o_proj", "gate_proj", "up_proj", "down_proj"],
        task_type=TaskType.CAUSAL_LM
    )

class ActionHead(nn.Module):
    def __init__(self, input_dim: int = HIDDEN_DIM + STATE_EMBED_DIM, hidden_dim: int = 512,
                 action_dim: int = ACTION_DIM, chunk_size: int = CHUNK_SIZE, dropout: float = 0.1):
        super().__init__()
        self.chunk_size = chunk_size
        self.action_dim = action_dim
        output_dim = chunk_size * action_dim

        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.LayerNorm(hidden_dim // 2),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim // 2, output_dim),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out = self.net(x)
        return out.view(-1, self.chunk_size, self.action_dim)

class StateEncoder(nn.Module):
    def __init__(self, state_dim: int = STATE_DIM, embed_dim: int = STATE_EMBED_DIM):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, embed_dim),
            nn.LayerNorm(embed_dim),
            nn.GELU(),
            nn.Linear(embed_dim, embed_dim),
            nn.LayerNorm(embed_dim),
        )

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        return self.net(state)

class ArhaQwenVLA(nn.Module):
    def __init__(self, model_id: str = MODEL_ID, chunk_size: int = CHUNK_SIZE, load_in_4bit: bool = True):
        super().__init__()
        self.chunk_size = chunk_size
        bnb_config = get_bnb_config() if load_in_4bit else None

        self.backbone = AutoModelForCausalLM.from_pretrained(
            model_id,
            quantization_config=bnb_config,
            device_map="auto",
            dtype=torch.bfloat16,
            trust_remote_code=True
        )

        if load_in_4bit:
            self.backbone = prepare_model_for_kbit_training(
                self.backbone,
                use_gradient_checkpointing=True
            )

        lora_config = get_lora_config()
        self.backbone = get_peft_model(self.backbone, lora_config)
        self.backbone.print_trainable_parameters()

        self.device = next(self.backbone.parameters()).device
        self.state_encoder = StateEncoder(STATE_DIM, STATE_EMBED_DIM).to(self.device)
        self.action_head = ActionHead(HIDDEN_DIM + STATE_EMBED_DIM, 512, ACTION_DIM, chunk_size).to(self.device)
        
    def forward(self, input_ids, attention_mask, pixel_values, state, image_grid_thw=None, **kwargs):
        outputs = self.backbone(
            input_ids=input_ids,
            attention_mask=attention_mask,
            pixel_values=pixel_values,
            image_grid_thw=image_grid_thw,
            output_hidden_states=True,
            return_dict=True,
        )

        # take last token hidden state as scene representation
        last_hidden = outputs.hidden_states[-1]
        scene_embed = last_hidden[:, -1, :].float()
        
        # fuse scene + state then predict action chunk
        state_embed = self.state_encoder(state.to(self.device).float())
        fused = torch.cat([scene_embed, state_embed], dim=-1)
        actions = self.action_head(fused)

        return actions

# MSE loss for continuous action prediction
def loss_function(pred: torch.Tensor, 
                  target: torch.Tensor, 
                  reduction: str = 'mean') -> torch.Tensor:
    return torch.nn.functional.mse_loss(pred, target, reduction=reduction)