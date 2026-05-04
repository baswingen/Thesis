import torch
import torch.nn as nn

class DummyModel(nn.Module):
    def __init__(self, use_checkpointing=True):
        super().__init__()
        self.use_checkpointing = use_checkpointing
        self.spatial_pos_embedding = nn.Parameter(torch.randn(1, 1, 2, 8))
        self.spatial_transformer = nn.TransformerEncoder(
            nn.TransformerEncoderLayer(d_model=8, nhead=2, batch_first=True),
            num_layers=1
        )
        self.fc = nn.Linear(8, 1)

    def forward(self, x):
        # x: [B, T, C, d_model]
        x_spatial = x + self.spatial_pos_embedding
        B, T, C, d = x_spatial.shape
        x_spatial = x_spatial.view(B * T, C, d)

        if self.use_checkpointing and self.training:
            def create_custom_forward(module):
                def custom_forward(*inputs):
                    return module(*inputs)
                return custom_forward

            x_spatial_out = torch.utils.checkpoint.checkpoint(
                create_custom_forward(self.spatial_transformer),
                x_spatial,
                use_reentrant=False
            )
        else:
            x_spatial_out = self.spatial_transformer(x_spatial)

        return self.fc(x_spatial_out.mean(dim=1)).sum()

model = DummyModel(use_checkpointing=True)
x = torch.randn(2, 3, 2, 8, requires_grad=False)
loss = model(x)
loss.backward()

print("spatial_pos_embedding grad:", model.spatial_pos_embedding.grad is not None)
for name, param in model.spatial_transformer.named_parameters():
    print(f"{name} grad:", param.grad is not None)
