import torch
from torch import nn


__all__ = [
    "GAM_Attention",
]

# GAM Attention Start


def channel_shuffle(x, groups=2):
    """Shuffle channels between grouped-convolution outputs."""
    b, c, h, w = x.size()
    if groups < 1 or c % groups:
        raise ValueError(f"channels ({c}) must be divisible by shuffle groups ({groups})")
    out = x.reshape(b, groups, c // groups, h, w).permute(0, 2, 1, 3, 4).contiguous()
    out = out.reshape(b, c, h, w)
    return out


# Global Attention Mechanism
# https://arxiv.org/abs/2112.05561
class GAM_Attention(nn.Module):
    """Residual Global Attention Mechanism adapted for stable pretrained YOLO fine-tuning.

    The channel and spatial gates follow the channel-spatial GAM idea. A zero-initialized residual scale makes the
    module an exact identity mapping when it is first inserted into a pretrained network, allowing training to learn
    how much attention to apply without destroying the pretrained feature distribution.
    """

    def __init__(self, c1, c2, group=True, rate=4, shortcut=True):
        super().__init__()
        if c1 != c2:
            raise ValueError(f"GAM_Attention requires equal input/output channels, got c1={c1}, c2={c2}")
        if rate < 1 or c1 % rate or c2 % rate:
            raise ValueError(f"c1 ({c1}) and c2 ({c2}) must be divisible by rate ({rate})")
        hidden_channels = c1 // rate
        groups = rate if group else 1
        if hidden_channels % groups:
            raise ValueError(f"hidden channels ({hidden_channels}) must be divisible by groups ({groups})")
        self.shuffle_groups = groups
        self.shortcut = shortcut

        self.channel_attention = nn.Sequential(
            nn.Linear(c1, hidden_channels),
            nn.ReLU(inplace=True),
            nn.Linear(hidden_channels, c1),
        )

        self.spatial_reduce = nn.Sequential(
            nn.Conv2d(c1, hidden_channels, kernel_size=7, padding=3, groups=groups, bias=False),
            nn.BatchNorm2d(hidden_channels),
            nn.ReLU(inplace=True),
        )
        self.spatial_expand = nn.Sequential(
            nn.Conv2d(hidden_channels, c2, kernel_size=7, padding=3, groups=groups, bias=False),
            nn.BatchNorm2d(c2),
        )
        self.gamma = nn.Parameter(torch.zeros(1)) if shortcut else None

    def forward(self, x):
        b, c, h, w = x.shape
        x_permute = x.permute(0, 2, 3, 1).reshape(b, -1, c)
        channel_gate = self.channel_attention(x_permute).sigmoid().reshape(b, h, w, c).permute(0, 3, 1, 2)
        attended = x * channel_gate

        spatial_features = self.spatial_reduce(attended)
        spatial_features = channel_shuffle(spatial_features, self.shuffle_groups)
        spatial_gate = self.spatial_expand(spatial_features).sigmoid()
        attended = attended * spatial_gate

        return x + torch.tanh(self.gamma) * attended if self.shortcut else attended


# GAM Attention End
