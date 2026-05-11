#!/usr/bin/env python3
import os, json, glob
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader, random_split
import torch.nn as nn, torch.optim as optim
import matplotlib.pyplot as plt

class ERIDataset(Dataset):
    def __init__(self, root):
        self.X, self.y = [], []
        for f in glob.glob(os.path.join(root, "*", "samples.jsonl")):
            with open(f) as fp:
                for line in fp:
                    row = json.loads(line)
                    if "features" in row and "eri_label" in row:
                        self.X.append(row["features"])
                        self.y.append(row["eri_label"])
        self.X = torch.tensor(self.X, dtype=torch.float32)
        self.y = torch.tensor(self.y, dtype=torch.float32).view(-1, 1)
        print(f"Loaded {len(self.X)} samples from {root}")

    def __len__(self):  return len(self.X)
    def __getitem__(self, idx):  return self.X[idx], self.y[idx]

# if __name__ == "__main__":
    # root = os.path.join(os.path.dirname(__file__), "data/ros_data")
    # ds = ERIDataset(root)
    # print(ds.X[:5], ds.y[:5])


class ERINet(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(2, 32), nn.ReLU(),
            nn.Linear(32, 32), nn.ReLU(),
            nn.Linear(32, 1), nn.Sigmoid()
        )
    def forward(self, x): return self.net(x)

# 确保目录创建函数在最前面
def ensure_dir(path):
    """确保目录存在，如果不存在则创建"""
    os.makedirs(path, exist_ok=True)

def visualize_prediction(model, epoch=None):
    """可视化模型预测结果"""
    # 确定保存目录
    base_dir = os.path.join(os.path.dirname(__file__), "imilearning_model/visualizations")
    ensure_dir(base_dir)  # 确保目录存在
    
    with torch.no_grad():
        # 创建网格数据
        t_vals = np.linspace(0, 1, 40)
        r_vals = np.linspace(0, 1, 40)
        Xgrid = torch.tensor([[t, r] for t in t_vals for r in r_vals], dtype=torch.float32)
        
        # 预测并转换格式
        Ygrid = model(Xgrid).numpy().reshape(len(t_vals), len(r_vals))
    
    # 绘制结果
    plt.figure(figsize=(10, 8))
    plt.imshow(Ygrid, origin='lower', extent=[0, 1, 0, 1], aspect='auto', cmap='viridis')
    plt.xlabel("tau")
    plt.ylabel("rho")
    title = "Predicted ERI"
    if epoch is not None:
        title += f" (Epoch {epoch})"
    plt.title(title)
    plt.colorbar(label="ERI value")
    
    # 构建完整的保存路径
    filename = f"pred_epoch_{epoch}.png" if epoch is not None else "final_pred.png"
    save_path = os.path.join(base_dir, filename)
    plt.savefig(save_path, bbox_inches='tight')
    plt.close()  # 关闭图形避免内存泄漏
    
    print(f"可视化结果已保存至: {save_path}")


def save_model_as_torchscript(model, save_path):
    """将模型保存为TorchScript格式"""
    try:
        # 创建示例输入
        example_input = torch.rand(1, 2)  # 与你的模型输入维度匹配
        
        # 追踪模型
        traced_model = torch.jit.trace(model, example_input)
        
        # 保存TorchScript模型
        traced_model.save(save_path)
        print(f"TorchScript模型已保存至: {save_path}")
        return True
    except Exception as e:
        print(f"保存TorchScript模型失败: {e}")
        return False


if __name__ == "__main__":
    # 确保主目录存在
    base_dir = os.path.join(os.path.dirname(__file__), "imilearning_model")
    ensure_dir(base_dir)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = ERINet().to(device)

    # 数据集加载代码
    ds = ERIDataset(os.path.join(os.path.dirname(__file__), "data/ros_data"))
    n_train = int(0.8 * len(ds))
    n_val = len(ds) - n_train
    train_ds, val_ds = random_split(ds, [n_train, n_val])
    train_loader = DataLoader(train_ds, batch_size=64, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=128)
    
    # 模型文件保存路径
    model_save_path = os.path.join(base_dir, "eri_net_best.pt")
    torchscript_save_path = os.path.join(base_dir, "eri_net_ts.pt")

    model = ERINet()
    loss_fn = nn.MSELoss()
    opt = optim.Adam(model.parameters(), lr=1e-3)

    best_val = 1e9
    for epoch in range(50):
        model.train()
        for X, y in train_loader:
            opt.zero_grad()
            loss = loss_fn(model(X), y)
            loss.backward()
            opt.step()
        
        # 验证
        model.eval()
        val_loss = np.mean([loss_fn(model(X), y).item() for X, y in val_loader])
        print(f"Epoch {epoch:02d}: val_loss={val_loss:.4f}")
        
        # 保存最佳模型
        if val_loss < best_val:
            best_val = val_loss
            torch.save(model.state_dict(), model_save_path)
            save_model_as_torchscript(model, torchscript_save_path)
            print(f"保存新最佳模型 (loss={val_loss:.4f}) 到 {model_save_path}")
            
            # 可视化当前最佳模型的预测
            visualize_prediction(model, epoch)

    # 训练结束后的最终可视化
    print("\n训练完成，加载最佳模型进行最终可视化")
    model.load_state_dict(torch.load(model_save_path))
    visualize_prediction(model)
    save_model_as_torchscript(model, torchscript_save_path)

