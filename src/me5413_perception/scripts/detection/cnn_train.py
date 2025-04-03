#!/usr/bin/env python3
import os
import torch
import torch.nn as nn
import torchvision.transforms as transforms
import torchvision.datasets as datasets
from torch.utils.data import DataLoader

class SimpleCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(1, 32, 3, 1),  # 28x28 -> 26x26
            nn.ReLU(),
            nn.MaxPool2d(2),         # -> 13x13
            nn.Conv2d(32, 64, 3, 1), # -> 11x11
            nn.ReLU(),
            nn.MaxPool2d(2)          # -> 5x5
        )
        self.fc = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64*5*5, 128),
            nn.ReLU(),
            nn.Linear(128, 10)
        )

    def forward(self, x):
        return self.fc(self.conv(x))

transform = transforms.Compose([
    transforms.Grayscale(),
    transforms.Resize((28, 28)),
    transforms.ToTensor()
])

train_dataset = datasets.ImageFolder("dataset/train", transform=transform)
val_dataset = datasets.ImageFolder("dataset/val", transform=transform)

train_loader = DataLoader(train_dataset, batch_size=64, shuffle=True)
val_loader = DataLoader(val_dataset, batch_size=64)

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = SimpleCNN().to(device)
optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
criterion = nn.CrossEntropyLoss()

for epoch in range(10):
    model.train()
    total, correct = 0, 0
    for img, label in train_loader:
        img, label = img.to(device), label.to(device)
        pred = model(img)
        loss = criterion(pred, label)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        correct += (pred.argmax(1) == label).sum().item()
        total += label.size(0)
    print(f"[Epoch {epoch+1}] Train Acc: {correct/total:.4f}")

# Save
torch.save(model.state_dict(), "simple_cnn.pt")
print("Saved as simple_cnn.pt")
