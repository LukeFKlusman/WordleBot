"""
train_letter_cnn.py
════════════════════════════════════════════════════════════
Train the LetterCNN on images collected by collect_training_data.py

USAGE:
  python3 train_letter_cnn.py
  (run from perception/src/)

EXPECTS:
  ../data/raw/
    A/  *.png
    B/  *.png
    ...

OUTPUTS (all in ../outputs/):
  letter_cnn.pt               ← model weights
  training_report.txt         ← per-class accuracy (clean + hard test)
  confusion_matrix.png        ← final test confusion matrix
  confusion_matrix_hard.png   ← hard augmented test confusion matrix
  confusion_matrix_epoch_N.png← periodic validation snapshots
  training_curves.png         ← loss + accuracy over epochs

REQUIREMENTS:
  pip install torch torchvision scikit-learn matplotlib seaborn
"""

import os
import sys
import time
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, random_split, Subset
from torchvision import transforms
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from sklearn.metrics import confusion_matrix, classification_report
import seaborn as sns

# ── Paths — relative to perception/src/ ───────────────────
_HERE       = os.path.dirname(os.path.abspath(__file__))
DATA_ROOT   = os.path.join(_HERE, "../data/raw")
OUTPUTS_DIR = os.path.join(_HERE, "../outputs")
MODEL_OUT   = os.path.join(OUTPUTS_DIR, "letter_cnn.pt")

# ── Hyperparameters ────────────────────────────────────────
IMG_SIZE    = 64
BATCH_SIZE  = 32
EPOCHS      = 15
LR          = 1e-3
VAL_SPLIT   = 0.15
TEST_SPLIT  = 0.10
LABEL_MAP   = list("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789")
NUM_CLASSES = len(LABEL_MAP)
# ──────────────────────────────────────────────────────────


# ══════════════════════════════════════════════════════════
# DATASET
# ══════════════════════════════════════════════════════════

class LetterDataset(Dataset):
    """Loads all images from data/raw/<LABEL>/*.png"""
    def __init__(self, root, transform=None):
        self.samples   = []
        self.transform = transform
        missing        = []

        for idx, label in enumerate(LABEL_MAP):
            folder = os.path.join(root, label)
            if not os.path.exists(folder):
                missing.append(label)
                continue
            files = [f for f in os.listdir(folder) if f.lower().endswith('.png')]
            if not files:
                missing.append(label)
            for f in files:
                self.samples.append((os.path.join(folder, f), idx))

        if missing:
            print(f"[WARNING] Missing or empty classes: {missing}")
            print("  Collect more data for these letters before training.")

        print(f"[Dataset] {len(self.samples)} images across "
              f"{NUM_CLASSES - len(missing)}/{NUM_CLASSES} classes")

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        path, label = self.samples[idx]
        img = Image.open(path).convert('L')
        if self.transform:
            img = self.transform(img)
        return img, label


class TransformSubset(Dataset):
    """
    Wraps a Subset and applies a different transform.
    Fixes the data leakage bug where val_ds.dataset = LetterDataset(...)
    was replacing the whole dataset and breaking the index split.
    """
    def __init__(self, subset, transform):
        self.subset    = subset
        self.transform = transform

    def __len__(self):
        return len(self.subset)

    def __getitem__(self, idx):
        # Load raw image path and label from the original dataset
        path, label = self.subset.dataset.samples[self.subset.indices[idx]]
        img = Image.open(path).convert('L')
        if self.transform:
            img = self.transform(img)
        return img, label


# ══════════════════════════════════════════════════════════
# MODEL  (must match realsense_camera_cnn.py exactly)
# ══════════════════════════════════════════════════════════

class LetterCNN(nn.Module):
    def __init__(self, num_classes=36):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 32, kernel_size=3, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),           # 32×32

            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),           # 16×16

            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),           # 8×8
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(128 * 8 * 8, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(0.4),
            nn.Linear(256, num_classes),
        )

    def forward(self, x):
        return self.classifier(self.features(x))


# ══════════════════════════════════════════════════════════
# TRANSFORMS
# ══════════════════════════════════════════════════════════

# Training — heavy augmentation to simulate real camera variation
train_tf = transforms.Compose([
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.RandomRotation(20),
    transforms.RandomAffine(0, translate=(0.1, 0.1)),
    transforms.ColorJitter(brightness=0.3, contrast=0.3),
    transforms.RandomPerspective(distortion_scale=0.2, p=0.3),
    transforms.RandomHorizontalFlip(p=0.0),    # NEVER flip letters
    transforms.ToTensor(),
    transforms.Normalize((0.5,), (0.5,)),
    transforms.RandomErasing(p=0.1),
])

# Validation — clean images only, no augmentation
# Measures how well the model learned, not how robust it is
val_tf = transforms.Compose([
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.ToTensor(),
    transforms.Normalize((0.5,), (0.5,)),
])

# Hard test — aggressive augmentation simulating real-world conditions:
# large rotations, lighting changes, perspective distortion, blur, noise
# This is what actually predicts live camera performance
hard_test_tf = transforms.Compose([
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.RandomRotation(30),                              # cards held at angles
    transforms.RandomAffine(0, translate=(0.15, 0.15)),        # off-centre cards
    transforms.ColorJitter(brightness=0.5, contrast=0.5),      # harsh lighting
    transforms.RandomPerspective(distortion_scale=0.35, p=0.8),# angled camera view
    transforms.GaussianBlur(kernel_size=3, sigma=(0.1, 1.5)),  # focus blur
    transforms.ToTensor(),
    transforms.Normalize((0.5,), (0.5,)),
    transforms.RandomErasing(p=0.2),                           # partial occlusion
])


# ══════════════════════════════════════════════════════════
# TRAINING
# ══════════════════════════════════════════════════════════

def make_confusion_matrix(cm, title, cmap, path, best_val_acc=None):
    """Plot confusion matrix with labels on all four sides and sample counts on right."""
    row_totals = cm.sum(axis=1)  # total samples per class

    fig, axes = plt.subplots(1, 2, figsize=(19, 14),
                             gridspec_kw={'width_ratios': [16, 1], 'wspace': 0.02})
    ax, ax_counts = axes

    sns.heatmap(cm, annot=True, fmt='d', cmap=cmap,
                xticklabels=LABEL_MAP, yticklabels=LABEL_MAP, ax=ax)

    ax.set_ylabel("ACTUAL LETTER (True Label)", fontsize=12, fontweight='bold')
    ax.set_xlabel("PREDICTED LETTER (Model Guess)", fontsize=12, fontweight='bold')

    # Mirror tick labels to top and right
    ax.xaxis.set_ticks_position('both')
    ax.yaxis.set_ticks_position('both')
    ax.tick_params(top=True, labeltop=True, bottom=True, labelbottom=True)
    ax.tick_params(left=True, labelleft=True, right=True, labelright=False)

    # Top axis label
    ax2 = ax.twiny()
    ax2.set_xlim(ax.get_xlim())
    ax2.set_xticks(ax.get_xticks())
    ax2.set_xticklabels(LABEL_MAP)
    ax2.set_xlabel("PREDICTED LETTER (Model Guess)", fontsize=12, fontweight='bold')

    # ── Sample count bar on right ──────────────────────────
    n = len(LABEL_MAP)
    colours = ['#d73027' if t < 10 else '#fee08b' if t < 20 else '#1a9850'
               for t in row_totals]
    ax_counts.barh(range(n), row_totals, color=colours, edgecolor='white', height=0.8)
    ax_counts.set_ylim(ax.get_ylim())
    ax_counts.set_yticks(range(n))
    ax_counts.set_yticklabels(LABEL_MAP, fontsize=8)
    ax_counts.yaxis.set_label_position('right')
    ax_counts.yaxis.tick_right()
    ax_counts.set_xlabel("N", fontsize=9)
    ax_counts.xaxis.set_label_position('bottom')
    ax_counts.tick_params(axis='x', labelsize=7)
    ax_counts.set_title("Samples", fontsize=9, pad=4)
    ax_counts.invert_yaxis()

    # Annotate each bar with the count, flag low samples with warning
    for i, v in enumerate(row_totals):
        label = f"{v} ⚠" if v < 10 else str(v)
        ax_counts.text(v + 0.3, i, label, va='center', fontsize=7,
                       color='#d73027' if v < 10 else 'black')

    ax.set_title(title, fontsize=13, fontweight='bold', pad=40)
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()


def train():
    os.makedirs(OUTPUTS_DIR, exist_ok=True)
    print(f"[Train] Outputs → {OUTPUTS_DIR}/")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[Train] Device: {device}")

    # ── Load full dataset with no transform (raw PIL images) ─
    # TransformSubset applies the right transform per split
    base_dataset = LetterDataset(DATA_ROOT, transform=None)
    if len(base_dataset) == 0:
        print("[ERROR] No images found. Run collect_training_data.py first.")
        sys.exit(1)

    n_total = len(base_dataset)
    n_val   = int(n_total * VAL_SPLIT)
    n_test  = int(n_total * TEST_SPLIT)
    n_train = n_total - n_val - n_test

    # Split indices — fixed seed for reproducibility
    train_sub, val_sub, test_sub = random_split(
        base_dataset, [n_train, n_val, n_test],
        generator=torch.Generator().manual_seed(42)
    )

    # Apply correct transform to each split — NO leakage
    train_ds      = TransformSubset(train_sub, train_tf)
    val_ds        = TransformSubset(val_sub,   val_tf)
    test_ds       = TransformSubset(test_sub,  val_tf)       # clean test
    hard_test_ds  = TransformSubset(test_sub,  hard_test_tf) # hard augmented test

    train_loader     = DataLoader(train_ds,     batch_size=BATCH_SIZE, shuffle=True,  num_workers=2)
    val_loader       = DataLoader(val_ds,       batch_size=BATCH_SIZE, shuffle=False, num_workers=2)
    test_loader      = DataLoader(test_ds,      batch_size=BATCH_SIZE, shuffle=False, num_workers=2)
    hard_test_loader = DataLoader(hard_test_ds, batch_size=BATCH_SIZE, shuffle=False, num_workers=2)

    print(f"[Train] Split → train: {n_train}  val: {n_val}  test: {n_test}")
    print(f"[Train] Hard test uses same {n_test} images with aggressive augmentation\n")

    # ── Model, loss, optimiser ────────────────────────────
    model     = LetterCNN(num_classes=NUM_CLASSES).to(device)
    criterion = nn.CrossEntropyLoss(label_smoothing=0.05)
    optimizer = optim.AdamW(model.parameters(), lr=LR, weight_decay=1e-4)
    scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=EPOCHS)

    best_val_acc = 0.0
    train_losses, val_accs = [], []

    print(f"[Train] Starting {EPOCHS} epochs...\n")

    for epoch in range(1, EPOCHS + 1):
        # ── Training pass ─────────────────────────────────
        model.train()
        running_loss = 0.0
        t0 = time.time()
        for imgs, labels in train_loader:
            imgs, labels = imgs.to(device), labels.to(device)
            optimizer.zero_grad()
            loss = criterion(model(imgs), labels)
            loss.backward()
            optimizer.step()
            running_loss += loss.item() * imgs.size(0)
        scheduler.step()
        avg_loss = running_loss / n_train

        # ── Validation pass ───────────────────────────────
        model.eval()
        correct = 0
        with torch.no_grad():
            for imgs, labels in val_loader:
                imgs, labels = imgs.to(device), labels.to(device)
                preds = model(imgs).argmax(dim=1)
                correct += (preds == labels).sum().item()
        val_acc = correct / n_val * 100

        train_losses.append(avg_loss)
        val_accs.append(val_acc)

        elapsed = time.time() - t0
        print(f"  Epoch {epoch:02d}/{EPOCHS}  "
              f"loss: {avg_loss:.4f}  "
              f"val_acc: {val_acc:.1f}%  "
              f"({elapsed:.1f}s)")

        if val_acc > best_val_acc:
            best_val_acc = val_acc
            torch.save(model.state_dict(), MODEL_OUT)
            print(f"    ✓ Best model saved → {MODEL_OUT}")

        # ── Periodic confusion matrix every 5 epochs ──────
        if epoch % 5 == 0:
            model.eval()
            temp_preds, temp_labels = [], []
            with torch.no_grad():
                for imgs, labels in val_loader:
                    imgs = imgs.to(device)
                    preds = model(imgs).argmax(dim=1).cpu().numpy()
                    temp_preds.extend(preds)
                    temp_labels.extend(labels.numpy())

            cm = confusion_matrix(temp_labels, temp_preds)
            make_confusion_matrix(
                cm,
                title=f"Confusion Matrix — Epoch {epoch} (Val Acc: {val_acc:.1f}%)",
                cmap='YlGnBu',
                path=os.path.join(OUTPUTS_DIR, f"confusion_matrix_epoch_{epoch}.png"),
            )
            print(f"    ✓ Confusion matrix saved → epoch_{epoch}.png")

    # ══════════════════════════════════════════════════════
    # EVALUATION
    # ══════════════════════════════════════════════════════
    print(f"\n[Eval] Loading best model...")
    model.load_state_dict(torch.load(MODEL_OUT, map_location=device))
    model.eval()

    def evaluate(loader, name):
        all_preds, all_labels = [], []
        with torch.no_grad():
            for imgs, labels in loader:
                imgs = imgs.to(device)
                preds = model(imgs).argmax(dim=1).cpu().numpy()
                all_preds.extend(preds)
                all_labels.extend(labels.numpy())
        report = classification_report(
            all_labels, all_preds,
            target_names=LABEL_MAP, zero_division=0
        )
        print(f"\n{'═'*60}")
        print(f"  {name}")
        print(f"{'═'*60}")
        print(report)
        return all_labels, all_preds, report

    # ── Clean test ────────────────────────────────────────
    clean_labels, clean_preds, clean_report = evaluate(test_loader, "CLEAN TEST SET")

    # ── Hard augmented test ───────────────────────────────
    hard_labels, hard_preds, hard_report = evaluate(
        hard_test_loader,
        "HARD TEST SET (rotation ±30°, lighting, blur, perspective, occlusion)"
    )

    # ── Save text report ──────────────────────────────────
    with open(os.path.join(OUTPUTS_DIR, "training_report.txt"), "w") as f:
        f.write(f"Best validation accuracy: {best_val_acc:.1f}%\n")
        f.write(f"{'═'*60}\n")
        f.write("CLEAN TEST SET\n")
        f.write(f"{'═'*60}\n")
        f.write(clean_report + "\n")
        f.write(f"{'═'*60}\n")
        f.write("HARD TEST SET (rotation, lighting, blur, perspective, occlusion)\n")
        f.write(f"{'═'*60}\n")
        f.write(hard_report + "\n")
    print("\n[Done] Report saved → training_report.txt")

    # ── Clean confusion matrix ────────────────────────────
    cm = confusion_matrix(clean_labels, clean_preds)
    make_confusion_matrix(
        cm,
        title=f"Clean Test Set — Best val acc: {best_val_acc:.1f}%",
        cmap='Blues',
        path=os.path.join(OUTPUTS_DIR, "confusion_matrix.png"),
    )
    print("[Done] Clean confusion matrix saved → confusion_matrix.png")

    # ── Hard confusion matrix ─────────────────────────────
    cm_hard = confusion_matrix(hard_labels, hard_preds)
    make_confusion_matrix(
        cm_hard,
        title="Hard Test Set (rotation, lighting, blur, perspective, occlusion)",
        cmap='Oranges',
        path=os.path.join(OUTPUTS_DIR, "confusion_matrix_hard.png"),
    )
    print("[Done] Hard confusion matrix saved → confusion_matrix_hard.png")

    # ── Training curves ───────────────────────────────────
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
    ax1.plot(train_losses)
    ax1.set_title("Training Loss")
    ax1.set_xlabel("Epoch")
    ax2.plot(val_accs)
    ax2.set_title("Validation Accuracy %")
    ax2.set_xlabel("Epoch")
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUTS_DIR, "training_curves.png"), dpi=150)
    plt.close()
    print("[Done] Training curves saved → training_curves.png")

    print(f"\n✅ Training complete.")
    print(f"   Best val accuracy : {best_val_acc:.1f}%")
    print(f"   Model saved to    : {MODEL_OUT}")
    print(f"\n   Hard test accuracy is your real-world prediction.")
    print(f"   If hard << clean, collect more varied training data.")


if __name__ == '__main__':
    train()