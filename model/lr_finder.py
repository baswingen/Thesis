import torch
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

class LRFinder:
    """
    Learning Rate Finder: exponentially increases learning rate batch-by-batch
    to figure out the optimal max_lr for OneCycleLR.
    """
    def __init__(self, model, optimizer, criterion, device):
        self.model = model
        self.optimizer = optimizer
        self.criterion = criterion
        self.device = device
        
        self.history = {"lr": [], "loss": []}

    def range_test(self, train_loader, end_lr=10, num_iter=100, smooth_f=0.05, diverge_th=5):
        self.model.train()
        
        start_lr = self.optimizer.param_groups[0]['lr']
        lr_mult = (end_lr / start_lr) ** (1 / num_iter)
        
        avg_loss = 0.
        best_loss = float('inf')
        
        loader_iter = iter(train_loader)
        
        print(f"Running LR Finder from {start_lr} to {end_lr} over {num_iter} iterations...")
        
        for i in tqdm(range(num_iter), desc="LR Finder iterations"):
            try:
                batch = next(loader_iter)
            except StopIteration:
                loader_iter = iter(train_loader)
                batch = next(loader_iter)
                
            # Match the return signature of raw_pad_collate_fn
            if len(batch) == 5:
                batch_x, batch_y, lengths, _pids, batch_weights = batch
                batch_x = batch_x.transpose(1, 2).to(self.device)  # (B, C, T)
                batch_y = batch_y.to(self.device)
                lengths = lengths.to(self.device)
                batch_weights = batch_weights.to(self.device)
                
                self.optimizer.zero_grad()
                outputs = self.model(batch_x, lengths)
                
                # Assume criterion(reduction='none') isn't set, we keep it simple for finder
                # Or handle both cases:
                if getattr(self.criterion, 'reduction', 'mean') == 'none':
                    loss = (self.criterion(outputs, batch_y) * batch_weights).mean()
                else:
                    loss = self.criterion(outputs, batch_y)
                    
            else:
                # Fallback for standard loaders
                batch_x, batch_y = batch[0].to(self.device), batch[1].to(self.device)
                self.optimizer.zero_grad()
                outputs = self.model(batch_x)
                loss = self.criterion(outputs, batch_y)
                
            loss.backward()
            torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)
            self.optimizer.step()
            
            # Exponential moving average for smoothing the loss
            if i == 0:
                avg_loss = loss.item()
            else:
                avg_loss = smooth_f * loss.item() + (1 - smooth_f) * avg_loss
                
            smoothed_loss = avg_loss / (1 - (1 - smooth_f) ** (i + 1))
            
            if i > 0 and smoothed_loss > diverge_th * best_loss:
                print("\nStopping early, loss has diverged.")
                break
                
            if smoothed_loss < best_loss or i == 0:
                best_loss = smoothed_loss
                
            self.history["lr"].append(start_lr * (lr_mult ** i))
            self.history["loss"].append(smoothed_loss)
            
            # Step the LR
            for param_group in self.optimizer.param_groups:
                param_group['lr'] *= lr_mult
                
    def plot(self, save_path=None):
        lrs = self.history["lr"]
        losses = self.history["loss"]
        
        plt.figure(figsize=(10, 6))
        plt.plot(lrs, losses)
        plt.xscale("log")
        plt.xlabel("Learning Rate (Log Scale)")
        plt.ylabel("Loss")
        plt.title("Learning Rate Finder Sweep")
        plt.grid()
        
        if save_path:
            plt.savefig(save_path)
            print(f"Saved LR Finder plot to {save_path}")
        else:
            plt.show()
        plt.close()
