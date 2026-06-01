import torch
import math
import warnings

class WarmupScheduler:
    """Learning rate scheduler that implements a linear warmup phase followed by a decay phase.
    
    The decay phase can be either Cosine Annealing (CosineAnnealingLR) or Reduce on Plateau (ReduceLROnPlateau).
    """
    def __init__(self, optimizer, warmup_epochs, initial_lr, total_epochs, target_scheduler_type='cosine', min_lr=1e-6, patience=5, factor=0.5):
        self.optimizer = optimizer
        self.warmup_epochs = max(0, warmup_epochs)
        self.initial_lr = initial_lr
        self.total_epochs = total_epochs
        self.target_scheduler_type = target_scheduler_type.lower()
        self.min_lr = min_lr
        self.patience = patience
        self.factor = factor
        self.current_epoch = 0
        
        # Set initial learning rate to warmup start value
        if self.warmup_epochs > 0:
            self._set_lr(self.initial_lr / self.warmup_epochs)
        else:
            self._set_lr(self.initial_lr)
            
        self.target_scheduler = None

    def _set_lr(self, lr):
        for param_group in self.optimizer.param_groups:
            param_group['lr'] = lr

    def get_lr(self):
        return [param_group['lr'] for param_group in self.optimizer.param_groups]

    def step(self, val_loss=None):
        self.current_epoch += 1
        if self.current_epoch <= self.warmup_epochs:
            # Linear warmup
            lr = self.initial_lr * (self.current_epoch / self.warmup_epochs)
            self._set_lr(lr)
        else:
            # Post-warmup phase
            if self.target_scheduler is None:
                # Initialize target scheduler
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    if self.target_scheduler_type == 'cosine':
                        self.target_scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
                            self.optimizer,
                            T_max=max(1, self.total_epochs - self.warmup_epochs),
                            eta_min=self.min_lr
                        )
                    elif self.target_scheduler_type in ['plateau', 'reducelronplateau']:
                        self.target_scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
                            self.optimizer,
                            mode='min',
                            patience=self.patience,
                            factor=self.factor
                        )
            
            # Step target scheduler
            if self.target_scheduler is not None:
                if isinstance(self.target_scheduler, torch.optim.lr_scheduler.ReduceLROnPlateau):
                    self.target_scheduler.step(val_loss)
                else:
                    self.target_scheduler.step()

    def state_dict(self):
        return {
            'warmup_epochs': self.warmup_epochs,
            'initial_lr': self.initial_lr,
            'total_epochs': self.total_epochs,
            'target_scheduler_type': self.target_scheduler_type,
            'min_lr': self.min_lr,
            'patience': self.patience,
            'factor': self.factor,
            'current_epoch': self.current_epoch,
            'target_scheduler_state': self.target_scheduler.state_dict() if self.target_scheduler is not None else None
        }

    def load_state_dict(self, state_dict):
        self.warmup_epochs = state_dict['warmup_epochs']
        self.initial_lr = state_dict['initial_lr']
        self.total_epochs = state_dict['total_epochs']
        self.target_scheduler_type = state_dict['target_scheduler_type']
        self.min_lr = state_dict['min_lr']
        self.patience = state_dict['patience']
        self.factor = state_dict['factor']
        self.current_epoch = state_dict['current_epoch']
        
        if state_dict['target_scheduler_state'] is not None:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                if self.target_scheduler_type == 'cosine':
                    self.target_scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
                        self.optimizer,
                        T_max=max(1, self.total_epochs - self.warmup_epochs),
                        eta_min=self.min_lr
                    )
                elif self.target_scheduler_type in ['plateau', 'reducelronplateau']:
                    self.target_scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
                        self.optimizer,
                        mode='min',
                        patience=self.patience,
                        factor=self.factor
                    )
            if self.target_scheduler is not None:
                self.target_scheduler.load_state_dict(state_dict['target_scheduler_state'])
