import logging
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader


class Trainer(object):
    def __init__(self, model, memory, device, batch_size):
        """
        Train the trainable model of a policy
        """
        self.model = model
        self.device = device
        self.criterion = nn.MSELoss().to(device)
        self.memory = memory
        self.data_loader = None
        self.batch_size = batch_size
        self.optimizer = None

    def set_learning_rate(self, learning_rate):
        logging.info('Current learning rate: %f', learning_rate)
        self.optimizer = optim.SGD(self.model.parameters(), lr=learning_rate, momentum=0.9)

    def _create_data_loader(self):
        """Create DataLoader for training"""
        # Note: num_workers=0 to avoid CUDA multiprocessing issues
        # pin_memory=False because data may already be on GPU
        self.data_loader = DataLoader(
            self.memory,
            self.batch_size,
            shuffle=True,
            num_workers=0,
            pin_memory=False
        )

    def optimize_epoch(self, num_epochs):
        if self.optimizer is None:
            raise ValueError('Learning rate is not set!')
        if self.data_loader is None:
            self._create_data_loader()
        average_epoch_loss = 0
        for epoch in range(num_epochs):
            epoch_loss = 0
            for data in self.data_loader:
                inputs, values = data
                # Move data to GPU
                inputs = inputs.to(self.device, non_blocking=True)
                values = values.to(self.device, non_blocking=True)

                self.optimizer.zero_grad()
                outputs = self.model(inputs)
                loss = self.criterion(outputs, values)
                loss.backward()
                self.optimizer.step()
                epoch_loss += loss.data.item()

            average_epoch_loss = epoch_loss / len(self.memory)
            logging.debug('Average loss in epoch %d: %.2E', epoch, average_epoch_loss)

        return average_epoch_loss

    def optimize_batch(self, num_batches):
        if self.optimizer is None:
            raise ValueError('Learning rate is not set!')
        if self.data_loader is None:
            self._create_data_loader()
        losses = 0
        batch_iter = iter(self.data_loader)
        for _ in range(num_batches):
            try:
                inputs, values = next(batch_iter)
            except StopIteration:
                batch_iter = iter(self.data_loader)
                inputs, values = next(batch_iter)

            # Move data to GPU
            inputs = inputs.to(self.device, non_blocking=True)
            values = values.to(self.device, non_blocking=True)

            self.optimizer.zero_grad()
            outputs = self.model(inputs)
            loss = self.criterion(outputs, values)
            loss.backward()
            self.optimizer.step()
            losses += loss.data.item()

        average_loss = losses / num_batches
        logging.debug('Average loss : %.2E', average_loss)

        return average_loss
