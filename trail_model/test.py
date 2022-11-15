from torch.utils.data import DataLoader
from dataset import TrailDataset
import matplotlib.pylab as plt
import torch
from torchvision.transforms import ToTensor, Lambda
from model import Net
import torch.optim as optim
import torch.nn as nn
import numpy as np
from torchvision.utils import make_grid
from torch.autograd import Variable

classes = ('left', 'center', 'right')

full_dataset = TrailDataset('labels.csv', 
    target_transform=Lambda(lambda y: torch.zeros(3, dtype=torch.float).scatter_(dim=0, index=torch.tensor(y), value=1)))

train_size = int(0.8 * len(full_dataset))
test_size = len(full_dataset) - train_size
train_dataset, test_dataset = torch.utils.data.random_split(full_dataset, [train_size, test_size])

train_dataloader = DataLoader(train_dataset, batch_size=64, shuffle=True)
test_dataloader = DataLoader(test_dataset, batch_size=64, shuffle=True)

train_features, train_labels = next(iter(train_dataloader))
print(f"Feature batch shape: {train_features.size()}")
print(f"Labels batch shape: {train_labels.size()}")

device = "cuda" if torch.cuda.is_available() else "cpu"

model = Net().to(device)
print(model)


criterion = nn.CrossEntropyLoss()
optimizer = optim.SGD(model.parameters(), lr=0.001, momentum=0.9)

for epoch in range(2):  # loop over the dataset multiple times

    running_loss = 0.0
    for i, data in enumerate(train_dataloader, 0):
        # get the inputs; data is a list of [inputs, labels]
        inputs, labels = data

        # zero the parameter gradients
        optimizer.zero_grad()
        # forward + backward + optimize
        outputs = model(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()

        # print statistics
        running_loss += loss.item()
        if i % 2000 == 1999:    # print every 2000 mini-batches
            print(f'[{epoch + 1}, {i + 1:5d}] loss: {running_loss / 2000:.3f}')
            running_loss = 0.0

print('Finished Training')
PATH = './cifar_net.pth'
torch.save(model.state_dict(), PATH)
