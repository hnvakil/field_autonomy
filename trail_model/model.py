import os
import torch
from torch import nn
from torch.utils.data import DataLoader
from torchvision import datasets, transforms
import torch.nn.functional as F

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        
        self.conv1 = nn.Conv2d(in_channels=3, out_channels=12, kernel_size=5, stride=1, padding=1)
        self.bn1 = nn.BatchNorm2d(12)
        self.conv2 = nn.Conv2d(in_channels=12, out_channels=12, kernel_size=5, stride=1, padding=1)
        self.bn2 = nn.BatchNorm2d(12)
        self.pool = nn.MaxPool2d(2,2)
        self.conv4 = nn.Conv2d(in_channels=12, out_channels=24, kernel_size=5, stride=1, padding=1)
        self.bn4 = nn.BatchNorm2d(24)
        self.conv5 = nn.Conv2d(in_channels=24, out_channels=24, kernel_size=5, stride=1, padding=1)
        self.bn5 = nn.BatchNorm2d(24)
        self.fc1 = nn.Linear(24*10*10, 3)

    def forward(self, input):
        output = F.relu(self.bn1(self.conv1(input)))      
        output = F.relu(self.bn2(self.conv2(output)))     
        output = self.pool(output)                        
        output = F.relu(self.bn4(self.conv4(output)))     
        output = F.relu(self.bn5(self.conv5(output)))     
        output = output.view(-1, 24*10*10)
        output = self.fc1(output)

        return output


# class Net(nn.Module):
#     def __init__(self):
#         super(Net, self).__init__()
#         self.conv1 = nn.Conv2d(3,32,4)
#         self.pool1 = nn.MaxPool2d(2)
#         # self.flatten = nn.Flatten()
#         # self.linear_relu_stack = nn.Sequential(
#         #     # nn.Conv2d(3, 32, 4),
#         #     # nn.MaxPool2d(2),
#         #     # nn.Conv2d(49,46,4),
#         #     # nn.MaxPool2d(2),
#         #     # nn.Conv2d(46,23,4),
#         #     # nn.MaxPool2d(2),
#         #     # nn.Conv2d(23,20,4),
#         #     # nn.MaxPool2d(2),
#         #     nn.Linear(101*101*3,3)
#         # )

#     def forward(self, x):
#         output = F.relu(self.conv1(x))
#         output = self.pool1(output)
        
#         # x = self.flatten(x)
#         # output = self.linear_relu_stack(x)
#         return output