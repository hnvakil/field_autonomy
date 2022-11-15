import torch
import torch.nn as nn
import torchvision
import torchvision.transforms as transforms
from dataset import TrailDataset
from torchvision.transforms import ToTensor, Lambda
from model import Net

batch_size = 64
num_classes = 3
learning_rate = 0.001
num_epochs = 3

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

all_transforms = transforms.Compose([transforms.Resize((32,32)),
                                     transforms.ToTensor(),
                                     transforms.Normalize(mean=[0.4914, 0.4822, 0.4465],
                                                          std=[0.2023, 0.1994, 0.2010])
                                     ])

#create full dataset
full_dataset = TrailDataset('labels.csv', 
    transform=all_transforms)
    # target_transform=transforms.Lambda(lambda y: torch.zeros(3, dtype=torch.float).scatter_(dim=0, index=torch.tensor(y), value=1)))

#split into train and test
train_size = int(0.8 * len(full_dataset))
test_size = len(full_dataset) - train_size
train_dataset, test_dataset = torch.utils.data.random_split(full_dataset, [train_size, test_size])

#make data loaders
train_loader = torch.utils.data.DataLoader(dataset = train_dataset, batch_size = batch_size, shuffle = True)
test_loader = torch.utils.data.DataLoader(dataset = test_dataset, batch_size = batch_size, shuffle = True)

model = Net(num_classes)
criterion = nn.CrossEntropyLoss()
optimizer = torch.optim.SGD(model.parameters(), lr=learning_rate, weight_decay = 0.005, momentum = 0.9)  
total_step = len(train_loader)

for epoch in range(num_epochs):
	#load in data with train loader
    for i, (images, labels) in enumerate(train_loader):  
        images = images.to(device)
        labels = labels.to(device)
        
        outputs = model(images)
        loss = criterion(outputs, labels)
        
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    print('Epoch [{}/{}], Loss: {:.4f}'.format(epoch+1, num_epochs, loss.item()))

print('Finished Training')
PATH = './cifar_net.pth'
torch.save(model.state_dict(), PATH)

with torch.no_grad():
    correct = 0
    total = 0
    for images, labels in test_loader:
        images = images.to(device)
        labels = labels.to(device)
        outputs = model(images)
        _, predicted = torch.max(outputs.data, 1)
        total += labels.size(0)
        correct += (predicted == labels).sum().item()
    
    print('Accuracy of the network on the {} train images: {} %'.format(len(test_loader), 100 * correct / total))

