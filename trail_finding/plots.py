import matplotlib.pyplot as plt
import numpy
from sklearn import metrics
from dataset import TrailDataset
from model_paper import Net
import torch
from PIL import Image
import torchvision.transforms as transforms


img_path = "test_imgs/trail_1.jpg"
img_resize = 101
model_path = "trained_models/paper_10e.pth"

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
img = Image.open(img_path)

all_transforms = transforms.Compose([transforms.Resize((img_resize,img_resize)),
                                     transforms.ToTensor(),
                                     transforms.Normalize(mean=[0.4914, 0.4822, 0.4465],
                                                          std=[0.2023, 0.1994, 0.2010])
                                     ])

model = Net(3)
model.load_state_dict(torch.load(model_path))
model.eval()

full_dataset = TrailDataset('labels.csv', 
    transform=all_transforms)

test_loader = torch.utils.data.DataLoader(dataset = full_dataset, batch_size = 64, shuffle = True)

actual_labels = []
predicted_labels = []

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
        actual_labels += labels
        predicted_labels += predicted
    
    print('Accuracy of the network on the {} test images: {} %'.format(len(test_loader), 100 * correct / total))

confusion_matrix = metrics.confusion_matrix(actual_labels, predicted_labels)
cm_display = metrics.ConfusionMatrixDisplay(confusion_matrix = confusion_matrix, display_labels = [False, True])
cm_display.plot()
plt.show()

