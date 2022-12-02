from model_paper import Net
import torch
from PIL import Image
import torchvision.transforms as transforms


img_path = "/home/kat/ros2_ws/src/field_autonomy/trail_finding/trail_finding/trail_model/test_imgs/trail_1.jpg"
img_resize = 101
model_path = "/home/kat/ros2_ws/src/field_autonomy/trail_finding/trail_finding/trail_model/trained_models/paper_10e.pth"

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

def predict_image(image):
    image_tensor = all_transforms(image).float()
    image_tensor = image_tensor.unsqueeze_(0)
    # input = Variable(image_tensor)
    input = image_tensor.to(device)
    output = model(input)
    weights = output.data.cpu().numpy()
    index = weights.argmax()
    return index, weights

print(predict_image(img))

