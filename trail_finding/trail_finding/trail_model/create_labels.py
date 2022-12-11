import os  
import pandas as pd  

directory_path = "data"
dir_list = os.listdir(directory_path)

lc = 0
sc = 1
rc = 2

paths_list = []
labels_list = []

for dirpath, dirnames, filenames in os.walk("./data"):
    for filename in filenames:
        if filename.endswith(".jpg") or filename.endswith(".png"):
            path = os.path.join(dirpath, filename)
            print(path)
            if "rc" in path:
                labels_list.append(2)
                paths_list.append(path)
            elif "sc" in path:
                labels_list.append(1)
                paths_list.append(path)
            elif "lc" in path:
                labels_list.append(0)
                paths_list.append(path)

df = pd.DataFrame({'path': paths_list, 'label': labels_list}) 

# saving the dataframe 
df.to_csv('labels.csv', index=False) 



