import os  
import pandas as pd  

directory_path = "data"
dir_list = os.listdir(directory_path)

#lc = 0
#sc = 1
#rc = 2

lc_list = []
rc_list = []
sc_list = []

for section in dir_list:
    for frame in os.scandir(f'data/{section}/videos/lc'):
        if frame.is_dir():
            for img in os.scandir(frame.path):
                lc_list.append(img.path)
    for frame in os.scandir(f'data/{section}/videos/sc'):
        if frame.is_dir():
            for img in os.scandir(frame.path):
                sc_list.append(img.path) 
    for frame in os.scandir(f'data/{section}/videos/rc'):
        if frame.is_dir():
            for img in os.scandir(frame.path):
                rc_list.append(img.path)


paths_list = lc_list + rc_list + sc_list
labels_list = [0 for i in range(len(lc_list))] + [2 for i in range(len(rc_list))] + [1 for i in range(len(sc_list))]

df = pd.DataFrame({'path': paths_list, 'label': labels_list}) 

# saving the dataframe 
df.to_csv('labels.csv', index=False) 



