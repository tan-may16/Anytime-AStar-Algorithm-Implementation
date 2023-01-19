import numpy as np
import matplotlib.pyplot as plt

def read_txt(path):
    with open(path) as f:
        for line in f:
            if (line[0]=="#"): 
                x_size,y_size = line[1:].split(",")
                x_size,y_size = int(x_size),int(y_size)
                break
            
    map = np.zeros((x_size,y_size))
    x_ = []
    y_ = []
    with open(path) as f:
        for line in f:
            if (line[0]=="#"): continue
            x, y, z = line.split(",")
            x, y, z = int(x), int(y),int(z)
            x_.append(x)
            y_.append(y)
            map[x,y] = z
    
    return map
def visualize_path(map, path_list):
    for i in range(len(path_list)):
        map[path_list[i][0],path_list[i][1]] = 150
    plt.matshow(map.T)
    plt.show()
    return map
def save_path(map):
    plt.imsave('Images/plan.png', map.T)