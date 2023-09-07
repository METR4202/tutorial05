import os
import numpy as np
import matplotlib.pyplot as plt
from skimage import io

def in_collision(x_test, occ_map):

    for i in range(x_test.shape[0]):
    
        if occ_map[int(np.ceil(x_test[i][0])), int(np.ceil(x_test[i][1]))] > 0:
            return True
        
    return False

# Add your rrt helper functions here like holonomic steering dynamics etc.

def f_steer(x0, x1, v, t, varargin):
    x_new = np.zeros(3)

    max_travel_dist = v*t 
    d = np.sqrt(np.sum((x1 - x0[0:1])**2, 2)) 
    dtheta = 0  #not considering turning for holonomic vehicle

    travel_dist = min(d, max_travel_dist) # only steer as far as the straight line distance between the points
    x_new[0:1] = (travel_dist / d)*(x1 - x0[0:1]) + x0[1:2] 
    x_new[2] = np.atan2(x_new[2]-x0[2],x_new[1]-x0[1]) 

    disc = v*t # set number of discrete checks along edge to the travel distance for each steering command
    x_check = np.linspace(x0[1],x_new[1],disc) 
    y_check = np.linspace(x0[2],x_new[2],disc) 
    steer_path = [np.traspose(x_check), np.transpose(y_check)] 
    
    dxy = np.sqrt(sum(([x_check(1),y_check(1)] - [x_check(2),y_check(2)])**2, 2)) 

    return x_new, steer_path, dtheta, dxy

def main():
    maps_dir = "maps"
    map_files = [f for f in os.listdir(maps_dir) if f.endswith('_map.bmp')]
    f = np.zeros(len(map_files))

    # Set the robot steering parameters
    v = 1 ;                         # robot speed
    t = 100 ;                       # max steering timestep

    plot_online = False

    for k, map_file_name in enumerate(map_files):
        full_map_file_name = os.path.join(maps_dir, map_file_name)
        print("Mapping:", map_file_name)

        curr_map = io.imread(full_map_file_name)

        plt.figure(k)
        plt.imshow(np.flip(curr_map, axis=0), cmap="gray")
        plt.xlim([0, 1080])
        plt.ylim([0, 720])

        occ_map = (np.rot90(~curr_map, 3)>0).astype(np.uint8)
        X, Y = occ_map.shape

        n = 100 # Number of samples
        V = np.zeros((n, 3)) #set root node to (0,0,0)

        # Set the goal region [xmin, xmax, ymin, ymax]
        goal = [1030, 1080, 0, 50] # 50x50 square in the bottom right corner

        # Construct and search the tree
        best_parent = np.zeros((n,1)) # keep track of best parent node as we expand the tree
        path_found = False 

        E = []

        for i in range(1,n):
            # similar sampling to PRM
            while True:
                V_rand_x = np.random.randint(0,X)
                V_rand_y = np.random.randint(0,Y)
                V_rand = np.array([[V_rand_x, V_rand_y]])

                # Keep if sample is in free space
                if not in_collision(V_rand, occ_map):
                    dist = np.sqrt(np.sum(np.power(V_rand-V[:i-1,:1], 2), axis=1))
                    nn_idx = np.argmin(dist)
                    break
                # Steer from nearest vertex towards the random sample
                x_near, steer_path, dtheta, dxy = f_steer(V[nn_idx,:],V_rand,v,t) # need to implement the steering function 

            #add the rest of your rrt algorithm from here ...




if __name__ == "__main__":
    main()