import os
import numpy as np
import matplotlib.pyplot as plt
from skimage import io

def in_collision(x_test, occ_map):

    for i in range(x_test.shape[0]):
    
        if occ_map[int(np.ceil(x_test[i][0])), int(np.ceil(x_test[i][1]))] > 0:
            return True
        
    return False


def main():
    maps_dir = "maps"
    map_files = [f for f in os.listdir(maps_dir) if f.endswith('_map.bmp')]
    f = np.zeros(len(map_files))

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

        # Number of samples
        n = 500
        V = np.zeros((n, 2))

        # Sample
        for i in range(n):
            while True:
                V_rand_x = np.random.randint(0,X)
                V_rand_y = np.random.randint(0,Y)
                V_rand = np.array([[V_rand_x, V_rand_y]])

                # Keep if sample is in free space
                if not in_collision(V_rand, occ_map):
                    V[i, :] = V_rand
                    break

        plt.scatter(V[:, 0], V[:, 1], marker='.', c='red', label='Sampled Points')

        # Construct the graph
        # Compute the connection radius
        d = 2  # dimension
        zeta_d = np.pi  # 2D = area of the unit circle
        mu_X_free = X * Y - np.sum(occ_map)
        
        gamma_PRM = np.ceil((2 * (1 + 1 / d) ** (1 / d)) * ((mu_X_free / zeta_d) ** (1 / d)))
        r = gamma_PRM * (np.log(n) / n) ** (1 / d)

        print("mu", mu_X_free)
        print("gamma", gamma_PRM)
        print("rad", r)
        
        
        E = []

        # Check for neighbors within the connection radius
        for i in range(n):
            dist = np.linalg.norm(V - V[i, :], axis=1)
            nn_idx = np.argwhere(dist < r)
            nn_idx = nn_idx[nn_idx > i]  # Remove self-connections

            # Check for collisions along potential edges
            disc = int(np.ceil(r))  # Number of discrete checks along the edge
            for j in nn_idx:
                x_check = np.linspace(V[i, 0], V[j, 0], disc)
                y_check = np.linspace(V[i, 1], V[j, 1], disc)
                if in_collision(np.column_stack((x_check, y_check)), occ_map):
                    continue
                else:
                    E.append([i, j, dist[j]])

    
        E = np.array(E).astype(np.int32)

        for i in range(len(E)):
            point_1 = E[i][0]
            point_2 = E[i][1]
            plt.plot([V[point_1][0], V[point_2][0]], [V[point_1][1], V[point_2][1]], 'r-', linewidth=1)
        
    plt.show()


if __name__ == "__main__":
    main()