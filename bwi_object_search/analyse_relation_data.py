import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def main():
    target_id = 12

    id_to_loc_dict = {}

    for i in range(16):
        id_to_loc_dict[i] =  [[], [], []]


    with open('relation_data.txt') as f:
        for line in f:
            fields = line.split(" ")
            time = float(fields[0])
            id = int(fields[1])
            x =  float(fields[2])
            y =  float(fields[3])
            z =  float(fields[4])
            # print("{} {} {} {} {}".format(float(fields[0]), int(fields[1]), float(fields[2]), float(fields[3]), float(fields[4])))

            id_to_loc_dict[id][0].append(x)
            id_to_loc_dict[id][1].append(y)
            id_to_loc_dict[id][2].append(z)


    # Normalization
    for id, loc_list in id_to_loc_dict.iteritems():
        print("ID: {}".format(id))
        for axis_list in loc_list:
            if (len(axis_list) == 0):
                continue
            mean = sum(axis_list) / len(axis_list)
            axis_list[:] = [elem - mean for elem in axis_list]
            print(np.var(axis_list))

    # Plot
    fig = plt.figure(num=None, figsize=(8*2, 6*2))

    subplot_index = 1

    for id, loc_list in id_to_loc_dict.iteritems():
        # ax = fig.add_subplot(4, 4, subplot_index, projection='3d')
        ax = fig.add_subplot(4, 4, subplot_index)
        subplot_index += 1

        ax.set_xlabel('x (var = {0:.2f})'.format(np.var(id_to_loc_dict[id][0])))
        ax.set_ylabel('y (var = {0:.2f})'.format(np.var(id_to_loc_dict[id][1])))
        ax.set_title('id : {}'.format(id))
        # ax.set_zlabel('Z Label')

        ax.scatter(id_to_loc_dict[id][0], id_to_loc_dict[id][1])
        # ax.scatter(id_to_loc_dict[id][0], id_to_loc_dict[id][1], zs=0, zdir='z', label='zs=0, zdir=z')
        # ax.view_init(90, -90)

    fig.tight_layout()
    plt.show()





if __name__ == "__main__":
    main()
