
import numpy as np
import open3d as o3d
import csv
import sys
import networkx as nx
from tqdm import tqdm
import matplotlib.pyplot as plt
import time
import os
csv.field_size_limit(sys.maxsize)

if __name__ == '__main__':

    # Function to get file names in a folder
    def get_file_names(folder_path):
        file_names = []
        for filename in os.listdir(folder_path):
            if os.path.isfile(os.path.join(folder_path, filename)):
                file_names.append(folder_path + '/' + filename)
            if len(file_names) > 19900:
                break
        # Combine file names from both folders into a single list
        # all_files = folder1_files + folder2_files
        return file_names


    filelist=get_file_names('/home/sensorbox/data/opv2v/train/additional/2021_08_16_22_26_54/-42')
    with tqdm(filelist) as pbar:
        pc = o3d.geometry.PointCloud()

        for file in filelist:    
            array_radar = np.load(file)
            pc_radar = o3d.geometry.PointCloud()
            # array_radar = array_radar[abs(array_radar[:,1]) > 0.01]

            x = array_radar[:,0] * np.cos(array_radar[:,3]) * np.cos(array_radar[:,2])
            y = -array_radar[:,0] * np.cos(array_radar[:,3]) * np.sin(array_radar[:,2])
            z = array_radar[:,0] * np.sin(array_radar[:,3])
            np_radar=np.vstack((x,y,z))
            pc_radar.points = o3d.utility.Vector3dVector(np_radar.T)
            pc_radar.paint_uniform_color([0,0,0.5])
            pc = pc + pc_radar
            pbar.update(1)
    o3d.visualization.draw_geometries([pc])