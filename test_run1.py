import open3d as o3d
import numpy as np
import os
import matplotlib.pyplot as plt


colour_image = o3d.io.read_image("images\image000003_left.png")
depth_image = o3d.io.read_image("depth\image000003_disp_raw.png")
print(np.asarray(depth_image))
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(colour_image, depth_image, convert_rgb_to_intensity=False)
print(rgbd_image)

plt.subplot(1, 2, 1)
plt.imshow(rgbd_image.color)
plt.subplot(1, 2, 2)
plt.imshow(rgbd_image.depth)
plt.show()

pcd=o3d.io.read_point_cloud("pcloud\image000003_3d.ply")
o3d.visualization.draw_geometries([rgbd_image, pcd])
