import open3d as o3d
import numpy as np
import os
import matplotlib.pyplot as plt


colour_image = o3d.io.read_image("Run 2\images\image000000_left.png")
depth_image = o3d.io.read_image("Run 2\depth\image000000_disp_raw.png")
print(np.asarray(depth_image))
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(colour_image, depth_image, convert_rgb_to_intensity=False)
print(rgbd_image)

plt.subplot(1, 2, 1)
plt.imshow(rgbd_image.color)
plt.subplot(1, 2, 2)
plt.imshow(rgbd_image.depth)
plt.show()

pcd=o3d.io.read_point_cloud("Run 2\pcloud\image000001_3d.ply")
#o3d.visualization.draw_geometries([rgbd_image])

cam_prop = o3d.camera.PinholeCameraIntrinsic()
cam_prop.height = 768
cam_prop.width = 1024
cam_prop.intrinsic_matrix = [[872.76442, 0, -766.876], [0, 872.76442, -574.1995], [0, 0, 1]]
pcd_comp = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, cam_prop)
o3d.visualization.draw_geometries([pcd])
