import numpy as np
# import matplotlib.pyplot as plt
import open3d as o3d


class KeyFrame():
    def __init__(self):
        self.pointcloud = None

    def from_file(self, filename):
        self.pointcloud = o3d.io.read_point_cloud(filename, print_progress=True)

    def from_points(self, points):
        self.pointcloud = o3d.geometry.PointCloud()
        self.pointcloud.points = o3d.utility.Vector3dVector(points)

    def save_pointcloud(self, output_filename):
        o3d.io.write_point_cloud(output_filename, self.pointcloud)

    def draw_cloud(self):
        o3d.visualization.draw_geometries([self.pointcloud],
                                          zoom=0.3412,
                                          front=[0.4257, -0.2125, -0.8795],
                                          lookat=[2.6172, 2.0475, 1.532],
                                          up=[-0.0694, -0.9768, 0.2024])

    def downwample(self, voxel_size):
        self.pointcloud = self.pointcloud.voxel_down_sample(voxel_size=voxel_size)

    def transform(self, T):
        # pointcloud = self.pointcloud.uniform_down_sample(every_k_points=point_cloud_sampling)
        self.pointcloud = self.pointcloud.transform(T)

    def compute_traversability(self, Z=0.2):
        points = np.asarray(self.pointcloud.points)
        # filter
        idx_traversable = points[:, 2] <= Z
        idx_obstacle = points[:, 2] > Z
        return points[idx_traversable, :], points[idx_obstacle, :]

















