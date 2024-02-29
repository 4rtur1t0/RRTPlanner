import numpy as np
# import matplotlib.pyplot as plt
import open3d as o3d


class KeyFrame():
    def __init__(self, filename, voxel_size=0.2):
        # self.transform = None
        # voxel sizes
        # self.voxel_size = voxel_size
        # self.voxel_size_normals = 5*self.voxel_size
        # self.voxel_size_fpfh = 5*self.voxel_size
        # self.icp_threshold = 5
        # self.fpfh_threshold = 5
        # filename = directory + '/robot0/lidar/data/' + str(scan_time) + '.pcd'

        self.pointcloud = o3d.io.read_point_cloud(filename, print_progress=True)
        # downsample pointcloud and save to pointcloud in keyframe
        # self.pointcloud = self.pointcloud.voxel_down_sample(voxel_size=self.voxel_size)


        # calcular las normales a cada punto
        # self.pointcloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size_normals,
        #                                                                       max_nn=30))

        # extraer los Fast Point Feature Histograms
        # self.pointcloud_fpfh = o3d.pipelines.registration.compute_fpfh_feature(self.pointcloud,
        #                                                                        o3d.geometry.KDTreeSearchParamHybrid(
        #                                                                            radius=self.voxel_size_fpfh,
        #                                                                            max_nn=100))


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

















