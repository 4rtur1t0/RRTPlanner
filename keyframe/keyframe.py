import numpy as np
# import matplotlib.pyplot as plt
import open3d as o3d

class KeyFrame():
    def __init__(self, filename):
        self.transform = None
        # voxel sizes
        self.voxel_size = 0.05
        self.voxel_size_normals = 5*self.voxel_size
        self.voxel_size_fpfh = 5*self.voxel_size
        self.icp_threshold = 5
        self.fpfh_threshold = 5
        # filename = directory + '/robot0/lidar/data/' + str(scan_time) + '.pcd'

        self.pointcloud = o3d.io.read_point_cloud(filename, print_progress=True)
        # downsample pointcloud and save to pointcloud in keyframe
        self.pointcloud = self.pointcloud.voxel_down_sample(voxel_size=self.voxel_size)
        # calcular las normales a cada punto
        self.pointcloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size_normals,
                                                                              max_nn=30))
        # extraer los Fast Point Feature Histograms
        self.pointcloud_fpfh = o3d.pipelines.registration.compute_fpfh_feature(self.pointcloud,
                                                                               o3d.geometry.KDTreeSearchParamHybrid(
                                                                                   radius=self.voxel_size_fpfh,
                                                                                   max_nn=100))
        # self.draw_cloud()

    # def read_keyframe(self, filename):
    #     """
    #          Reads keyframe with index i from disk
    #     """
    #     kf = KeyFrame(directory=self.directory, scan_time=self.scan_times[index])
    #     return kf

    # def local_registration(self, other, initial_transform):
    #     """
    #     use icp to compute transformation using an initial estimate.
    #     caution, initial_transform is a np array.
    #     """
    #     print("Apply point-to-plane ICP")
    #     # print("Using threshold: ", self.icp_threshold)
    #     # reg_p2p = o3d.pipelines.registration.registration_icp(
    #     #         other.pointcloud, self.pointcloud, self.icp_threshold, initial_transform,
    #     #         o3d.pipelines.registration.TransformationEstimationPointToPoint())
    #     reg_p2p = o3d.pipelines.registration.registration_icp(
    #         other.pointcloud, self.pointcloud, self.icp_threshold, initial_transform,
    #         o3d.pipelines.registration.TransformationEstimationPointToPlane())
    #     print(reg_p2p)
    #     # print("Transformation is:")
    #     # print(reg_p2p.transformation)
    #     print("")
    #     # other.draw_registration_result(self, reg_p2p.transformation)
    #     return reg_p2p

    # def global_registration(self, other):
    #     """
    #     perform global registration followed by icp
    #     """
    #     initial_transform = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
    #         other.pointcloud, self.pointcloud, other.pointcloud_fpfh, self.pointcloud_fpfh,
    #         o3d.pipelines.registration.FastGlobalRegistrationOption(maximum_correspondence_distance=self.fpfh_threshold))
    #     # other.draw_registration_result(self, initial_transform.transformation)
    #
    #     reg_p2p = o3d.pipelines.registration.registration_icp(
    #         other.pointcloud, self.pointcloud, self.icp_threshold, initial_transform.transformation,
    #         o3d.pipelines.registration.TransformationEstimationPointToPoint())
    #     # other.draw_registration_result(self, reg_p2p.transformation)
    #     print(reg_p2p)
    #     print("Refined transformation is:")
    #     print(reg_p2p.transformation)
    #     return reg_p2p.transformation

    # def draw_registration_result(self, other, transformation):
    #     source_temp = copy.deepcopy(self.pointcloud)
    #     target_temp = copy.deepcopy(other.pointcloud)
    #     source_temp.paint_uniform_color([1, 0, 0])
    #     target_temp.paint_uniform_color([0, 0, 1])
    #     source_temp.transform(transformation)
    #     o3d.visualization.draw_geometries([source_temp, target_temp],
    #                                       zoom=0.4459,
    #                                       front=[0.9288, -0.2951, -0.2242],
    #                                       lookat=[1.6784, 2.0612, 1.4451],
    #                                       up=[-0.3402, -0.9189, -0.1996])

    def draw_cloud(self):
        o3d.visualization.draw_geometries([self.pointcloud],
                                          zoom=0.3412,
                                          front=[0.4257, -0.2125, -0.8795],
                                          lookat=[2.6172, 2.0475, 1.532],
                                          up=[-0.0694, -0.9768, 0.2024])

    def compute_traversability(self, Z=-0.3):
        points = np.asarray(self.pointcloud.points)
        # filter
        idx_traversable = points[:, 2] <= Z
        idx_obstacle = points[:, 2] > Z
        return points[idx_traversable, :], points[idx_obstacle, :]

    # def set_global_transform(self, transform):
    #     self.transform = transform
    #     return

    # def transform_to_global(self, point_cloud_sampling=10):
    #     """
    #         Use open3d to fast transform to global coordinates.
    #         Returns the pointcloud in global coordinates
    #     """
    #     T = HomogeneousMatrix(self.transform)
    #     pointcloud = self.pointcloud.uniform_down_sample(every_k_points=point_cloud_sampling)
    #     return pointcloud.transform(T.array)













