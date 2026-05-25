from humanoid_navigation.depth_image_to_pointcloud import (
    DepthProjectionConfig,
    camera_info_intrinsics,
    project_depth_to_points,
)


def test_project_depth_to_points_uses_camera_intrinsics_and_stride():
    depth_m = [
        1.0, 2.0, 3.0, 4.0,
        5.0, 6.0, 7.0, 8.0,
        9.0, 10.0, 11.0, 12.0,
        13.0, 14.0, 15.0, 16.0,
    ]
    intrinsics = camera_info_intrinsics(
        width=4,
        height=4,
        k=[2.0, 0.0, 1.0, 0.0, 4.0, 1.0, 0.0, 0.0, 1.0],
    )

    points = project_depth_to_points(
        depth_m,
        intrinsics,
        DepthProjectionConfig(stride=2, min_depth_m=0.5, max_depth_m=20.0),
    )

    assert points == [
        (-0.5, -0.25, 1.0),
        (1.5, -0.75, 3.0),
        (-4.5, 2.25, 9.0),
        (5.5, 2.75, 11.0),
    ]


def test_project_depth_to_points_filters_invalid_and_out_of_range_depth():
    depth_m = [
        0.0,
        float("nan"),
        0.49,
        0.5,
        float("inf"),
        2.0,
        2.1,
        1.0,
        1.5,
    ]
    intrinsics = camera_info_intrinsics(
        width=3,
        height=3,
        k=[1.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0],
    )

    points = project_depth_to_points(
        depth_m,
        intrinsics,
        DepthProjectionConfig(stride=1, min_depth_m=0.5, max_depth_m=2.0),
    )

    assert points == [
        (-0.5, 0.0, 0.5),
        (2.0, 0.0, 2.0),
        (0.0, 1.0, 1.0),
        (1.5, 1.5, 1.5),
    ]
