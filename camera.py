import open3d as o3d
import numpy as np

def create_camera_parameters(fov_deg=60, width=640, height=480, z_near=0.01, z_far=1000.0):
    # Create camera intrinsic parameters based on fov and image dimensions
    fov_rad = np.deg2rad(fov_deg)
    fx = fy = width / (2 * np.tan(fov_rad / 2))
    cx = width / 2
    cy = height / 2
    intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    return intrinsic

def create_trajectory(intrinsic, num_poses=100):
    # Create a trajectory with camera moving along the Z-axis
    trajectory = o3d.camera.PinholeCameraTrajectory()
    trajectory.parameters = []

    for z in np.linspace(-2, 2, num_poses):
        # Extrinsic matrix (camera pose)
        extrinsic = np.eye(4)
        extrinsic[0:3, 3] = [0, 0, z]
        camera_params = o3d.camera.PinholeCameraParameters()
        camera_params.intrinsic = intrinsic
        camera_params.extrinsic = extrinsic
        trajectory.parameters.append(camera_params)

    return trajectory

# Load your 3D model
model_path = "/Users/brettfloyd/arproject/Apr22at8-03 PM-poly/textured.obj"  # Update with the path to your model
model = o3d.io.read_triangle_mesh(model_path)
model.compute_vertex_normals()  # Optionally compute normals if not present

# Camera intrinsic parameters
intrinsic = create_camera_parameters()

# Create a camera trajectory
trajectory = create_trajectory(intrinsic)

# Visualization
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(model)  # Add your model to the visualization
for param in trajectory.parameters:
    ctr = vis.get_view_control()
    ctr.convert_from_pinhole_camera_parameters(param)
    vis.poll_events()
    vis.update_renderer()
vis.run()
vis.destroy_window()
