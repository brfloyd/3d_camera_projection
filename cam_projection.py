import open3d as o3d
import numpy as np

mesh_path = "/Users/brettfloyd/arproject/Apr22at8-03 PM-poly/textured.obj"
room_scan = o3d.io.read_triangle_mesh(mesh_path, True)

point_cloud = room_scan.sample_points_poisson_disk(number_of_points=1000)


def create_colored_sphere(radius=0.1, color=[1, 0, 0], center=[0,0,0]):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=20)
    sphere.paint_uniform_color(color)
    sphere.translate(center, relative=False)
    return sphere

intrinsic = o3d.camera.PinholeCameraIntrinsic(
    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
)

def pick_points(point_cloud):
    print("")
    print("Points can be picked by pressing shift + left click")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(point_cloud)
    vis.run()  # user picks points
    vis.destroy_window()
    return vis.get_picked_points()

# A frutsum defines the cone line object that defines the camera POV come projection

picked_points = pick_points(point_cloud)

point = []

point  = np.asarray(point_cloud.points[picked_points[0]])

print(point)



def create_camera_frustum(width, height, fx, fy, cx, cy, depth, extrinsic, points):
    # Create frustum corners in camera space
    color = [1, 0, 0]
    corners = np.array([
        points,  # camera origin
        [(cx / fx) * depth, (cy / fy) * depth, depth],  # top-left
        [(cx / fx) * depth, ((cy - height) / fy) * depth, depth],  # bottom-left
        [((cx - width) / fx) * depth, ((cy - height) / fy) * depth, depth],  # bottom-right
        [((cx - width) / fx) * depth, (cy / fy) * depth, depth]  # top-right
    ])

    # Transform corners to world space
    corners = (extrinsic @ np.hstack((corners, np.ones((5, 1)))).T).T[:, :3]

    # Create lines between corners
    lines = [
        [0, 1], [0, 2], [0, 3], [0, 4],  # from origin to all points
        [1, 2], [2, 3], [3, 4], [4, 1]   # rectangle connecting the corners
    ]
    #Let create the colors for the boundaries of the frustum

    colors = [color for i in range(len(lines))]
    # Create LineSet object
    frustum = o3d.geometry.LineSet()
    frustum.points = o3d.utility.Vector3dVector(corners)
    frustum.lines = o3d.utility.Vector2iVector(lines)
    frustum.colors = o3d.utility.Vector3dVector(colors)
    return frustum

# Define camera parameters
width, height = 640, 480
fx, fy = 525, 525  # Focal length
cx, cy = width / 2, height / 2
depth = 2  # Depth of the frustum in meters

# Example camera pose (identity matrix, i.e., no rotation or translation)
extrinsic = np.eye(4)

# Create frustum
frustum = create_camera_frustum(width, height, fx, fy, cx, cy, depth, extrinsic, point)

# Visualize the mesh
#
red_sphere = create_colored_sphere(0.1, [1, 0, 0], [1,1,3.5])
o3d.visualization.draw_geometries([room_scan, frustum])
