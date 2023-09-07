import numpy as np
import matplotlib.pyplot as plt
import cv2
import sys
import pyrealsense2 as rs
from segment_anything import sam_model_registry, SamPredictor
import open3d as o3d

# Global variables
points = []
fig = None
continue_feed = True
current_frame = None
depth_image = None  # Store the aligned depth image
aligned_depth_frame = None  # Store the aligned depth frame for point cloud conversion

def on_click(event):
    global points, fig
    if event.inaxes is not None:
        points.append((int(event.xdata), int(event.ydata)))
        if len(points) == 2:
            plt.disconnect(fig.canvas.mpl_connect('button_press_event', on_click))
            apply_sam_on_image(current_frame)
            points = []

def get_continuous_feed():
    global continue_feed, current_frame, depth_image, aligned_depth_frame

    # Create a context object. This object owns the handles to all connected realsense devices
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Create an align object to align depth to color
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Start streaming
    pipeline.start(config)

    global fig
    fig, ax = plt.subplots()

    try:
        while continue_feed:
            frames = pipeline.wait_for_frames()

            # Align the depth frame to the color frame
            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            
            # Set the current frame to the global variable
            current_frame = color_image

            # Display the frame
            ax.imshow(color_image)
            fig.canvas.mpl_connect('button_press_event', on_click)
            plt.draw()
            plt.pause(0.01)
            ax.clear()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop streaming
        pipeline.stop()

def apply_sam_on_image(image):
    global points, continue_feed, depth_image, aligned_depth_frame

    sys.path.append("..")

    sam_checkpoint = "sam_vit_l_0b3195.pth"
    model_type = "vit_l"
    device = "cpu"

    sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
    sam.to(device=device)

    predictor = SamPredictor(sam)
    predictor.set_image(image)

    if len(points) >= 1:
        input_points = np.array(points[:2])
        input_labels = np.array([1, 1])

        masks, scores, logits = predictor.predict(
            point_coords=input_points,
            point_labels=input_labels,
            multimask_output=False,
        )

        mask_input = logits[np.argmax(scores), :, :]

        masks, _, _ = predictor.predict(
            point_coords=input_points,
            point_labels=input_labels,
            mask_input=mask_input[None, :, :],
            multimask_output=False,
        )

        # Convert the depth frame to a point cloud
        pc = rs.pointcloud()
        pointcloud = pc.calculate(aligned_depth_frame)
        vertices = np.asanyarray(pointcloud.get_vertices()).view(np.float32).reshape(-1, 3)  # XYZ coordinates

        # Convert entire vertices to open3d point cloud and save
        entire_pcd = o3d.geometry.PointCloud()
        entire_pcd.points = o3d.utility.Vector3dVector(vertices)
        o3d.io.write_point_cloud("entire_scene.pcd", entire_pcd)

        # Use the mask to filter the vertices
        object_vertices = vertices[masks[0].flatten().astype(bool)]

        # Convert object_vertices to open3d point cloud, visualize, and save
        object_pcd = o3d.geometry.PointCloud()
        object_pcd.points = o3d.utility.Vector3dVector(object_vertices)
        o3d.visualization.draw_geometries([object_pcd])
        o3d.io.write_point_cloud("masked_object.pcd", object_pcd)

        # Display the mask on the image
        fig, ax = plt.subplots()
        ax.imshow(image)
        ax.imshow(masks[0], alpha=0.5)
        plt.show()

        continue_feed = False

if __name__ == "__main__":
    get_continuous_feed()
