import logging
import sys
from pathlib import Path

import cv2
from tqdm import tqdm

logger = logging.getLogger(__name__)
sys.path.append("/data/shared/ORB_SLAM3/libs/")
import hypr_slam  # noqa: E402


def save_trajectory(trajectory, output_path):
    """Save trajectory in TUM format (timestamp tx ty tz qx qy qz qw)"""
    with open(output_path, "w") as f:
        poses = trajectory["poses"]
        timestamps = trajectory["timestamps"]
        for i in range(len(timestamps)):
            # Each pose is [tx, ty, tz, qx, qy, qz, qw]
            pose = poses[i]
            timestamp = timestamps[i]
            f.write(
                f"{timestamp:.6f} {pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} {pose[3]:.6f} {pose[4]:.6f} {pose[5]:.6f} {pose[6]:.6f}\n"
            )

    print(f"Saved trajectory to '{output_path}'")


if __name__ == "__main__":
    vocab_path = Path("/data/shared/ORB_SLAM3/ORBvoc.txt")
    settings_path = Path("/data/shared/ORB_SLAM3/hypr.yaml")
    assert vocab_path.exists() and settings_path.exists()

    # Initialize SLAM
    slam = hypr_slam.SLAM(str(vocab_path), str(settings_path))

    # Process images and get frame-by-frame poses
    images_folder = Path("/home/mdeloche/ORB_SLAM3/data/hypr7/rgb/")
    frame_poses = []
    timestamps = []

    # Sort PNG files by timestamp
    image_files = sorted(images_folder.glob("*.png"), key=lambda x: float(x.stem))

    for image_path in tqdm(image_files):
        timestamp = float(image_path.stem)

        image = cv2.imread(str(image_path))
        result = slam.process_image(image, timestamp)

        if result["status"] == "OK":
            frame_poses.append(result["pose"])  # 4x4 pose matrix
            timestamps.append(timestamp)
        else:
            logger.warning(f"Frame {timestamp} tracking lost")

    # Get final optimized trajectory
    final_trajectory = slam.get_trajectory(keyframes_only=False)
    keyframe_timestamps = final_trajectory["timestamps"]
    # Shape: (N, 4, 4) where N is number of keyframes
    keyframe_poses = final_trajectory["poses"]

    print(f"\nProcessed {len(timestamps)} frames")
    print(f"Final trajectory has {len(keyframe_timestamps)} keyframes")

    save_trajectory(final_trajectory, Path("hypr7_trajectory_full.tum"))
