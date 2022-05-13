include "imu_tracker.lua"
include "feature_tracker.lua"
include "pose_graph.lua"

VINSOptions = {
    thread_nums = 4,
    imu_tracker = IMU_TRACKER,
    feature_tracker = FEATURE_TRACKER,
    pose_graph = POSE_GRAPH,
}
