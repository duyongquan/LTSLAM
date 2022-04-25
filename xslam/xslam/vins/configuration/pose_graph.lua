--- pose graph

POSE_GRAPH = {
    -- optimization parameters
    max_solver_time = 0.04,             -- max solver itration time (ms), to guarantee real time
    max_num_iterations = 8,             -- max solver itrations, to guarantee real time
    keyframe_parallax = 10.0,           -- keyframe selection threshold (pixel)

    -- loop closure parameters
    loop_closure = 1,                   -- start loop closure
    load_previous_pose_graph = 0,       -- load and reuse previous pose graph; load from 'pose_graph_save_path'
    fast_relocalization = 0,            -- useful in real-time and large project
    -- pose_graph_save_path: "/home/shaozu/output/pose_graph/" # save and load path
}
