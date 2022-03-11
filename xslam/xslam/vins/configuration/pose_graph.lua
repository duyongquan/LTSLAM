include "sensor.lua"

POSE_GRAPH = {
    -- sensor
    sensor = SENSOR,

    -- feature traker paprameters
    feature_tracker = {
        max_cnt = 150,                      -- max feature number in feature tracking
        min_dist = 30,                      -- min distance between two features 
        freq = 10,                          -- frequence (Hz) of publish tracking result. At least 10Hz for good estimation. If set 0, the frequence will be same as raw image 
        F_threshold = 1.0,                  -- ransac threshold (pixel)
        show_track = 1,                     -- publish tracking image as topic
        equalize = 1,                       -- if image is too dark or light, trun on equalize to find enough features
        fisheye = 0,                        -- if using fisheye, trun on it. A circle mask will be loaded to remove edge noisy points
    },

    -- optimization parameters
    optimization = {
        max_solver_time = 0.04,             -- max solver itration time (ms), to guarantee real time
        max_num_iterations = 8,             -- max solver itrations, to guarantee real time
        keyframe_parallax = 10.0,           -- keyframe selection threshold (pixel)
    },

    -- loop closure parameters
    loop_closure = {
        loop_closure = 1,                   -- start loop closure
        load_previous_pose_graph = 0,       -- load and reuse previous pose graph; load from 'pose_graph_save_path'
        fast_relocalization = 0,            -- useful in real-time and large project
        -- pose_graph_save_path: "/home/shaozu/output/pose_graph/" # save and load path
    },
}
