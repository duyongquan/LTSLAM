-- Copyright Authors

-- include "map_builder.lua"

options = {
  map_builder = {
                  use_trajectory_builder_2d = false,
                  collate_by_trajectory = false,
              },

  map_frame = "map",
  use_landmarks = false,
  num_laser_scans = 1,
  pose_publish_period_sec = 5e-3,
}

return options
