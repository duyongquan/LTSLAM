include "vins.lua"

options = {
  vins_options = VINSOptions,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  lookup_transform_timeout_sec = 0.2,
  pose_publish_period_sec = 5e-3,
  publish_to_tf = true,
  publish_tracked_pose = false,
}

return options