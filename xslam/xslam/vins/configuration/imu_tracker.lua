--- IMU Tracker

IMU_TRACKER = {

    -- imu parameters       The more accurate parameters you provide, the better performance
    acc_n = 0.08,          -- accelerometer measurement noise standard deviation.       #0.2   0.04
    gyr_n = 0.004,         -- gyroscope measurement noise standard deviation.           #0.05  0.004
    acc_w = 0.00004,       -- accelerometer bias random work noise standard deviation.  #0.02
    gyr_w = 2.0e-6,        -- gyroscope bias random work noise standard deviation.      #4.0e-5
    g_norm = 9.81007,      -- gravity magnitude
}