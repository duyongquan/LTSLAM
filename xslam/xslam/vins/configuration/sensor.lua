SENSOR = {
    -- imu parameters       The more accurate parameters you provide, the better performance
    imu = {
        acc_n = 0.08,          -- accelerometer measurement noise standard deviation. #0.2   0.04
        gyr_n = 0.004,         -- gyroscope measurement noise standard deviation.     #0.05  0.004
        acc_w = 0.00004,       -- accelerometer bias random work noise standard deviation.  #0.02
        gyr_w = 2.0e-6,        -- gyroscope bias random work noise standard deviation.      #4.0e-5
        g_norm = 9.81007,      -- gravity magnitude
    },
   
    camera = {
        -- camera calibration 
        model_type = "PINHOLE",
        camera_name = "camera",
        image_width = 752,
        image_height = 480,

        distortion = {
            k1 = -2.917e-01,
            k2 = 8.228e-02,
            p1 = 5.333e-05,
            p2 = -1.578e-04,
        },

        projection = {
            fx = 4.616e+02,
            fy = 4.603e+02,
            cx = 3.630e+02,
            cy = 2.481e+02,
        },
    
        -- Extrinsic parameter between IMU and Camera.
        estimate_extrinsic = 0,   -- 0  Have an accurate extrinsic parameters. We will trust the following imu^R_cam, imu^T_cam, don't change it.
                                  -- 1  Have an initial guess about extrinsic parameters. We will optimize around your initial guess.
                                  -- 2  Don't know anything about extrinsic parameters. You don't need to give R,T. We will try to calibrate it. 
                                  --    Do some rotation movement at beginning.  
    }, 
    
    -- unsynchronization parameters
    unsynchronization = {
        estimate_td = 0,          -- online estimate time offset between camera and imu
        td = 0.0,                 -- initial value of time offset. unit: s. readed image clock + td = real image clock (IMU clock)
    },
    
    -- rolling shutter parameters
    rolling_shutter = {
        rolling_shutter = 0,      -- 0: global shutter camera, 1: rolling shutter camera
        rolling_shutter_tr = 0, 
    }
}