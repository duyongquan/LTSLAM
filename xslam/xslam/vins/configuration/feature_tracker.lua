--- Feature Tracker

FEATURE_TRACKER = {

    -- camera calibration 
    model_type = "PINHOLE",
    camera_name = "camera",
    image_width = 752,
    image_height = 480,

    k1 = -2.917e-01,
    k2 = 8.228e-02,
    p1 = 5.333e-05,
    p2 = -1.578e-04,

    fx = 4.616e+02,
    fy = 4.603e+02,
    cx = 3.630e+02,
    cy = 2.481e+02,

    -- Extrinsic parameter between IMU and Camera.
    estimate_extrinsic = 0,   -- 0  Have an accurate extrinsic parameters. We will trust the following imu^R_cam, imu^T_cam, don't change it.
                              -- 1  Have an initial guess about extrinsic parameters. We will optimize around your initial guess.
                              -- 2  Don't know anything about extrinsic parameters. You don't need to give R,T. We will try to calibrate it. 
                              --    Do some rotation movement at beginning.  
    

    -- unsynchronization parameters
    estimate_td = 0,          -- online estimate time offset between camera and imu
    td = 0.0,                 -- initial value of time offset. unit: s. readed image clock + td = real image clock (IMU clock)

    -- rolling shutter parameters
    rolling_shutter = 0,      -- 0: global shutter camera, 1: rolling shutter camera
    rolling_shutter_tr = 0, 

    -- feature traker paprameters
    max_cnt = 150,                      -- max feature number in feature tracking
    min_dist = 30,                      -- min distance between two features 
    freq = 10,                          -- frequence (Hz) of publish tracking result. At least 10Hz for good estimation. If set 0, the frequence will be same as raw image 
    F_threshold = 1.0,                  -- ransac threshold (pixel)
    show_track = 1,                     -- publish tracking image as topic
    equalize = 1,                       -- if image is too dark or light, trun on equalize to find enough features
    fisheye = 0,                        -- if using fisheye, trun on it. A circle mask will be loaded to remove edge noisy points
}