% #feature traker paprameters
% max_cnt: 150            # max feature number in feature tracking
% min_dist: 30            # min distance between two features 
% freq: 10                # frequence (Hz) of publish tracking result. At least 10Hz for good estimation. If set 0, the frequence will be same as raw image 
% F_threshold: 1.0        # ransac threshold (pixel)
% show_track: 1           # publish tracking image as topic
% equalize: 1             # if image is too dark or light, trun on equalize to find enough features
% fisheye: 0              # if using fisheye, trun on it. A circle mask will be loaded to remove edge noisy points
% 
% #optimization parameters
% max_solver_time: 0.04  # max solver itration time (ms), to guarantee real time
% max_num_iterations: 8   # max solver itrations, to guarantee real time
% keyframe_parallax: 10.0 # keyframe selection threshold (pixel)
% 
% #imu parameters       The more accurate parameters you provide, the better performance
% acc_n: 0.08          # accelerometer measurement noise standard deviation. #0.2   0.04
% gyr_n: 0.004         # gyroscope measurement noise standard deviation.     #0.05  0.004
% acc_w: 0.00004         # accelerometer bias random work noise standard deviation.  #0.02
% gyr_w: 2.0e-6       # gyroscope bias random work noise standard deviation.     #4.0e-5
% g_norm: 9.81007     # gravity magnitude
global ACC_N
global GYR_N
global ACC_W
global GYR_W
ACC_N=0.08;
GYR_N=0.004;
ACC_W=0.00004;
GYR_W=2.0e-6;

% #loop closure parameters
% loop_closure: 1                    # start loop closure
% load_previous_pose_graph: 0        # load and reuse previous pose graph; load from 'pose_graph_save_path'
% fast_relocalization: 0             # useful in real-time and large project
% pose_graph_save_path: "/home/shaozu/output/pose_graph/" # save and load path
% 
% #unsynchronization parameters
% estimate_td: 0                      # online estimate time offset between camera and imu
% td: 0.0                             # initial value of time offset. unit: s. readed image clock + td = real image clock (IMU clock)
% 
% #rolling shutter parameters
% rolling_shutter: 0                  # 0: global shutter camera, 1: rolling shutter camera
% rolling_shutter_tr: 0               # unit: s. rolling shutter read out time per frame (from data sheet). 
% 
% #visualization parameters
% save_image: 1                   # save image in pose graph for visualization prupose; you can close this function by setting 0 
% visualize_imu_forward: 0        # output imu forward propogation to achieve low latency and high frequence results
% visualize_camera_size: 0.4      # size of camera marker in RVIZ
