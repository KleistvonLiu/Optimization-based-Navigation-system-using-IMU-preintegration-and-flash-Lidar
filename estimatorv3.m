classdef estimatorv3 < handle
    properties
        
        %         acc_1;
        %         gyr_1;
        %         %!!should be const
        %         linearized_acc; %acc value when initializing the class
        %         linearized_gyr;
        %         linearized_ba;
        %         linearized_bg;
        %
        %         jacobian;%!!should have initial size
        %         covariance;
        %         step_jacobian;
        %         step_V;
        %         noise;
        
        sum_dt;
        delta_p;
        delta_q;
        delta_v;
        
        dt_buf;%存储所有的imu数据，目的：当bias很大的时候 需要repropagate
        acc_buf;
        gyr_buf;
        
        dt;
        acc_0;
        gyr_0;
        acc_L_0;
        gyr_B_0;
        previous_a
        initial; %for processIMU, if initial data or not
        frame_count; % number of key frames, from 1 to window_size+1
        new_frame; % if new frame,Pnew initial value = Pold end value
        Ps;
        Vs;
        Rs;
        Qs;
        Bas;
        Bgs;
        window_size;
        pre_integrations;
        first_imu;
        g_sum;  
        
        % container for point cloud related data
        deltaT2last;%results of icp to last pc,7*10,the first value is between X1 and X2
        deltaT2base;%results of icp to base pc,7*1
        first_scan;    
        
        % container for parameters that needs to be sent to mex files
        dpContainer;%3*10
        dqContainer;%4*10
        dqContainerArray;%4*10
        dvContainer;%3*10
        JContainer;%15*15*10
        PContainer;%15*15*10
        baContainer;%3*10
        bgContainer;%3*10
        dtContainer;
        
        count;% just for test processPC
    end
    methods
        %!these 4 variables should be references
        function obj = estimatorv3(window_size, X_init)
            if nargin >= 1
                obj.window_size = window_size;
            else
                obj.window_size = 10;
            end
            obj.Ps = zeros(3,obj.window_size+1);
            obj.Ps(:,1) = X_init(1:3);
            obj.Vs = zeros(3,obj.window_size+1);
            obj.Vs(:,1) = X_init(4:6);
            %obj.Qs = quaternion(rand(obj.window_size+1,4));
            obj.Qs = zeros(4,obj.window_size+1);
            obj.Qs(:,1) = X_init(7:10);
            obj.Rs = repmat(eye(3),1,1,obj.window_size+1);
            obj.Rs(:,:,1) = quat2rotmliub(obj.Qs(:,1)).';
            
            obj.Bas = zeros(3,obj.window_size+1);
            obj.Bgs = zeros(3,obj.window_size+1);
            %g_sum(:,i+1) 是Ps(:,i)和Ps(:,i+1)之间的重力积分和，所以第一个值不使用
            obj.g_sum = zeros(3,obj.window_size+1);
            obj.first_imu = false;
            obj.dt_buf = [];
            obj.acc_buf = [];
            obj.gyr_buf = [];
            obj.count = 1;
            obj.frame_count = 1;
            obj.new_frame = 0;
            obj.pre_integrations = [];
            
            % obj.pc = ;
            obj.deltaT2last = zeros(7,obj.window_size);%results of icp to last pc,7*11,the first value will not be used
            obj.deltaT2base = zeros(7,1);%results of icp to base pc,7*1
            obj.first_scan = 1;
            
            obj.dpContainer = zeros(3,obj.window_size+1);%3*10
            %obj.dqContainer = quaternion(zeros(obj.window_size,4));%10*4
            obj.dqContainerArray = zeros(4,obj.window_size);%4*10
            obj.dvContainer = zeros(3,obj.window_size);%3*10
            obj.JContainer = zeros(15,15,obj.window_size);%15*15*10
            obj.PContainer = zeros(15,15,obj.window_size);%15*15*10
            obj.baContainer = zeros(3,obj.window_size);%3*10
            obj.bgContainer = zeros(3,obj.window_size);%3*10
            obj.dtContainer = zeros(1,obj.window_size);%3*10
            %             obj.Ps = initial_acc_0;
            %             obj.gyr_0 = initial_gyr_0;
            %             obj.linearized_acc = initial_acc_0;
            %             obj.linearized_gyr = initial_gyr_0;
            %             obj.linearized_ba = initial_linearized_ba;
            %             obj.linearized_bg = initial_linearized_bg;
            %             obj.jacobian = eye(15);
            %             obj.covariance = zeros(15,15);
            %             obj.sum_dt = 0;%默认为double
            %             obj.delta_p = zeros(3,1);
            %             obj.delta_q = quaternion(1,0,0,0);%四元数的表达方式
            %             obj.delta_v = zeros(3,1);
            %             obj.noise = zeros(18,18);
            %             setNoise(obj);
            
        end
        
        function processIMU(obj,dt,linear_acceleration,angular_velocity,gravity_T)
            global posi_TL_T
            global R_LT;
            global angRate_IT_T
            global angRate_IL_L;
            % calculate the effective gravitation
            effGrav_T = gravity_T - cross(angRate_IT_T,cross(angRate_IT_T, posi_TL_T + R_LT* obj.Ps(:,obj.frame_count)));
            effGrav_L = R_LT' * effGrav_T;
            
            %1.初始化，如果当前帧不是第一帧IMU，那么就把它看成第一个IMU，而且把他的值取出来作为初始值
            if (~obj.first_imu)
                obj.first_imu = true;
                obj.acc_0 = linear_acceleration;
                obj.gyr_0 = angular_velocity - obj.Rs(:,:,obj.frame_count).' * angRate_IL_L;
                current_a = obj.Rs(:,:,obj.frame_count) * (linear_acceleration - obj.Bas(:,obj.frame_count)) + effGrav_L - 2 * cross(angRate_IL_L, obj.Vs(:,obj.frame_count));
                % obj.gyr_B_0 = angular_velocity - obj.Rs(:,:,obj.frame_count).' * angRate_IL_L;
                % obj.previous_a_sup = effGrav_L - 2 * cross(angRate_IL_L, obj.Vs(:,obj.frame_count));
            end
            % 注意此处是使用上一个历元的IMU数据!因此first_imu的设置还是有作用的
            if (length(obj.pre_integrations)<(obj.frame_count))
                obj.pre_integrations = [obj.pre_integrations IntegrationBasev2(obj.acc_0, obj.gyr_0, obj.Bas(obj.frame_count), obj.Bgs(obj.frame_count))];
            end
            
            current_g = angular_velocity - obj.Rs(:,:,obj.frame_count).' * angRate_IL_L;
            % current_a = obj.Rs(:,:,obj.frame_count) * (linear_acceleration - obj.Bas(:,obj.frame_count)) + effGrav_L - 2 * cross(angRate_IL_L, obj.Vs(:,obj.frame_count));
            % current_a_sup = effGrav_L - 2 * cross(angRate_IL_L, obj.Vs(:,obj.frame_count));
            if (obj.frame_count ~= 1)
                
                obj.pre_integrations(obj.frame_count).push_back(dt, linear_acceleration, current_g);
                %if(solver_flag != NON_LINEAR)
                %    tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);
                
                obj.dt_buf=[obj.dt_buf,dt];
                obj.acc_buf=[obj.acc_buf,linear_acceleration];
                obj.gyr_buf=[obj.gyr_buf,current_g];
                
                j = obj.frame_count;
                
                % if new frame,Pnew initial value = Pold end value
%                 if (obj.new_frame == 1)
%                     obj.Rs(:,:,j) = obj.Rs(:,:,j-1);
%                     obj.Ps(:,j) =obj.Ps(:,j-1);
%                     obj.Vs(:,j) =obj.Vs(:,j-1);
%                     obj.new_frame = 0;
%                 end 
                
                obj.g_sum(:,j) = obj.g_sum(:,j) + 0.5 * dt * dt * effGrav_L;
                % angRate_IB_B - obj.Rs(:,:,j).' * angRate_IL_L;
                % un_acc_0 = obj.Rs(:,:,j) * (obj.acc_0 - obj.Bas(:,j)) + obj.previous_a_sup;% 这里都应该是上一时刻的数据
                un_gyr = 0.5 * (obj.gyr_0 + current_g) - obj.Bgs(:,j);% !! Bgs(j)
                un_gyr = un_gyr * dt;
                obj.Rs(:,:,j) =obj.Rs(:,:,j)*quat2rotmliub([un_gyr(1)/2,un_gyr(2)/2,un_gyr(3)/2,1]).';
                current_a = obj.Rs(:,:,j) * (linear_acceleration - obj.Bas(:,j)) + effGrav_L - 2 * cross(angRate_IL_L, obj.Vs(:,obj.frame_count));% 这里都应该是这一时刻的数据
                un_acc = 0.5 * (obj.previous_a + current_a);
                obj.Ps(:,j) =obj.Ps(:,j) + dt * obj.Vs(:,j) + 0.5 * dt * dt * un_acc;
                obj.Vs(:,j) =obj.Vs(:,j) + dt * un_acc;
            end
            obj.acc_0 = linear_acceleration;%应该是用来初始化IntegrationBase，每来一次数据都更新但是只有framecount+1的时候才用到
            obj.gyr_0 = current_g; % 将这一时刻的数据存下来给下一时刻用
            % previous_g = current_g;
            % obj.previous_a_sup = current_a_sup; % 将这一时刻的数据存下来给下一时刻用
            obj.previous_a = current_a;
        end
        
        function testProcessIMU(obj,dt,linear_acceleration,angular_velocity)
            %1.初始化，如果当前帧不是第一帧IMU，那么就把它看成第一个IMU，而且把他的值取出来作为初始值
            if (~obj.first_imu)
                obj.first_imu = true;
                obj.acc_0 = linear_acceleration;
                obj.gyr_0 = angular_velocity;
            end
            % 注意此处是使用上一个历元的IMU数据!因此first_imu的设置还是有作用的
            if (length(obj.pre_integrations)<(obj.frame_count))
                obj.pre_integrations = [obj.pre_integrations IntegrationBasev2(obj.acc_0, obj.gyr_0, obj.Bas(obj.frame_count), obj.Bgs(obj.frame_count))];
            end
            
            if (obj.frame_count ~= 1)
                
                obj.pre_integrations(obj.frame_count).push_back(dt, linear_acceleration, angular_velocity);
                %if(solver_flag != NON_LINEAR)
                %    tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);
                
                obj.dt_buf=[obj.dt_buf,dt];
                obj.acc_buf=[obj.acc_buf,linear_acceleration];
                obj.gyr_buf=[obj.gyr_buf,angular_velocity];
                
                j = obj.frame_count;
                global g
                un_acc_0 = obj.Rs(:,:,j) * (obj.acc_0 - obj.Bas(:,j)) - g;
                un_gyr = 0.5 * (obj.gyr_0 + angular_velocity) - obj.Bgs(:,j);% !! Bgs(j)
                un_gyr = un_gyr * dt;
                obj.Rs(:,:,j) =obj.Rs(:,:,j)*quat2rotmliub([un_gyr(1)/2,un_gyr(2)/2,un_gyr(3)/2,1]).';
                un_acc_1 = obj.Rs(:,:,j) * (linear_acceleration - obj.Bas(:,j)) - g;% !! Bas(j)
                un_acc = 0.5 * (un_acc_0 + un_acc_1);
                obj.Ps(:,j) =obj.Ps(:,j)+ dt * obj.Vs(:,j) + 0.5 * dt * dt * un_acc;
                obj.Vs(:,j) =obj.Vs(:,j)+ dt * un_acc;
            end
            obj.acc_0 = linear_acceleration;%应该是用来初始化IntegrationBase，每来一次数据都更新但是只有framecount+1的时候才用到
            obj.gyr_0 = angular_velocity;
        end
        
        function processPC(obj,pc,params)
            process(pc)
            obj.pc(obj.frame_count) = pc;
            if obj.frame_count ~= 1
                [transform, ~] = pcicpFL_LJF_V2(obj.pc(obj.frame_count), obj.pc(obj.frame_count-1), params);%movingPc, fixedPc, params
                obj.Timu(obj.frame_count) = transform;%第2-11个值是有用的 
            end
            
            if obj.frame_count == obj.window_size + 1
                optimization(obj)
                slide_window(obj,1,2,3);
            else
                obj.frame_count = obj.frame_count + 1;
                obj.new_frame = 1;
                obj.Rs(:,:,obj.frame_count) = obj.Rs(:,:,obj.frame_count-1);
                obj.Ps(:,obj.frame_count) =obj.Ps(:,obj.frame_count-1);
                obj.Vs(:,obj.frame_count) =obj.Vs(:,obj.frame_count-1);
            end
        end
        
        function testProcessPC(obj,flag,c,deltaX_icp2base, deltaX_icp2last)
            %% 只测试连续的imu积分
            if(flag == 1)
                obj.frame_count = 2; 
                obj.Rs(:,:,obj.frame_count) = obj.Rs(:,:,obj.frame_count-1);
                obj.Ps(:,obj.frame_count) =obj.Ps(:,obj.frame_count-1);
                obj.Vs(:,obj.frame_count) =obj.Vs(:,obj.frame_count-1);
                obj.new_frame = 1;
                if obj.count == c
                    obj.count = 1;
                    % obj.frame_count = obj.frame_count + 1;
                end
            end
            %% 测试10个frame的imu积分
            if(flag == 2)
                if obj.frame_count == obj.window_size + 1
                    %optimization(obj)
                    slide_window(obj,1,2,3);
                else
                    obj.frame_count = obj.frame_count + 1;
                    obj.Rs(:,:,obj.frame_count) = obj.Rs(:,:,obj.frame_count-1);
                    obj.Ps(:,obj.frame_count) =obj.Ps(:,obj.frame_count-1);
                    obj.Vs(:,obj.frame_count) =obj.Vs(:,obj.frame_count-1);
                    obj.new_frame = 1;
                end
            end
            %% 测试可变窗口以及存储icp，以及优化
            if(flag == 3)
                if (obj.first_scan == 1)
                    obj.first_scan = 0;
                else
                    obj.deltaT2last(:,obj.frame_count-1) = deltaX_icp2last;
                    obj.deltaT2base = deltaX_icp2base;
                    optimization(obj)
                end
                
                if obj.frame_count == obj.window_size + 1
                    slide_window(obj,1,2,3);
                else
                    obj.frame_count = obj.frame_count + 1;
                    obj.Rs(:,:,obj.frame_count) = obj.Rs(:,:,obj.frame_count-1);
                    obj.Ps(:,obj.frame_count) =obj.Ps(:,obj.frame_count-1);
                    obj.Vs(:,obj.frame_count) =obj.Vs(:,obj.frame_count-1);
                    obj.Bas(:,obj.frame_count) =obj.Bas(:,obj.frame_count-1);
                    obj.Bgs(:,obj.frame_count) =obj.Bgs(:,obj.frame_count-1);
                    obj.new_frame = 1;
                end
            end            
            
        end
        
        function optimization(obj)
            %input integrationBase2-11的预积分变量 J and Q also bias，Ps[1-11]
            %Timu 2-11
            obj.Qs = rotm2quatliub(permute(obj.Rs,[2,1,3]));
            obj.Qs = [obj.Qs(4,:);obj.Qs(1:3,:)];
%             obj.Ps
%             obj.Rs
%             obj.Vs
%             obj.Bas
%             obj.Bgs
            for i = 1:obj.frame_count-1
                obj.dpContainer(:,i) = obj.pre_integrations(i+1).delta_p;%3*10
                %obj.dqContainer(i) = obj.pre_integrations(i+1).delta_q;%10*4
                obj.dqContainerArray(:,i) = obj.pre_integrations(i+1).delta_q;%10*4
                obj.dvContainer(:,i) = obj.pre_integrations(i+1).delta_v;%3*10
                obj.JContainer(:,:,i) = obj.pre_integrations(i+1).jacobian;%15*15*10
                obj.PContainer(:,:,i) = obj.pre_integrations(i+1).covariance;%15*15*10
                obj.baContainer(:,i) = obj.pre_integrations(i+1).linearized_ba;%3*10
                obj.bgContainer(:,i) = obj.pre_integrations(i+1).linearized_bg;%3*10
                obj.dtContainer(i) = obj.pre_integrations(i+1).sum_dt;%1*10
            end
                %obj.dqContainerArray = compact(obj.dqContainer);%convert quaternion class into array
                obj.dqContainerArray = [obj.dqContainerArray(4,:);obj.dqContainerArray(1:3,:)];
                
                global ICP_N_t;
                global ICP_N_q;
                ICP_N_t_ = ICP_N_t;
                ICP_N_q_ = ICP_N_q;
%             [para_pose,para_speedbias] = ceres_optimization(obj.Ps,obj.Rs,obj.Qs,obj.Vs,obj.Bas,obj.Bgs,...
%                 obj.dpContainer,obj.dqContainerArray,obj.dvContainer,obj.JContainer,obj.PContainer,...
%                 obj.baContainer,obj.bgContainer,obj.dtContainer,obj.deltaT2last,obj.deltaT2base,obj.frame_count,obj.g_sum,ICP_N_t_,ICP_N_q_);
            
            [para_pose,para_speedbias] = ceres_optimization_onlyPC(obj.Ps,obj.Rs,obj.Qs,obj.Vs,obj.Bas,obj.Bgs,...
                obj.dpContainer,obj.dqContainerArray,obj.dvContainer,obj.JContainer,obj.PContainer,...
                obj.baContainer,obj.bgContainer,obj.dtContainer,obj.deltaT2last,obj.deltaT2base,obj.frame_count,obj.g_sum,ICP_N_t_,ICP_N_q_);
            
%             ceres_optimization(obj.Ps,obj.Rs,obj.Qs,obj.Vs,obj.Bas,obj.Bgs,...
%                 obj.dpContainer,obj.dqContainerArray,obj.dvContainer,obj.JContainer,obj.PContainer,...
%                 obj.baContainer,obj.bgContainer,obj.dtContainer,obj.deltaT2last,obj.deltaT2base,obj.frame_count,obj.g_sum);
            
            % return
            para_pose = reshape(para_pose,[7,11]).';
            para_speedbias = reshape(para_speedbias,[9,11]).';

            
            % 计算初始位置的R的偏移，先用ypr 如果不行，直接算R的差
            origin_R0 = R2ypr(obj.Rs(:,:,1));
            origin_R00 = R2ypr(quat2rotmliub([para_pose(1,4),para_pose(1,5),para_pose(1,6),para_pose(1,7)]).');
            origin_P0 = obj.Ps(:,1);
            y_diff = origin_R0(1) - origin_R00(1);
            rot_diff = eul2rotm([y_diff, 0, 0]);
            if (abs(abs(origin_R0(2)) - 90) < 1.0 || abs(abs(origin_R00(2)) - 90) < 1.0)
                disp(["euler singular point!"]);
                rot_diff = obj.Rs(:,:,1) * quat2rotmliub([para_pose(1,4),para_pose(1,5),para_pose(1,6),para_pose(1,7)]).'.';
            end
            % 不对第一帧的位姿进行处理
            % rot_diff = [1 0 0;0 1 0;0 0 1];
            
            for i = 1:obj.frame_count
           
                obj.Rs(:,:,i) = rot_diff * quat2rotmliub([para_pose(i,4),para_pose(i,5),para_pose(i,6),para_pose(i,7)]/norm([para_pose(i,4),para_pose(i,5),para_pose(i,6),para_pose(i,7)])).';

                obj.Ps(:,i) = rot_diff * [para_pose(i,1) - para_pose(1,1);
                                        para_pose(i,2) - para_pose(1,2);
                                        para_pose(i,3) - para_pose(1,3)] + origin_P0;

                obj.Vs(:,i) = rot_diff * [para_speedbias(i,1);
                                            para_speedbias(i,2);
                                            para_speedbias(i,3)];

                obj.Bas(:,i) = [para_speedbias(i,4);
                          para_speedbias(i,5);
                          para_speedbias(i,6)];

                obj.Bgs(:,i) = [para_speedbias(i,7);
                                            para_speedbias(i,8);
                                            para_speedbias(i,9)];
            end
            obj.Qs = rotm2quatliub(permute(obj.Rs,[2,1,3]));
        end
        
        function slide_window(obj,dt,acc,gyr)
            
            %直接平移正好可以当作初值
            obj.Rs(:,:,1:obj.window_size) =obj.Rs(:,:,2:end);
            obj.Ps(:,1:obj.window_size) =obj.Ps(:,2:end);
            obj.Vs(:,1:obj.window_size) =obj.Vs(:,2:end);
            obj.Bas(:,1:obj.window_size) =obj.Bas(:,2:end);
            obj.Bgs(:,1:obj.window_size) =obj.Bgs(:,2:end);
            obj.g_sum(:,1:obj.window_size) =obj.g_sum(:,2:end);
            obj.g_sum(:,obj.window_size + 1) =zeros(3,1);
            %直接平移，processPC的时候覆盖旧值吗？
            obj.deltaT2last(:,1:obj.frame_count-2) = obj.deltaT2last(:,2:obj.frame_count-1);
            %obj.deltaT2base = deltaX_icp2base;
            
            obj.pre_integrations(1) = [];
        end
        
        function [P,Q,R,V,Ba,Bg] = outputState(obj)
            %state = zeros(16,1);
            P = obj.Ps(:,obj.frame_count);
            Q = obj.Qs(:,obj.frame_count);
            R = obj.Rs(:,:,obj.frame_count);
            V = obj.Vs(:,obj.frame_count);
            Ba = obj.Bas(:,obj.frame_count);
            Bg = obj.Bgs(:,obj.frame_count);
        end
        
        function [g_sum_] = outputDeltaG(obj)
            g_sum_ = obj.g_sum(:,obj.frame_count);

        end        
    end
end