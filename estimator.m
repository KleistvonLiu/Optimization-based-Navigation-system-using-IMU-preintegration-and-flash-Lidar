classdef estimator < handle
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
        initial; %for processIMU, if initial data or not
        frame_count; % number of key frames, from 1 to window_size+1
        new_frame; % if new frame,Pnew initial value = Pold end value
        Ps;
        Vs;
        Rs;
        Bas;
        Bgs;
        window_size;
        pre_integrations;
        first_imu;
        
        
        count;% just for test processPC
    end
    methods
        %!these 4 variables should be references
        function obj = estimator(window_size)
            if nargin == 1
                obj.window_size = window_size;
            else
                obj.window_size = 10;
            end
            obj.Ps = zeros(3,obj.window_size+1);
            obj.Vs = zeros(3,obj.window_size+1);
            obj.Rs = repmat(eye(3),1,1,obj.window_size+1);
            obj.Bas = zeros(3,obj.window_size+1);
            obj.Bgs = zeros(3,obj.window_size+1);
            obj.first_imu = false;
            obj.dt_buf = [];
            obj.acc_buf = [];
            obj.gyr_buf = [];
            obj.count = 1;
            obj.frame_count = 1;
            obj.new_frame = 0;
            obj.pre_integrations = [];
            
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
        
        function processIMU(obj,dt,linear_acceleration,angular_velocity)
            %1.初始化，如果当前帧不是第一帧IMU，那么就把它看成第一个IMU，而且把他的值取出来作为初始值
            if (~obj.first_imu)
                obj.first_imu = true;
                obj.acc_0 = linear_acceleration;
                obj.gyr_0 = angular_velocity;
            end
            % 注意此处是使用上一个历元的IMU数据!因此first_imu的设置还是有作用的
            if (length(obj.pre_integrations)<(obj.frame_count))
                obj.pre_integrations = [obj.pre_integrations IntegrationBase(obj.acc_0, obj.gyr_0, obj.Bas(obj.frame_count), obj.Bgs(obj.frame_count))];
            end
            
            
            if (obj.frame_count ~= 1)
                
                obj.pre_integrations(obj.frame_count).push_back(dt, linear_acceleration, angular_velocity);
                %if(solver_flag != NON_LINEAR)
                %    tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);
                
                obj.dt_buf=[obj.dt_buf,dt];
                obj.acc_buf=[obj.acc_buf,linear_acceleration];
                obj.gyr_buf=[obj.gyr_buf,angular_velocity];
                
                j = obj.frame_count;
                
                % if new frame,Pnew initial value = Pold end value
                if (obj.new_frame == 1)
                    obj.Rs(:,:,j) = obj.Rs(:,:,j-1);
                    obj.Ps(:,j) =obj.Ps(:,j-1);
                    obj.Vs(:,j) =obj.Vs(:,j-1);
                    obj.new_frame = 0;
                end 
                
                global g
                un_acc_0 = obj.Rs(:,:,j) * (obj.acc_0 - obj.Bas(:,j)) - g;
                un_gyr = 0.5 * (obj.gyr_0 + angular_velocity) - obj.Bgs(:,j);% !! Bgs(j)
                un_gyr = un_gyr * dt;
                obj.Rs(:,:,j) =obj.Rs(:,:,j)*quat2rotm(quaternion(1,un_gyr(1)/2,un_gyr(2)/2,un_gyr(3)/2));
                un_acc_1 = obj.Rs(:,:,j) * (linear_acceleration - obj.Bas(:,j)) - g;% !! Bas(j)
                un_acc = 0.5 * (un_acc_0 + un_acc_1);
                obj.Ps(:,j) =obj.Ps(:,j)+ dt * obj.Vs(:,j) + 0.5 * dt * dt * un_acc;
                obj.Vs(:,j) =obj.Vs(:,j)+ dt * un_acc;
            end
            obj.acc_0 = linear_acceleration;%应该是用来初始化IntegrationBase，每来一次数据都更新但是只有framecount+1的时候才用到
            obj.gyr_0 = angular_velocity;
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
                obj.pre_integrations = [obj.pre_integrations IntegrationBase(obj.acc_0, obj.gyr_0, obj.Bas(obj.frame_count), obj.Bgs(obj.frame_count))];
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
                obj.Rs(:,:,j) =obj.Rs(:,:,j)*quat2rotm(quaternion(1,un_gyr(1)/2,un_gyr(2)/2,un_gyr(3)/2));
                un_acc_1 = obj.Rs(:,:,j) * (linear_acceleration - obj.Bas(:,j)) - g;% !! Bas(j)
                un_acc = 0.5 * (un_acc_0 + un_acc_1);
                obj.Ps(:,j) =obj.Ps(:,j)+ dt * obj.Vs(:,j) + 0.5 * dt * dt * un_acc;
                obj.Vs(:,j) =obj.Vs(:,j)+ dt * un_acc;
            end
            obj.acc_0 = linear_acceleration;%应该是用来初始化IntegrationBase，每来一次数据都更新但是只有framecount+1的时候才用到
            obj.gyr_0 = angular_velocity;
        end
        
        function processPC(obj,c)
            
            obj.count = obj.count+1;
            %             if obj.frame_count = obj.window_size + 1
            %                 [Ps,Rs,Vs,Bas,Bgs]=DOoptimization(Ps,Rs,Vs,Bas,Bgs,预积分变量,J,Q)
            %             else
            %                 obj.frame_count = obj.frame_count + 1;
            %                 obj.new_frame = 1;
            %             end
            
            
            
        end
        
        function testProcessPC(obj,flag,c)
            %% 只测试连续的imu积分
            if(flag == 1)
                obj.frame_count = 2; %
                if obj.count == c
                    obj.count = 1;
                    % obj.frame_count = obj.frame_count + 1;
                end
            end
            %% 测试10个frame的imu积分
            if(flag == 2)
                if obj.frame_count == obj.window_size + 1
                    %[Ps,Rs,Vs,Bas,Bgs]=optimization(Ps,Rs,Vs,Bas,Bgs,预积分变量,J,Q)
                    slide_window(obj,1,2,3);
                else
                    obj.frame_count = obj.frame_count + 1;
                    obj.new_frame = 1;
                end
            end
            
        end
        
        function optimization(obj,dt,acc,gyr)
            
            %             if obj.frame_count = obj.window_size
            %                 [Ps,Rs,Vs,Bas,Bgs]=DOoptimization(Ps,Rs,Vs,Bas,Bgs,预积分变量,J,Q)
            %             else
            %                 obj.frame_count = obj.frame_count + 1;
            %             end
            
        end
        
        function slide_window(obj,dt,acc,gyr)
            
            obj.Rs(:,:,1:obj.window_size) =obj.Rs(:,:,2:end);
            obj.Ps(:,1:obj.window_size) =obj.Ps(:,2:end);
            obj.Vs(:,1:obj.window_size) =obj.Vs(:,2:end);
            
            obj.pre_integrations(1) = [];
        end
        
        function push_back(obj,dt,acc,gyr)
            
            obj.dt_buf=[obj.dt_buf,dt];
            obj.acc_buf=[obj.acc_buf,acc];
            obj.gyr_buf=[obj.gyr_buf,gyr];
            propagate(obj,dt,acc,gyr)
        end
        
        function repropagate(obj,initial_linearized_ba,initial_linearized_bg)
            obj.sum_dt = 0.0;
            obj.acc_0 = obj.linearized_acc;
            obj.gyr_0 = obj.linearized_gyr;
            obj.delta_p = zeros(3,1);
            obj.delta_q = quaternion(1,0,0,0);%四元数的表达方式
            obj.delta_v = zeros(3,1);
            obj.linearized_ba = initial_linearized_ba;
            obj.linearized_bg = initial_linearized_bg;
            obj.jacobian = eye(15);
            obj.covariance = zeros(15,15);
            for i=1:size(obj.dt_buf,2)
                propagate(obj.dt_buf(i), obj.acc_buf(i), obj.gyr_buf(i));
            end
        end
        
        function propagate(obj,dt,acc_1,gyr_1)
            obj.dt=dt;
            obj.acc_1=acc_1;
            obj.gyr_1=gyr_1;
            [result_delta_q,result_delta_p,result_delta_v,result_linearized_ba,result_linearized_bg]=...
                midPointIntegration(obj, obj.dt, obj.acc_0, obj.gyr_0, acc_1, gyr_1, obj.delta_p,...
                obj.delta_q, obj.delta_v,obj.linearized_ba, obj.linearized_bg,...
                result_delta_p, result_delta_q, result_delta_v,...
                result_linearized_ba, result_linearized_bg, 1);
            obj.delta_p = result_delta_p;
            obj.delta_q = result_delta_q;
            obj.delta_v = result_delta_v;
            obj.linearized_ba = result_linearized_ba;
            obj.linearized_bg = result_linearized_bg;
            obj.delta_q = normalize(obj.delta_q);
            obj.sum_dt = obj.sum_dt+dt;
            obj.acc_0 = acc_1;
            obj.gyr_0 = gyr_1;
        end
        
        function [result_delta_q,result_delta_p,result_delta_v,result_linearized_ba,result_linearized_bg] ...
                = midPointIntegration(obj, dt_, acc_0_, gyr_0, acc_1, gyr_1, delta_p,...
                delta_q, delta_v,linearized_ba, linearized_bg,...
                result_delta_p, result_delta_q, result_delta_v,...
                result_linearized_ba, result_linearized_bg, update_jacobian)
            un_acc_0 = quat2rotm(obj.delta_q)*(obj.acc_0 - obj.linearized_ba);
            un_gyr = 0.5 * (obj.gyr_0 + obj.gyr_1) - obj.linearized_bg;
            result_delta_q = obj.delta_q*quaternion(1,un_gyr(0)*obj.dt/2,un_gyr(1)*obj.dt/2,un_gyr(2)*obj.dt/2);
            un_acc_1 = quat2rotm(result_delta_q)*(obj.acc_1-obj.linearized_ba);
            un_acc = 0.5 * (un_acc_0 + un_acc_1);
            result_delta_p = obj.delta_p + obj.delta_v * obj.dt + 0.5 * un_acc * obj.dt * obj.dt;
            result_delta_v = obj.delta_v + un_acc * obj.dt;
            result_linearized_ba = obj.linearized_ba;
            result_linearized_bg = obj.linearized_bg;
            if(update_jacobian)
                w_x = 0.5 * (obj.gyr_0 + obj.gyr_1) - obj.linearized_bg;
                a_0_x = obj.acc_0 - obj.linearized_ba;
                a_1_x = obj.acc_1 - obj.linearized_ba;
                R_w_x=skew(w_x);
                R_a_0_x=skew(a_0_x);
                R_a_1_x=skew(a_1_x);
                
                F = zeros(15, 15);
                F(1:3,1:3) = eye(3);
                F(1:3, 4:6) = -0.25 *quat2rotm(obj.delta_q)* R_a_0_x * obj.dt * obj.dt +...
                    -0.25 * quat2rotm(result_delta_q)* R_a_1_x * (eye(3) - R_w_x * obj.dt) * obj.dt * obj.dt;
                F(1:3,7:9) = eye(3)*obj.dt;
                F(1:3, 10:12) = -0.25 * (quat2rotm(obj.delta_q) + quat2rotm(result_delta_q)) *obj.dt * obj.dt;
                F(1:3, 13:15) = -0.25 * quat2rotm(result_delta_q) * R_a_1_x * obj.dt *obj.dt * (-obj.dt);
                F(4:6, 4:6) = eye(3) - R_w_x * obj.dt;
                F(4:6, 13:15) = -1.0 * eye(3) * obj.dt;
                F(7:9, 4:6) = -0.5 * quat2rotm(obj.delta_q) * R_a_0_x * obj.dt + ...
                    -0.5 *quat2rotm(result_delta_q)* R_a_1_x * (eye(3) - R_w_x * obj.dt) * obj.dt;
                F(7:9, 7:9) = eye(3);
                F(7:9, 10:12) = -0.5 * (quat2rotm(obj.delta_q) + quat2rotm(result_delta_q)) *obj.dt;
                F(7:9, 13:15) = -0.5 * quat2rotm(result_delta_q) * R_a_1_x * obj.dt * -obj.dt;
                F(10:12, 10:12) = eye(3);
                F(13:15, 13:15) = eye(3);
                %cout<<"A"<<endl<<A<<endl;
                
                %MatrixXd V = MatrixXd::Zero(15,18);
                V(1:3,1:3) =  0.25 * quat2rotm(obj.delta_q) * obj.dt * obj.dt;
                V(1:3, 4:6) =  0.25 * -quat2rotm(result_delta_q) * R_a_1_x  * obj.dt * obj.dt * 0.5 * obj.dt;
                V(1:3,7:9) =  0.25 * quat2rotm(result_delta_q) * obj.dt * obj.dt;
                V(1:3, 10:12) =  V(1:3, 4:6);
                V(4:6, 4:6) =  0.5 * eye(3) * obj.dt;
                V(4:6, 10:12) =  0.5 * eye(3) * obj.dt;
                V(7:9, 1:3) =  0.5 * quat2rotm(obj.delta_q) * obj.dt;
                V(7:9, 4:6) =  0.5 * -quat2rotm(result_delta_q) * R_a_1_x  * obj.dt * 0.5 * obj.dt;
                V(7:9, 7:9) =  0.5 * quat2rotm(result_delta_q) * obj.dt;
                V(7:9, 10:12) =  V(7:9, 4:6);
                V(10:12, 13:15) = eye(3) * obj.dt;
                V(13:15, 16:18) = eye(3) * obj.dt;
                
                %step_jacobian = F;
                %step_V = V;
                obj.jacobian = F * obj.jacobian;
                obj.covariance = F * obj.covariance * F.transpose() + V * obj.noise * V.transpose();
            end
        end
        function setNoise(obj)
            global ACC_N
            global GYR_N
            global ACC_W
            global GYR_W
            % ACC_N
            obj.noise(1:3,1:3)= ACC_N*ACC_N*eye(3);%should be defined global
            obj.noise(4:6,4:6)= GYR_N*GYR_N*eye(3);
            obj.noise(7:9,7:9)= ACC_N*ACC_N*eye(3);
            obj.noise(10:12,10:12)= GYR_N*GYR_N*eye(3);
            obj.noise(13:15,13:15)= ACC_W*ACC_W*eye(3);
            obj.noise(16:18,16:18)= GYR_W*GYR_W*eye(3);
        end
    end
end