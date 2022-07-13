classdef IntegrationBasev3 < handle
    properties
        dt;
        acc_0;
        gyr_0;
        acc_1;
        gyr_1;
        %!!should be const
        linearized_acc; %acc value when initializing the class
        linearized_gyr;
        linearized_ba;
        linearized_bg;
        
        jacobian;%!!should have initial size
        covariance;
        step_jacobian;
        step_V;
        noise;
        
        sum_dt;
        delta_p;
        delta_q;
        delta_v;
        
        dt_buf;%存储所有的imu数据，目的：当bias很大的时候 需要repropagate
        acc_buf;
        gyr_buf;
    end
    methods
        %!these 4 variables should be references
        function obj = IntegrationBasev3(initial_acc_0,initial_gyr_0,initial_linearized_ba,initial_linearized_bg)
            if nargin == 4
                obj.acc_0 = initial_acc_0;
                obj.gyr_0 = initial_gyr_0;
                obj.linearized_acc = initial_acc_0;
                obj.linearized_gyr = initial_gyr_0;
                obj.linearized_ba = initial_linearized_ba;
                obj.linearized_bg = initial_linearized_bg;
                obj.jacobian = eye(15);
                obj.covariance = zeros(15,15);
                obj.sum_dt = 0;%默认为double
                obj.delta_p = zeros(3,1);
                %obj.delta_q = quaternion(1,0,0,0);%四元数的表达方式
                obj.delta_q = [0;0;0;1];
                obj.delta_v = zeros(3,1);
                obj.noise = zeros(18,18);
                setNoise(obj);
                obj.dt_buf = [];
                obj.acc_buf = [];
                obj.gyr_buf = [];
            end
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
            %obj.delta_q = quaternion(1,0,0,0);%四元数的表达方式
            obj.delta_q = [0;0;0;1];
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
                obj.delta_q, obj.delta_v,obj.linearized_ba, obj.linearized_bg, 1);
            obj.delta_p = result_delta_p;
            obj.delta_q = result_delta_q;
            obj.delta_v = result_delta_v;
            obj.linearized_ba = result_linearized_ba;
            obj.linearized_bg = result_linearized_bg;
            %obj.delta_q = normalize(obj.delta_q);
            obj.delta_q = (obj.delta_q)/norm(obj.delta_q);
            obj.sum_dt = obj.sum_dt+dt;
            obj.acc_0 = acc_1;
            obj.gyr_0 = gyr_1;
        end
        
        function [result_delta_q,result_delta_p,result_delta_v,result_linearized_ba,result_linearized_bg] ...
                = midPointIntegration(obj, dt_, acc_0_, gyr_0, acc_1, gyr_1, delta_p,...
                delta_q, delta_v,linearized_ba, linearized_bg, update_jacobian)
            un_acc_0 = quat2rotmliub(obj.delta_q).'*(obj.acc_0 - obj.linearized_ba);
            un_gyr = 0.5 * (obj.gyr_0 + obj.gyr_1) - obj.linearized_bg;
            result_delta_q = (quatMultiplication(obj.delta_q,[un_gyr(1)*obj.dt/2,un_gyr(2)*obj.dt/2,un_gyr(3)*obj.dt/2,1].'));
            un_acc_1 = quat2rotmliub(result_delta_q).'*(obj.acc_1-obj.linearized_ba);
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
                F(1:3, 4:6) = -0.25 *quat2rotmliub(obj.delta_q).'* R_a_0_x * obj.dt * obj.dt +...
                    -0.25 * quat2rotmliub(result_delta_q).'* R_a_1_x * (eye(3) - R_w_x * obj.dt) * obj.dt * obj.dt;
                F(1:3,7:9) = eye(3)*obj.dt;
                F(1:3, 10:12) = -0.25 * (quat2rotmliub(obj.delta_q).' + quat2rotmliub(result_delta_q).') *obj.dt * obj.dt;
                F(1:3, 13:15) = -0.25 * quat2rotmliub(result_delta_q).' * R_a_1_x * obj.dt *obj.dt * (-obj.dt);
                F(4:6, 4:6) = eye(3) - R_w_x * obj.dt;
                F(4:6, 13:15) = -1.0 * eye(3) * obj.dt;
                F(7:9, 4:6) = -0.5 * quat2rotmliub(obj.delta_q).' * R_a_0_x * obj.dt + ...
                    -0.5 *quat2rotmliub(result_delta_q).'* R_a_1_x * (eye(3) - R_w_x * obj.dt) * obj.dt;
                F(7:9, 7:9) = eye(3);
                F(7:9, 10:12) = -0.5 * (quat2rotmliub(obj.delta_q).' + quat2rotmliub(result_delta_q).') *obj.dt;
                F(7:9, 13:15) = -0.5 * quat2rotmliub(result_delta_q).' * R_a_1_x * obj.dt * (-obj.dt);
                F(10:12, 10:12) = eye(3);
                F(13:15, 13:15) = eye(3);
                %cout<<"A"<<endl<<A<<endl;
                
                %MatrixXd V = MatrixXd::Zero(15,18);
                V(1:3,1:3) =  0.25 * quat2rotmliub(obj.delta_q).' * obj.dt * obj.dt;
                V(1:3, 4:6) =  0.25 * (-quat2rotmliub(result_delta_q).') * R_a_1_x  * obj.dt * obj.dt * 0.5 * obj.dt;
                V(1:3,7:9) =  0.25 * quat2rotmliub(result_delta_q).' * obj.dt * obj.dt;
                V(1:3, 10:12) =  V(1:3, 4:6);
                V(4:6, 4:6) =  0.5 * eye(3) * obj.dt;
                V(4:6, 10:12) =  0.5 * eye(3) * obj.dt;
                V(7:9, 1:3) =  0.5 * quat2rotmliub(obj.delta_q).' * obj.dt;
                V(7:9, 4:6) =  0.5 * -quat2rotmliub(result_delta_q).' * R_a_1_x  * obj.dt * 0.5 * obj.dt;
                V(7:9, 7:9) =  0.5 * quat2rotmliub(result_delta_q).' * obj.dt;
                V(7:9, 10:12) =  V(7:9, 4:6);
                V(10:12, 13:15) = eye(3) * obj.dt;
                V(13:15, 16:18) = eye(3) * obj.dt;
                
                %step_jacobian = F;
                %step_V = V;
                obj.jacobian = F * obj.jacobian;
                obj.covariance = F * obj.covariance * F.' + V * obj.noise * V.';
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