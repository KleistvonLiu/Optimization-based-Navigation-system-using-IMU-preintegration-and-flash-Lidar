
function X_sw_up = optimEstimatorRN(X_sw_est,...
    deltaX_sw_INS, pairedFrames_sw_INS, deltaX_sw_PC, pairedFrames_sw_PC,...
    length_sw, sizeSlidingWindow,...
    gravity_N, angRate_IN_N, deltaT, flagUseTrueRelStates)
%see example https://de.mathworks.com/help/optim/ug/nonlinear-least-squares-problem-based-basics.html

%TODO consider R_LN

X_sw_up = zeros(10, length_sw);

% residual_sw_INS = zeros(3,sizeSlidingWindow-1);
obj_INS = optimexpr(1);
obj_PC = optimexpr(1);

X_sw_sym_posi = optimvar('X_sw_sym_posi', 3, length_sw,...
                        'LowerBound', (X_sw_est(1:3,end-length_sw+1:end)-0.2),...
                        'UpperBound', (X_sw_est(1:3,end-length_sw+1:end)+0.2));
X_sw_sym_vel = optimvar('X_sw_sym_vel', 3, length_sw,...
                        'LowerBound', (X_sw_est(4:6,end-length_sw+1:end)-0.02),...
                        'UpperBound', (X_sw_est(4:6,end-length_sw+1:end)+0.02));
X_sw_sym_quat = optimvar('X_sw_sym_quat', 4, length_sw,...
                        'LowerBound', (X_sw_est(7:10,end-length_sw+1:end)-0.01),...
                        'UpperBound', (X_sw_est(7:10,end-length_sw+1:end)+0.01));


% compute residual of IMU pre-integration
for i = flip(1:(length_sw-1))
    
        %% === for residual INS (IMU factor)
        vector_invVar_residual_INS = 1./[0.64 0.64 0.64...
                            0.64 0.64 0.64...
                            0.00001 0.00001 0.00001];
        % diag(positionXYZ, velocityXYZ, attitudeXYZ)
        
        %quat_prev = X_sw_sym_quat(:,end-i-1);
        %quat_cur = X_sw_sym_quat(:,end-i);

        idxPrev = pairedFrames_sw_INS(1,i);  
        idxCur = pairedFrames_sw_INS(2,i);        
        quat_prev = X_sw_sym_quat(:,idxPrev);
        quat_cur = X_sw_sym_quat(:,idxCur);
        
        q = quat_prev;
        R_prev = [q(1)^2+q(4)^2-q(2)^2-q(3)^2	2*(q(1)*q(2)+q(3)*q(4))     2*(q(1)*q(3)-q(2)*q(4));
                2*(q(1)*q(2)-q(3)*q(4))         q(2)^2+q(4)^2-q(1)^2-q(3)^2 2*(q(2)*q(3)+q(1)*q(4));
                2*(q(1)*q(3)+q(2)*q(4))         2*(q(2)*q(3)-q(1)*q(4)) 	q(3)^2+q(4)^2-q(1)^2-q(2)^2];


        %quat_inv_prev = quatInverseliub(quat_prev);
        quat_inv_prev = [-quat_prev(1:3); quat_prev(4)]; 
        
%         quat_prev_cur_est = quatMultiplicationliub(quat_inv_prev, quat_cur); % from prev to cur
        p = quat_inv_prev;
        L_p = [p(4)     -p(3)	p(2)	p(1);
                p(3)     p(4)	-p(1)	p(2);
                -p(2)    p(1)	p(4)	p(3);
                -p(1)	-p(2)	-p(3)	p(4)];
        quat_prev_cur_est_INS = L_p * quat_cur;

        quat_prev_cur_meas_INS = deltaX_sw_INS(7:10,i);
        

%         quat_inv_prev_cur_meas = quatInverseliub(quat_prev_cur_meas);
        quat_inv_prev_cur_meas = [-quat_prev_cur_meas_INS(1:3); quat_prev_cur_meas_INS(4)]; 

%         delta_quat = quatMultiplicationliub(quat_inv_prev_cur_meas, quat_prev_cur_est); % TODO or way around?
        p = quat_inv_prev_cur_meas;
        L_p = [p(4)     -p(3)	p(2)	p(1);
                p(3)     p(4)	-p(1)	p(2);
                -p(2)    p(1)	p(4)	p(3);
                -p(1)	-p(2)	-p(3)	p(4)];
        delta_quat = L_p * quat_prev_cur_est_INS;
        
%         delta_theta = 2 * sign(delta_quat(4)) * delta_quat(1:3); % ATTENSION!!!!!!!!!!!!!!!
        delta_theta = 2 * delta_quat(1:3);
%         residual_quat_INS = delta_theta(1)^2/vector_residual_INS(7)...
%                             + delta_theta(2)^2/vector_residual_INS(8)...
%                             + delta_theta(3)^2/vector_residual_INS(9);
%         residual_quat_INS = sum(delta_theta.^2);
        residual_quat_INS = vector_invVar_residual_INS(7) * delta_theta(1)^2 ...
                            + vector_invVar_residual_INS(8) * delta_theta(2)^2 ...
                            + vector_invVar_residual_INS(9) * delta_theta(3)^2;
        
        vel_prev = X_sw_sym_vel(:,idxPrev);
        vel_cur = X_sw_sym_vel(:,idxCur);
        delta_vel_est_INS = R_prev * (vel_cur - vel_prev - gravity_N * deltaT + 2*cross(angRate_IN_N, vel_prev) * deltaT);
        delta_vel_meas_INS = deltaX_sw_INS(4:6,i);
        dd_vel = (delta_vel_est_INS - delta_vel_meas_INS);
%         residual_vel_INS = sum(dd_vel.^2);
%         residual_vel_INS = dd_vel(1)^2/vector_residual_INS(4)...
%                             + dd_vel(2)^2/vector_residual_INS(5)...
%                             + dd_vel(3)^2/vector_residual_INS(6);
        residual_vel_INS = vector_invVar_residual_INS(4) * dd_vel(1)^2 ...
                            + vector_invVar_residual_INS(5) * dd_vel(2)^2 ...
                            + vector_invVar_residual_INS(6) * dd_vel(3)^2;

        posi_prev = X_sw_sym_posi(:,idxPrev);
        posi_cur = X_sw_sym_posi(:,idxCur);
        delta_posi_est_INS = R_prev * (posi_cur - posi_prev - vel_prev * deltaT...
                                    - 0.5 * gravity_N * deltaT^2 + 0.5*2*cross(angRate_IN_N, vel_prev) * deltaT^2);
        delta_posi_meas_INS = deltaX_sw_INS(1:3,i);
        dd_posi = (delta_posi_est_INS - delta_posi_meas_INS);
%         residual_posi_INS = sum(dd_posi.^2);
        residual_posi_INS = vector_invVar_residual_INS(1) * dd_posi(1)^2 ...
                            + vector_invVar_residual_INS(2) * dd_posi(2)^2 ...
                            + vector_invVar_residual_INS(3) * dd_posi(3)^2;
%         residual_posi_INS = dd_posi(1)^2/vector_residual_INS(1)...
%                             + dd_posi(2)^2/vector_residual_INS(2)...
%                             + dd_posi(3)^2/vector_residual_INS(3);
        
%         residual_sw_INS(:,i) = [residual_posi_INS residual_vel_INS residual_quat_INS]';
        if(i == 0)
            obj_INS = residual_posi_INS + residual_vel_INS + residual_quat_INS;
%             obj_INS = residual_vector_INS' * inv(cov_residual_INS) * residual_vector_INS;
        else
            obj_INS = obj_INS + residual_posi_INS + residual_vel_INS + residual_quat_INS;
%             obj_INS = obj_INS + residual_vector_INS' * inv(cov_residual_INS) * residual_vector_INS;
        end
        
        
        %% === for residual PC (PC factor)
        vector_invVar_residual_PC = 1./[0.0001 0.0001 0.0001...
                            100 100 100];
        % diag(positionXYZ, attitudeXYZ)
        
        idxPCA = pairedFrames_sw_PC(1, i);
        idxPCB = pairedFrames_sw_PC(2, i);
        
        posi_A = X_sw_sym_posi(:,idxPCA);
        posi_B = X_sw_sym_posi(:,idxPCB);       
        
        quat_AB = X_sw_sym_quat(:,idxPCB);
        
        q = quat_AB;
        R_AB = [q(1)^2+q(4)^2-q(2)^2-q(3)^2	2*(q(1)*q(2)+q(3)*q(4))     2*(q(1)*q(3)-q(2)*q(4));
                2*(q(1)*q(2)-q(3)*q(4))         q(2)^2+q(4)^2-q(1)^2-q(3)^2 2*(q(2)*q(3)+q(1)*q(4));
                2*(q(1)*q(3)+q(2)*q(4))         2*(q(2)*q(3)-q(1)*q(4)) 	q(3)^2+q(4)^2-q(1)^2-q(2)^2];
        
        delta_posi_est_PC = posi_B - posi_A;
        if(flagUseTrueRelStates >0.5)
%         if(true)
            delta_posi_meas_PC = deltaX_sw_PC(1:3,i);
        else
%             delta_posi_meas_PC = R_prev' * deltaX_sw_PC(1:3,end-i);
            delta_posi_meas_PC = deltaX_sw_PC(1:3,i);
        end
        
        dd_posi_PC = delta_posi_est_PC - delta_posi_meas_PC;
%         residual_posi_PC = dd_posi_PC(1)^2/vector_residual_PC(1)...
%                             + dd_posi_PC(2)^2/vector_residual_PC(2)...
%                             + dd_posi_PC(3)^2/vector_residual_PC(3);
        residual_posi_PC = vector_invVar_residual_PC(1) * dd_posi_PC(1)^2 ...
                            + vector_invVar_residual_PC(2) * dd_posi_PC(2)^2 ...
                            + vector_invVar_residual_PC(3) * dd_posi_PC(3)^2;
%         residual_posi_PC = sum(dd_posi_PC.^2);
        

        quat_AB_est_PC = quat_AB;
        quat_AB_meas_PC = deltaX_sw_PC(4:7,i);
        
%         quat_inv_prev_cur_meas = quatInverseliub(quat_prev_cur_meas);
        quat_inv_prev_cur_meas_PC = [-quat_AB_meas_PC(1:3); quat_AB_meas_PC(4)]; 

%         delta_quat = quatMultiplicationliub(quat_inv_prev_cur_meas, quat_prev_cur_est); % TODO or way around?
        p = quat_inv_prev_cur_meas_PC;
        L_p = [p(4)     -p(3)	p(2)	p(1);
                p(3)     p(4)	-p(1)	p(2);
                -p(2)    p(1)	p(4)	p(3);
                -p(1)	-p(2)	-p(3)	p(4)];
        delta_quat_PC = L_p * quat_AB_est_PC;
        
%         delta_theta = 2 * sign(delta_quat(4)) * delta_quat(1:3); % ATTENSION!!!!!!!!!!!!!!!
        delta_theta_PC = 2 * delta_quat_PC(1:3);
%         residual_quat_PC = delta_theta_PC(1)^2/vector_invVar_residual_PC(4)...
%                             + delta_theta_PC(2)^2/vector_invVar_residual_PC(5)...
%                             + delta_theta_PC(3)^2/vector_invVar_residual_PC(6);
        residual_quat_PC = vector_invVar_residual_PC(4) * delta_theta_PC(1)^2 ...
                            + vector_invVar_residual_PC(5) * delta_theta_PC(2)^2 ...
                            + vector_invVar_residual_PC(6) * delta_theta_PC(3)^2;
%         residual_quat_PC = sum(delta_theta_PC.^2);
                        
        if(i == 0)
            obj_PC = residual_posi_PC + residual_quat_PC;
%             obj_PC = residual_vector_PC' * residual_vector_PC;
%             obj_PC = residual_vector_PC' * inv(cov_residual_PC) * residual_vector_PC;
        else
            obj_PC = obj_PC + residual_posi_PC + residual_quat_PC;
%             obj_PC = obj_PC + residual_vector_PC' * residual_vector_PC;
%             obj_PC = obj_PC + residual_vector_PC' * inv(cov_residual_PC) * residual_vector_PC;
        end
end

obj = obj_INS + obj_PC;
% obj = obj_INS;


x0.X_sw_sym_posi = X_sw_est(1:3,end-length_sw+1:end);
x0.X_sw_sym_vel = X_sw_est(4:6,end-length_sw+1:end);
x0.X_sw_sym_quat = X_sw_est(7:10,end-length_sw+1:end);

% obj_INS = sum(sum(residual_sw_INS));
prob = optimproblem('Objective', obj);

options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt');

[sol2,fval,exitflag,output,lambda] = solve(prob, x0, 'options', options);
% [sol2,fval,exitflag,output,lambda] = solve(prob, x0);


X_sw_up(1:3,:) = sol2.X_sw_sym_posi;
X_sw_up(4:6,:) = sol2.X_sw_sym_vel;
X_sw_up(7:10,:) = sol2.X_sw_sym_quat;


end