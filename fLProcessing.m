function [deltaX_icp2base, deltaX_icp2last,flagRegFinished, consec2base,...
    deltaX2base_ref, deltaX2last_ref,flagNewNode,erricp2base,erricp2last,...
    numValidPoints, mHRF,deltaX_icp2base_g,deltaX_icp2last_g] = fLProcessing(...
    pose_ref, Xn_pose_cur,...
    fL1MeasArrayDir, fL1MeasArrayRange, fL1MeasFlagNewData,...
    navMode, flagUseTrueRelStates,computeICP2base,...                               %input
    fLArrayDim, fL1Pose_B, X_init,params4base,params4last)                                  %parameters)


%% --- initialization
persistent first_scan state_basePc basePc mHRF_basePc...
    deltaX_prev first2base second_scan state_base_ref ...
    state_lastPc lastPc state_last_ref;
if isempty(first_scan), first_scan = 1;end % 是否是第一个测量数据
if isempty(state_basePc), state_basePc = [X_init(1:3);X_init(7:10)];end % ini pose of B in L
if isempty(mHRF_basePc), mHRF_basePc = 0;end % no idea
if isempty(basePc), basePc = zeros(fLArrayDim*fLArrayDim, 3);end % base Pc
if isempty(deltaX_prev), deltaX_prev = [0 0 0 0 0 0 1]';end % 上一次的icp结果，我们不需要
if isempty(first2base), first2base = 0;end % 将第一次点云做为base点云？
if isempty(second_scan), second_scan = 0;end % 第二次测量，not used
if isempty(state_base_ref), state_base_ref = zeros(6,1);end % base点云时的ground truth state

if isempty(state_lastPc), state_lastPc = [X_init(1:3);X_init(7:10)];end % ini pose of B in L
if isempty(lastPc), lastPc = zeros(fLArrayDim*fLArrayDim, 3);end % last Pc
if isempty(state_last_ref), state_last_ref = zeros(6,1);end


deltaX_icp2base = zeros(7,1); %ICP结果的差
erricp2base = 0;
deltaX_g = zeros(7,1); %guess of relative transformation
deltaX2base_ref = zeros(7,1); % referece icp results
%deltaX_icp = zeros(7,1); % icp results
flagRegFinished = 0; % if registration finished or not. set to 1 at the end
consec2base = 0; % 用来换base的，还需研究
mHRF = 0; % manual height range finder, one criterion for changing base PC
numValidPoints = 0; 

deltaX_icp2last = zeros(7,1); %ICP结果的差
deltaX2last_ref = zeros(7,1); %ICP结果的差
erricp2last = 0;

deltaX_icp2base_g = zeros(7,1);
deltaX_icp2last_g = zeros(7,1);

flagNewNode = 0;%是否定义一个新的node frame

coder.extrinsic('pcRegistration'); %!!!attention: to be modified
% no new fL1Measurement, skip matching algorithm
if((fL1MeasFlagNewData <0.5)||(navMode == 96)), return;end

%% --- pre-processing
% -- compute ponit clouds in body frame

%---------------
ratio = 3;
sizeArray = floor(256/ratio);
flashLidar1Points_temp = zeros(sizeArray, sizeArray, 3);
dirVector_ds = zeros(sizeArray, sizeArray, 3);
range_ds = zeros(sizeArray, sizeArray);

for i = 1:sizeArray
	for j = 1:sizeArray
        i3 = i*ratio;
        j3 = j*ratio;
        dirVector_ds(i,j,:) = fL1MeasArrayDir(i3-1, j3-1, :);
        subPixelRanges = fL1MeasArrayRange(i3-2:i3, j3-2:j3);
        zeroRanges = find(~subPixelRanges);
        
        % variant 1
%         range_ds(i,j) = sum(sum(subPixelRanges))/ratio/ratio;
        
%         % variant 2
%         if(isempty(zeroRanges))
%             range_ds(i,j) = sum(sum(subPixelRanges))/ratio/ratio;
%         else
%             range_ds(i,j) = 0;
%         end
%         
%         % variant 3
        nonZeroRanges = find(subPixelRanges);
        sizeNonZeroRanges = size(nonZeroRanges,1);
        range_ds(i,j) = sum(sum(subPixelRanges(nonZeroRanges)))/sizeNonZeroRanges;
    end
end

% pc_base_S_ds = dirVector_ds .* range_ds;
flashLidar1Points_temp(:,:,1) = dirVector_ds(:,:,1) .* range_ds;
flashLidar1Points_temp(:,:,2) = dirVector_ds(:,:,2) .* range_ds;
flashLidar1Points_temp(:,:,3) = dirVector_ds(:,:,3) .* range_ds;

% current raw point cloud in size Mx3 in sensor unit frame
curPc_U_raw = reshape(flashLidar1Points_temp,[],3);
idxValidPixels = find(curPc_U_raw(:,1));
idxNaN = find(isnan(curPc_U_raw));
idxInf = find(isinf(curPc_U_raw));

numValidPoints = size(idxValidPixels,1);
% if(numValidPoints < sizeArray*sizeArray), return;end

% current point cloud with valid points
% curPc_U = curPc_U_raw(idxValidPixels,:);
curPc_U = curPc_U_raw;

% -- transform point cloud from sensor unit frame to body frame
fL1Att_Dcm_UB = quat2rotmliub(fL1Pose_B(4:7,1))';

% current point cloud in body frame
curPc_B = curPc_U * fL1Att_Dcm_UB' + repmat(fL1Pose_B(1:3,1)',size(curPc_U,1),1);
% numValidPoints = size(curPc_B,1);
% curPc_B_MN = reshape(curPc_B, sizeArray,sizeArray,3);


% ---calculate manual height range finder value
% mHRF_points_dir = reshape(dirVector_ds(35:50,35:50,:),[],3)';
% mHRF_points_range = reshape(range_ds(35:50,35:50),[],1)';
% mHRF_points_list = zeros(3,size(mHRF_points_range,2));
% 
% mHRF_points_list(1,:) = mHRF_points_dir(1,:) .* mHRF_points_range;
% mHRF_points_list(2,:) = mHRF_points_dir(2,:) .* mHRF_points_range;
% mHRF_points_list(3,:) = mHRF_points_dir(3,:) .* mHRF_points_range;
% % for i = 1:size(mHRF_points_range,2)
% %     mHRF_points_list(:,i) = mHRF_points_dir(:,i).*mHRF_points_range(i);
% % end
% mHRF_points_B = fL1Att_Dcm_UB * mHRF_points_list + repmat(fL1Pose_B(1:3,1),1,size(mHRF_points_list,2));
% mHRF_points_B_ave = sum(mHRF_points_B,2)/16/16;
% mHRF = mHRF_points_B_ave(3);


% % ---sampling method
% % applying MLsampling on the curPc_B
% arrayActive = true(sizeArray);
% numToSelectedPoints = 4500;
% paramNormal.arrayActive = arrayActive;
% paramNormal.numSPoints = 6;
% paramNormal.radius = 0.15;
% paramNormal.useRadius = false;
% paramNormal.searchArrayDim = 6;
% normalsMN = computeNormals_realtime(curPc_B_MN, paramNormal);
% normals_list = reshape(normalsMN, [], 3);
% 
% 
% [arrayActive, numSPoints] = MaximumLeverageSampling_SGM(curPc_B, normals_list, arrayActive, numToSelectedPoints);
% idxActiveList = find(arrayActive);

% curPc_B_sampled = curPc_B(idxActiveList,:); %  could be modified?
curPc_B_sampled = curPc_B;

%% --- run ICP

if((fL1MeasFlagNewData <0.5)||(numValidPoints < 5000))% 200 TBD
    % --- if trigger does not come or
    %number of valid points in flash LiDAR point clouds is too small
    % do nothing
else
    % --- if it is the first scan
    if (first_scan > 0.5)
        state_basePc = [X_init(1:3);X_init(7:10)];
        state_base_ref = pose_ref;
        basePc = curPc_B_sampled;
        mHRF_basePc = mHRF;
        first2base = 1;
        
        state_lastPc = [X_init(1:3);X_init(7:10)];
        state_last_ref = pose_ref;
        lastPc = curPc_B_sampled;
        
        first_scan = 0;        
        second_scan = 1;
        flagRegFinished = 1;
        %flagNewNode = 1;
        
        deltaX_icp2base = zeros(7,1); %ICP结果的差
        deltaX_icp2last = zeros(7,1); %ICP结果的差
    else
        % icp to base point cloud
        % --- if it is the second or later scan
        % -- compute initial guess of transformation matrix for ICP
        % from the base frame to the current frame
        if computeICP2base
            q_pre = state_basePc(4:7);
            q_pre_inv= [-q_pre(1) -q_pre(2) -q_pre(3) q_pre(4)];
            R_qpre_inv = [q_pre_inv(4) -q_pre_inv(3) q_pre_inv(2) q_pre_inv(1);
                q_pre_inv(3) q_pre_inv(4) -q_pre_inv(1) q_pre_inv(2);
                -q_pre_inv(2) q_pre_inv(1) q_pre_inv(4) q_pre_inv(3);
                -q_pre_inv(1) -q_pre_inv(2) -q_pre_inv(3) q_pre_inv(4)];
            
            % - rotational part
            q_d = R_qpre_inv * Xn_pose_cur(4:7);
            q_d = q_d/norm(q_d);
            R_base_cur = quat2rotmliub(q_d).';% rotation matrix
            
            % - translational part
            R_base = quat2rotmliub(q_pre);
            p_d_L = Xn_pose_cur(1:3) - state_basePc(1:3);
            p_d = R_base * p_d_L;
            
            params4base.initialTransform(1:3, 1:3) = R_base_cur;
            params4base.initialTransform(1:3,4) = p_d;
            
            deltaX_icp2base_g = [p_d;q_d];
            
            % -- compute reference relative transformation
            R_base_ref = eulAng2rotmliub(state_base_ref(4:6));
            deltaX2base_ref(1:3,1) = R_base_ref * (pose_ref(1:3) - state_base_ref(1:3));
            
            q_base_ref = eulAng2quatliub(state_base_ref(4:6));
            q_cur_ref = eulAng2quatliub(pose_ref(4:6));
            
            q_base_ref_inv = [-q_base_ref(1:3); q_base_ref(4)];
            deltaX2base_ref(4:7,1) = quatMultiplication(q_base_ref_inv, q_cur_ref);
            
            R_cur_ref = eulAng2rotmliub(pose_ref(4:6));
            R_base_cur_ref = R_cur_ref*R_base_ref';
            deltaX2last_ref(4:7,1) = rotm2quatliub(R_base_cur_ref);
            
            % -- compute ICP relative transformation
            if(flagUseTrueRelStates >0.5)
                deltaX_icp2base = deltaX2base_ref;
            else
                [T2base, erricp2base] = pcicpFL_LJF_V2_mex(curPc_B_sampled, basePc, params4base);
                deltaX_icp2base(1:3,1) = T2base(1:3,4);
                deltaX_icp2base(4:7,1) = rotm2quatliub(T2base(1:3,1:3).');
            end
            
                    % -- check overlapping area and change base PC if necessary
            %         deltaEulAng = quat2eulAngliub(deltaX_cur(4:7));
                    eulAng_base_cur = quat2eulAngliub(deltaX_icp2base(4:7,1));
            
                    flagOverlappingThreshold = ...
                        (abs(deltaX_icp2base(1,1))>1.0)||...
                        (abs(deltaX_icp2base(2,1))>1.0)||...
                         (abs(deltaX_icp2base(3,1))>7)||...
                         (mod(abs(eulAng_base_cur(3)),3.14159)>(10/180*pi));
            
                            %(abs(Xn_pose_cur(3) - state_basePc(3))>0.15*abs(state_basePc(3)))
            
            %         if(flagOverlappingThreshold ||(mHRF < 1))
                    if(flagOverlappingThreshold)
                    %if(false)
                        % consecutive INS nominal states and point cloud
                        state_basePc = Xn_pose_cur;
                        state_base_ref = pose_ref;
                        mHRF_basePc = mHRF;
                        basePc = curPc_B_sampled;
                        first2base = 1;
                        first_scan = 1;
                        flagNewNode = 1;
                    end
        end
        % icp to last point cloud
        % -- compute initial guess of transformation matrix for ICP
        % from the base frame to the current frame        flagRegFinished = 1;
        q_pre = state_lastPc(4:7);
        q_pre_inv= [-q_pre(1) -q_pre(2) -q_pre(3) q_pre(4)];
        R_qpre_inv = [q_pre_inv(4) -q_pre_inv(3) q_pre_inv(2) q_pre_inv(1);
                q_pre_inv(3) q_pre_inv(4) -q_pre_inv(1) q_pre_inv(2);
                -q_pre_inv(2) q_pre_inv(1) q_pre_inv(4) q_pre_inv(3);
                -q_pre_inv(1) -q_pre_inv(2) -q_pre_inv(3) q_pre_inv(4)];

        % - rotational part
        q_d = R_qpre_inv * Xn_pose_cur(4:7);
        q_d = q_d/norm(q_d);
        R_base_cur = quat2rotmliub(q_d).';% rotation matrix
        
        % - translational part               
        R_base = quat2rotmliub(q_pre);
        p_d_L = Xn_pose_cur(1:3) - state_lastPc(1:3);
        p_d = R_base * p_d_L;
        
        params4last.initialTransform(1:3, 1:3) = R_base_cur;
        params4last.initialTransform(1:3,4) = p_d;
        deltaX_icp2last_g = [p_d;q_d];
            
        % -- compute reference relative transformation
        R_base_ref = eulAng2rotmliub(state_last_ref(4:6));
        deltaX2last_ref(1:3,1) = R_base_ref * (pose_ref(1:3) - state_last_ref(1:3));

        q_base_ref = eulAng2quatliub(state_last_ref(4:6));
        q_cur_ref = eulAng2quatliub(pose_ref(4:6));

        q_base_ref_inv = [-q_base_ref(1:3); q_base_ref(4)];
        deltaX2last_ref(4:7,1) = quatMultiplication(q_base_ref_inv, q_cur_ref);
        
        R_cur_ref = eulAng2rotmliub(pose_ref(4:6));
        R_base_cur_ref = R_cur_ref*R_base_ref';
        deltaX2last_ref(4:7,1) = rotm2quatliub(R_base_cur_ref);
        
        % -- compute ICP relative transformation
        if(flagUseTrueRelStates >0.5)
            deltaX_icp2last = deltaX2last_ref;
        else
            [T2last, erricp2last] = pcicpFL_LJF_V2_mex(curPc_B_sampled, lastPc, params4last);
            deltaX_icp2last(1:3,1) = T2last(1:3,4);
            deltaX_icp2last(4:7,1) = rotm2quatliub(T2last(1:3,1:3).');
        end        
        
        state_lastPc = Xn_pose_cur;
        state_last_ref = pose_ref;
        lastPc = curPc_B_sampled;
    end
	consec2base = first2base;
end

end