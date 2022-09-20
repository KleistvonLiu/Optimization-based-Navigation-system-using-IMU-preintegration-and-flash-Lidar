


posi_NB_N_est = squeeze(stateEstimates.signals.values(1:3,:,:));
vel_NB_N_est = squeeze(stateEstimates.signals.values(4:6,:,:));
quat_NB_est = squeeze(stateEstimates.signals.values(7:10,:,:));
eulAng_NB_est = zeros(3,size(quat_NB_est,2));

estXn_old_node = zeros(9,numStepSim);

acc_N_est = get(logsout,'acc_N');
acc_N_est = squeeze(acc_N_est.Values.data(:,:,:));
angRate_BN_B_est = get(logsout,'angRate_BN_B');
angRate_BN_B_est = squeeze(angRate_BN_B_est.Values.data(:,:,:));
posi_N1N2_est = get(logsout,'posi_N1N2');
posi_N1N2_est = squeeze(posi_N1N2_est.Values.data(:,:,:));% 3xn
posi_LN_est = get(logsout,'posi_LN');
posi_LN_est = squeeze(posi_LN_est.Values.data(:,:,:));% 3xn
R_LN_est = get(logsout,'R_LN');
R_LN_est = squeeze(R_LN_est.Values.data(:,:,:));% 3xn
flagRegFinished_est = get(logsout,'flagRegFinished');
flagRegFinished_est = flagRegFinished_est.Values.data;
flagNewNode_est = get(logsout,'flagNewNode');
flagNewNode_est = flagNewNode_est.Values.data;
flagStateUpdated_est = get(logsout,'flagStateUpdated');
flagStateUpdated_est = flagStateUpdated_est.Values.data;
residual = get(logsout,'residual');
residual = squeeze(residual.Values.data(:,:,:));% 6xn
measmHRF = get(logsout,'mHRF');
measmHRF = squeeze(measmHRF.Values.data(:,:));% 6xn
numValidPoints = get(logsout,'numValidPoints');
numValidPoints = squeeze(numValidPoints.Values.data(:,:));% 6xn


eulAng_LN_est = zeros(3,numStepSim);
eulAng_N1N2_est = zeros(3,numStepSim);
eulAng_N_fixed_est = zeros(3,numStepSim);
deltaX_icp_est = zeros(6,numStepSim);
deltaX_guess_est = zeros(6,numStepSim);
deltaX_ref = zeros(6,numStepSim);

posi_LB_L_est = zeros(3,numStepSim);
eulAng_LB_est = zeros(3,numStepSim);

SGM_meanPs = get(logsout,'surfelMap_meanP_out');
SGM_meanP = SGM_meanPs.Values.data(:,:,:,numStepSim);
SGM_covars = get(logsout,'surfelMap_covar_out');
SGM_covar = SGM_covars.Values.data(:,:,:,numStepSim);
SGM_numPsCells = get(logsout,'surfelMap_numPsCell_out');
SGM_numPsCell = SGM_numPsCells.Values.data(:,:,numStepSim);

SGM_meanP_old = get(logsout,'surfelMap_meanP_oldout');
SGM_meanP_old = SGM_meanP_old.Values.data;
SGM_covar_old = get(logsout,'surfelMap_covar_oldout');
SGM_covar_old = SGM_covar_old.Values.data;
SGM_numPsCell_old = get(logsout,'surfelMap_numPsCell_oldout');
SGM_numPsCell_old = SGM_numPsCell_old.Values.data;

posi_LB_RNref = zeros(3,numStepSim);
vel_LB_RNref = zeros(3,numStepSim);
eulAng_NB_RNref = zeros(3,numStepSim);

posi_LN_ref = get(logsout,'posi_LN_ref');
posi_LN_ref = squeeze(posi_LN_ref.Values.data(:,:,:));% 3xn
R_LN_ref = get(logsout,'R_LN_ref');
R_LN_ref = squeeze(R_LN_ref.Values.data(:,:,:));% 3xn

% p_Lnode_ref = zeros(3,1);
% R_Lnode_ref = eye(3);
% p_Lnode_ref = posi_LB_L_ref(1,:)';
% R_Lnode_ref = eulAng2rotmliub(eulAng_LB_ref(1,:)');
p_Lnode_ref = posi_LB_L_ref(1,:)';
R_Lnode_ref = eulAng2rotmliub(eulAng_LB_ref(1,:)');

if(flagGravityAlignedNode > 0.5)
    %effective-gravity aligned node
    eulAng_N1N2_1 = [0; 0; eulAng_LB_ref(1,3)];
    R_N1N2_1 = eulAng2rotmliub(eulAng_N1N2_1);

    dir_N1 = [0 0 1]';
    effGrav_T = gravity_input.data(1,:)' - cross(angRate_IT_T,cross(angRate_IT_T,posi_TL_T + R_TL' * posi_LB_L_ref(1,:)'));
    effGrav_N1 = eye(3) * R_TL * effGrav_T;
    dir_Grav = effGrav_N1/norm(effGrav_N1);
    rotAng = acos(dot(dir_N1, dir_Grav));
    rotaxis_temp = cross(dir_Grav, dir_N1);              
    rotaxis = rotaxis_temp / norm(rotaxis_temp);
    quat_N2_1Grav = [rotaxis * sin(rotAng/2); cos(rotAng/2)];
    R_N2_1Grav = quat2rotmliub(quat_N2_1Grav);
    R_N1N2 = R_N2_1Grav * R_N1N2_1;
    
    R_Lnode_ref = R_N1N2 * eye(3); % rotm from L to new N frame   
end


p_Lnode_est = p_Lnode_ref;
R_Lnode_est = R_Lnode_ref;

for idxTime = 1:(numStepSim-1)
    eulAng_NB_est(:,idxTime) = quat2eulAngliub(quat_NB_est(:,idxTime)');
    eulAng_LN_est(:,idxTime) = rotm2eulAngliub(R_LN_est(:,:,idxTime));

	R_NN_est = get(logsout,'R_N1N2');
    R_temp = R_NN_est.Values.data(:,:,idxTime);
    eulAng_N1N2_est(:,idxTime) = rotm2eulAngliub(R_temp);
    
    R_N_fixed_est = get(logsout,'R_N_fixed');
 	R_temp = R_N_fixed_est.Values.data(:,:,idxTime);
    eulAng_N_fixed_est(:,idxTime) = rotm2eulAngliub(R_temp);
    
%     estXnoldnode = get(logsout,'Xn_old_node');
% 	estXn_old_node(1:6,idxTime) = estXnoldnode.Values.data(1:6,:,idxTime);
% 	q_temp = estXnoldnode.Values.data(7:10,:,idxTime);
% 	R_temp = quat2rotmliub(q_temp);
% 	estXn_old_node(7:9,idxTime) = rad2deg(rotm2eulAngliub(R_temp));
    
    if((idxTime > dtSimFLRatio)&&(mod(idxTime,dtSimFLRatio)==1))
        if(flagPILTest)
            deltaX_icp_est_out = get(logsout,'deltaX_ICP');
            deltaX_icp_est(1:3,idxTime) =...
                squeeze(deltaX_icp_est_out.Values.data(1:3,:,(floor(idxTime/100)+1)));
            deltaq_temp = squeeze(deltaX_icp_est_out.Values.data(4:7,:,(floor(idxTime/100)+1)));
            deltaR_temp = quat2rotmliub(deltaq_temp);
            deltaX_icp_est(4:6,idxTime) = rad2deg(rotm2eulAngliub(deltaR_temp));

%             deltaX_guess_est_out = get(logsout,'deltaX_g');
%             deltaX_guess_est(1:3,idxTime) =...
%                 deltaX_guess_est_out.Values.data(1:3,:,idxTime);
%             deltaq_temp = deltaX_guess_est_out.Values.data(4:7,:,idxTime);
%             deltaR_temp = quat2rotmliub(deltaq_temp);
%             deltaX_guess_est(4:6,idxTime) = rad2deg(rotm2eulAngliub(deltaR_temp));

            deltaX_ref_out = get(logsout,'deltaX_ref');
            deltaX_ref(1:3,idxTime) =...
                deltaX_ref_out.Values.data(idxTime,1:3);
            deltaq_temp = deltaX_ref_out.Values.data(idxTime,4:7);
            deltaX_ref(4:6,idxTime) = quat2eulAngliub(deltaq_temp');
            
        else
            deltaX_icp_est_out = get(logsout,'deltaX_ICP');
            deltaX_icp_est(1:3,idxTime) =...
                deltaX_icp_est_out.Values.data(1:3,:,idxTime);
            deltaq_temp = deltaX_icp_est_out.Values.data(4:7,:,idxTime);
            deltaR_temp = quat2rotmliub(deltaq_temp);
            deltaX_icp_est(4:6,idxTime) = rad2deg(rotm2eulAngliub(deltaR_temp));

            deltaX_guess_est_out = get(logsout,'deltaX_g');
            deltaX_guess_est(1:3,idxTime) =...
                deltaX_guess_est_out.Values.data(1:3,:,idxTime);
            deltaq_temp = deltaX_guess_est_out.Values.data(4:7,:,idxTime);
            deltaR_temp = quat2rotmliub(deltaq_temp);
            deltaX_guess_est(4:6,idxTime) = rad2deg(rotm2eulAngliub(deltaR_temp));

            deltaX_ref_out = get(logsout,'deltaX_ref');
            deltaX_ref(1:3,idxTime) =...
                deltaX_ref_out.Values.data(1:3,:,idxTime);
            deltaq_temp = deltaX_ref_out.Values.data(4:7,:,idxTime);
            deltaX_ref(4:6,idxTime) = quat2eulAngliub(deltaq_temp);
        end

    end
    
    posi_LB_RNref(:,idxTime) = R_Lnode_ref * (posi_LB_L_ref(idxTime*dtSimRatio,:)' - p_Lnode_ref);
    vel_LB_RNref(:,idxTime) = R_Lnode_ref * vel_LB_L_ref(idxTime*dtSimRatio,:)';
    
    R_cur = eulAng2rotmliub(eulAng_LB_ref(idxTime*dtSimRatio,:)');
    R_NB_cur = R_cur * (R_Lnode_ref');
    eulAng_NB_RNref(:,idxTime) = rotm2eulAngliub(R_NB_cur);
    
	posi_LB_L_est(:,idxTime) = p_Lnode_est + (R_Lnode_est') * posi_NB_N_est(:,idxTime);
    R_NB_temp = eulAng2rotmliub(eulAng_NB_est(:,idxTime));
    R_LB_temp = R_NB_temp * R_Lnode_est;
	eulAng_LB_est(:,idxTime) = rotm2eulAngliub(R_LB_temp);

    if((idxTime<numStepSim) && flagNewNode_est(idxTime+1))
%         p_Lnode_ref = posi_LN_ref(:,idxTime+1);
%         R_Lnode_ref = R_LN_ref(:,:,idxTime+1);
        
        p_Lnode_ref = posi_LB_L_ref(idxTime*dtSimRatio,:)';
        
        if(flagGravityAlignedNode < 0.5)
            R_Lnode_ref = eulAng2rotmliub([0;0;eulAng_LB_ref(idxTime*dtSimRatio, 3)]);
        else
            R_LB = eulAng2rotmliub(eulAng_LB_ref(idxTime*dtSimRatio, :)');
            R_N1B = R_LB * (R_Lnode_ref');
            eulAng_N1B = rotm2eulAngliub(R_N1B);

            R_N1N2_1 = eulAng2rotmliub([0 0 eulAng_N1B(3)]);

            dir_N1 = [0 0 1]';
            effGrav_T = gravity_input.data(idxTime*dtSimRatio,:)' - cross(angRate_IT_T,cross(angRate_IT_T,posi_TL_T + R_TL' * posi_LB_L_ref(idxTime*dtSimRatio,:)'));
            effGrav_N1 = R_Lnode_ref * R_TL * effGrav_T;
            dir_Grav = effGrav_N1/norm(effGrav_N1);
            rotAng = acos(dot(dir_N1, dir_Grav));
            rotaxis_temp = cross(dir_Grav, dir_N1);
            rotaxis = rotaxis_temp / norm(rotaxis_temp);
            quat_N2_1Grav = [rotaxis * sin(rotAng/2); cos(rotAng/2)];
            R_N2_1Grav = quat2rotmliub(quat_N2_1Grav);
            R_N1N2 = R_N2_1Grav * R_N1N2_1;

            R_Lnode_new = R_N1N2 * R_Lnode_ref;
            R_Lnode_ref = R_Lnode_new;
        end

        p_Lnode_est = posi_LN_est(:,idxTime+1);
        R_Lnode_est = R_LN_est(:,:,idxTime+1);    
    end
end

%% --- visualization
set(0,'DefaultFigureWindowStyle','docked')

tSim = tSeries(1:numStepSim);

% ====================================
h(1) = figure('Name','Position');
subplot(3,2,1)
plot(tSim, posi_NB_N_est(1,:),'b'); hold on;grid on;
% plot(tSim, posi_LB_L_ref.data(1,1:numStepSim),':r'); hold on;grid on;
plot(tSim, posi_LB_RNref(1,1:numStepSim),':r'); hold on;grid on;
title('Position Estimation');
ylabel('x [m]','FontSize',50);set(gca,'FontSize',15);
legend('R.Navigation','Ref. in L frame')
subplot(3,2,3)
plot(tSim, posi_NB_N_est(2,:),'b'); hold on;grid on;
% plot(tSim, posi_LB_L_ref.data(2,1:numStepSim),':r'); hold on;grid on;
plot(tSim, posi_LB_RNref(2,1:numStepSim),':r'); hold on;grid on;
legend('R.Navigation','Ref. in L frame')
ylabel('y [m]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,5)
plot(tSim, posi_NB_N_est(3,:),'b'); hold on;grid on;
% plot(tSim, posi_LB_L_ref.data(3,1:numStepSim),':r'); hold on;grid on;
plot(tSim, posi_LB_RNref(3,1:numStepSim),':r'); hold on;grid on;
legend('R.Navigation','Ref. in L frame')
ylabel('z [m]','FontSize',50);set(gca,'FontSize',15);
xlabel('time [s]','FontSize',20);
subplot(3,2,2)
% plot(tSim, estPos_BN(1,:)-posi_LB_L_ref.data(1,1:numStepSim)); hold on;grid on;
plot(tSim, posi_NB_N_est(1,:)-posi_LB_RNref(1,1:numStepSim)); hold on;grid on;
title('Position errors');
ylabel('x (nav-ref) [m]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,4)
% plot(tSim, estPos_BN(2,:)-posi_LB_L_ref.data(2,1:numStepSim)); hold on;grid on;
plot(tSim, posi_NB_N_est(2,:)-posi_LB_RNref(2,1:numStepSim)); hold on;grid on;
ylabel('y (nav-ref) [m]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,6)
% plot(tSim, estPos_BN(3,:)-posi_LB_L_ref.data(3,1:numStepSim)); hold on;grid on;
plot(tSim, posi_NB_N_est(3,:)-posi_LB_RNref(3,1:numStepSim)); hold on;grid on;
ylabel('z (nav-ref) [m]','FontSize',50);set(gca,'FontSize',15);
xlabel('time [s]','FontSize',20);

% ====================================
h(2) = figure('Name','Velocity');
subplot(3,2,1)
plot(tSim, vel_NB_N_est(1,:),'b'); hold on;grid on;
plot(tSim, vel_LB_RNref(1,1:numStepSim),':r'); hold on;grid on;
title('Velocity Estimation');
ylabel('x [m/s]');
legend('R.Navigation','Ref. in L frame')
subplot(3,2,3)
plot(tSim, vel_NB_N_est(2,:),'b'); hold on;grid on;
plot(tSim, vel_LB_RNref(2,1:numStepSim),':r'); hold on;grid on;
ylabel('y [m/s]');
legend('R.Navigation','Ref. in L frame')
subplot(3,2,5)
plot(tSim, vel_NB_N_est(3,:),'b'); hold on;grid on;
plot(tSim, vel_LB_RNref(3,1:numStepSim),':r'); hold on;grid on;
ylabel('z [m/s]');
legend('R.Navigation','Ref. in L frame')
xlabel('time [s]');

subplot(3,2,2)
plot(tSim, vel_NB_N_est(1,:)-vel_LB_RNref(1,1:numStepSim)); hold on;grid on;
title('Velocity error in L-frame');
ylabel('x (nav-ref) [m/s]');
subplot(3,2,4)
plot(tSim, vel_NB_N_est(2,:)-vel_LB_RNref(2,1:numStepSim)); hold on;grid on;
ylabel('y (nav-ref) [m/s]');
subplot(3,2,6)
plot(tSim, vel_NB_N_est(3,:)-vel_LB_RNref(3,1:numStepSim)); hold on;grid on;
ylabel('z (nav-ref) [m/s]');

% ====================================
h(3) = figure('Name','Acceleration in N frame');
subplot(3,2,1)
plot(tSim, acc_N_est(1,:),'b'); hold on;grid on;
% plot(tSim, refRN_Vel_BL(1,1:numStepSim),':r'); hold on;grid on;
title('Acc. in N-frame');
ylabel('x [m/s^2]');
% legend('Navigation','Reference')
subplot(3,2,3)
plot(tSim, acc_N_est(2,:),'b'); hold on;grid on;
% plot(tSim, refRN_Vel_BL(2,1:numStepSim),':r'); hold on;grid on;
ylabel('y [m/s^2]');
% legend('Navigation','Reference')
subplot(3,2,5)
plot(tSim, acc_N_est(3,:),'b'); hold on;grid on;
% plot(tSim, refRN_Vel_BL(3,1:numStepSim),':r'); hold on;grid on;
ylabel('z [m/s^2]');
% legend('Navigation','Reference')
xlabel('time [s]');

% ====================================
h(4) = figure('Name','Attitude Eul. angles');
subplot(3,2,1)
plot(tSim, rad2deg(eulAng_NB_est(1,:)),'b'); hold on;grid on;
% plot(tSim, rad2deg(refAtt_eul_BL(1,1:numStepSim)),':r'); hold on;grid on;
plot(tSim, rad2deg(eulAng_NB_RNref(1,1:numStepSim)),':r'); hold on;grid on;
title('Attitude Euler angles estimation');
ylabel('phi [deg]','FontSize',50);set(gca,'FontSize',15);
legend('R.Navigation','Ref. in L frame')
subplot(3,2,3)
plot(tSim, rad2deg(eulAng_NB_est(2,:)),'b'); hold on;grid on;
% plot(tSim, rad2deg(refAtt_eul_BL(2,1:numStepSim)),':r'); hold on;grid on;
plot(tSim, rad2deg(eulAng_NB_RNref(2,1:numStepSim)),':r'); hold on;grid on;
ylabel('theta [deg]','FontSize',50);set(gca,'FontSize',15);
legend('R.Navigation','Ref. in L frame')
subplot(3,2,5)
plot(tSim, rad2deg(eulAng_NB_est(3,:)),'b'); hold on;grid on;
% plot(tSim, rad2deg(refAtt_eul_BL(3,1:numStepSim)),':r'); hold on;grid on;
plot(tSim, rad2deg(eulAng_NB_RNref(3,1:numStepSim)),':r'); hold on;grid on;
ylabel('psi [deg]','FontSize',50);set(gca,'FontSize',15);
legend('R.Navigation','Ref. in L frame')
xlabel('time [s]','FontSize',20);

subplot(3,2,2)
% plot(tSim, estAtt_eul_BN(1,:)-rad2deg(refAtt_eul_BL(1,1:numStepSim))); hold on;grid on;
plot(tSim, rad2deg(eulAng_NB_est(1,:)-eulAng_NB_RNref(1,1:numStepSim))); hold on;grid on;
title('Euler angles error');
ylabel('phi (nav-ref) [deg]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,4)
% plot(tSim, rad2deg(eulAng_NB_est(2,:)-eulAng_NB_RNref(2,1:numStepSim))); hold on;grid on;
plot(tSim, -100*rad2deg(eulAng_NB_est(1,:)+eulAng_NB_est(3,:)-eulAng_NB_RNref(1,1:numStepSim)-eulAng_NB_RNref(3,1:numStepSim))/2); hold on;grid on;
ylabel('theta (nav-ref) [deg]','FontSize',50);set(gca,'FontSize',15);
subplot(3,2,6)
% plot(tSim, estAtt_eul_BN(3,:)-rad2deg(refAtt_eul_BL(3,1:numStepSim))); hold on;grid on;
plot(tSim, rad2deg(eulAng_NB_est(3,:)-eulAng_NB_RNref(3,1:numStepSim))); hold on;grid on;
ylabel('psi (nav-ref) [deg]','FontSize',50);set(gca,'FontSize',15);
xlabel('time [s]','FontSize',20);

% ====================================
h(5) = figure('Name','Angular rates NB in B');
subplot(3,2,1)
plot(tSim, angRate_BN_B_est(1,:),'b'); hold on;grid on;
% plot(tSim, refAtt_eul_BL(1,1:numStepSim),':r'); hold on;grid on;
title('INS attitude Euler angles wrt L-frame');
ylabel('phi [deg]');
% legend('Navigation','Reference')
subplot(3,2,3)
plot(tSim, angRate_BN_B_est(2,:),'b'); hold on;grid on;
% plot(tSim, refAtt_eul_BL(2,1:numStepSim),':r'); hold on;grid on;
ylabel('theta [deg]');
% legend('Navigation','Reference')
subplot(3,2,5)
plot(tSim, angRate_BN_B_est(3,:),'b'); hold on;grid on;
% plot(tSim, refAtt_eul_BL(3,1:numStepSim),':r'); hold on;grid on;
ylabel('psi [deg]');
% legend('Navigation','Reference')
xlabel('time [s]');

subplot(3,2,2)
% plot(tSim, estAtt_eul_BN(1,:)-refAtt_eul_BL(1,1:numStepSim)); hold on;grid on;
title('Euler angles error');
ylabel('phi (nav-ref) [deg]');
subplot(3,2,4)
% plot(tSim, estAtt_eul_BN(2,:)-refAtt_eul_BL(2,1:numStepSim)); hold on;grid on;
ylabel('theta (nav-ref) [deg]');
subplot(3,2,6)
% plot(tSim, estAtt_eul_BN(3,:)-refAtt_eul_BL(3,1:numStepSim)); hold on;grid on;
ylabel('psi (nav-ref) [deg]');
xlabel('time [s]');

% ====================================

h(6) = figure('Name','DeltaX ICP');
subplot(3,2,1)
plot(tSim, deltaX_ref(1,:),'g'); hold on;grid on;
plot(tSim, deltaX_guess_est(1,:),'r'); hold on;grid on;
plot(tSim, deltaX_icp_est(1,:),'b'); hold on;grid on;
title('DeltaX in position. g-ref r-guess b-est');
ylabel('x [m]');
subplot(3,2,3)
plot(tSim, deltaX_ref(2,:),'g'); hold on;grid on;
plot(tSim, deltaX_guess_est(2,:),'r'); hold on;grid on;
plot(tSim, deltaX_icp_est(2,:),'b'); hold on;grid on;
ylabel('y [m]');
subplot(3,2,5)
plot(tSim, deltaX_ref(3,:),'g'); hold on;grid on;
plot(tSim, deltaX_guess_est(3,:),'r'); hold on;grid on;
plot(tSim, deltaX_icp_est(3,:),'b'); hold on;grid on;
ylabel('z [m]');
xlabel('time [s]');

subplot(3,2,2)
plot(tSim, deltaX_ref(4,:),'g'); hold on;grid on;
plot(tSim, deltaX_guess_est(4,:),'r'); hold on;grid on;
plot(tSim, deltaX_icp_est(4,:),'b'); hold on;grid on;
title('DeltaX in Euler angles. g-ref r-guess b-est');
ylabel('x [deg]');
subplot(3,2,4)
plot(tSim, deltaX_ref(5,:),'g'); hold on;grid on;
plot(tSim, deltaX_guess_est(5,:),'r'); hold on;grid on;
plot(tSim, deltaX_icp_est(5,:),'b'); hold on;grid on;
ylabel('y [deg]');
subplot(3,2,6)
plot(tSim, deltaX_ref(6,:),'g'); hold on;grid on;
plot(tSim, deltaX_guess_est(6,:),'r'); hold on;grid on;
plot(tSim, deltaX_icp_est(6,:),'b'); hold on;grid on;
ylabel('z [deg]');
xlabel('time [s]');


% ====================================

h(7) = figure('Name','DeltaX Errors');
subplot(3,2,1)
plot(tSim, deltaX_guess_est(1,:)-deltaX_ref(1,:),'og'); hold on;grid on;
plot(tSim, deltaX_icp_est(1,:)-deltaX_ref(1,:),'.b'); hold on;grid on;
title('Errors in position (g-refguess,b-refest)');
ylabel('x [m]');
subplot(3,2,3)
plot(tSim, deltaX_guess_est(2,:)-deltaX_ref(2,:),'og'); hold on;grid on;
plot(tSim, deltaX_icp_est(2,:)-deltaX_ref(2,:),'.b'); hold on;grid on;
ylabel('y [m]');
subplot(3,2,5)
plot(tSim, deltaX_guess_est(3,:)-deltaX_ref(3,:),'og'); hold on;grid on;
plot(tSim, deltaX_icp_est(3,:)-deltaX_ref(3,:),'.b'); hold on;grid on;
ylabel('z [m]');
xlabel('time [s]');

subplot(3,2,2)
plot(tSim, deltaX_guess_est(4,:)-deltaX_ref(4,:),'og'); hold on;grid on;
plot(tSim, deltaX_icp_est(4,:)-deltaX_ref(4,:),'.b'); hold on;grid on;
title('Errors in Euler angles (g-refguess,b-refest)');
ylabel('x [deg]');
subplot(3,2,4)
plot(tSim, deltaX_guess_est(5,:)-deltaX_ref(5,:),'og'); hold on;grid on;
plot(tSim, deltaX_icp_est(5,:)-deltaX_ref(5,:),'.b'); hold on;grid on;
ylabel('y [deg]');
subplot(3,2,6)
plot(tSim, deltaX_guess_est(6,:)-deltaX_ref(6,:),'og'); hold on;grid on;
plot(tSim, deltaX_icp_est(6,:)-deltaX_ref(6,:),'.b'); hold on;grid on;
ylabel('z [deg]');
xlabel('time [s]');




% ====================================
h(8) = figure('Name','Node to node');
subplot(3,2,1)
plot(tSim, posi_N1N2_est(1,:),'b'); hold on;grid on;
title('Node to node position');
ylabel('x [m]');
subplot(3,2,3)
plot(tSim, posi_N1N2_est(2,:),'b'); hold on;grid on;
ylabel('y [m]');
subplot(3,2,5)
plot(tSim, posi_N1N2_est(3,:),'b'); hold on;grid on;
ylabel('z [m]');
xlabel('time [s]');

subplot(3,2,2)
plot(tSim, rad2deg(eulAng_N1N2_est(1,:)),'b'); hold on;grid on;
title('Node to node Euler angles');
ylabel('x [deg]');
subplot(3,2,4)
plot(tSim, rad2deg(eulAng_N1N2_est(2,:)),'b'); hold on;grid on;
ylabel('y [deg]');
subplot(3,2,6)
plot(tSim, rad2deg(eulAng_N1N2_est(3,:)),'b'); hold on;grid on;
ylabel('z [deg]');
xlabel('time [s]');


% ====================================
h(9) = figure('Name','Eul.Angle node to fixed frame');
subplot(3,2,1)
plot(tSim, rad2deg(eulAng_N_fixed_est(1,:)),'b'); hold on;grid on;
title('Node to fixed frame');
ylabel('x [deg]');
subplot(3,2,3)
plot(tSim, rad2deg(eulAng_N_fixed_est(2,:)),'b'); hold on;grid on;
ylabel('y [deg]');
subplot(3,2,5)
plot(tSim, rad2deg(eulAng_N_fixed_est(3,:)),'b'); hold on;grid on;
ylabel('z [deg]');
xlabel('time [s]');

subplot(3,2,2)

subplot(3,2,4)

subplot(3,2,6)

% ====================================
h(10) = figure('Name','L to node frame');
subplot(3,2,1)
plot(tSim, posi_LN_est(1,:),'b'); hold on;grid on;
title('Position from L to n');
ylabel('x [m]');
subplot(3,2,3)
plot(tSim, posi_LN_est(2,:),'b'); hold on;grid on;
ylabel('y [m]');
subplot(3,2,5)
plot(tSim, posi_LN_est(3,:),'b'); hold on;grid on;
ylabel('z [m]');
xlabel('time [s]');

subplot(3,2,2)
plot(tSim, rad2deg(eulAng_LN_est(1,:)),'b'); hold on;grid on;
title('Euler angles from L to n');
ylabel('x [deg]');
subplot(3,2,4)
plot(tSim, rad2deg(eulAng_LN_est(2,:)),'b'); hold on;grid on;
ylabel('y [deg]');
subplot(3,2,6)
plot(tSim, rad2deg(eulAng_LN_est(3,:)),'b'); hold on;grid on;
ylabel('z [deg]');
xlabel('time [s]');

% ====================================
h(11) = figure('Name','residual');
subplot(3,2,1)
plot(tSim, residual(1,:),'b'); hold on;grid on;
title('Position in residual');
ylabel('x [m]');
subplot(3,2,3)
plot(tSim, residual(2,:),'b'); hold on;grid on;
ylabel('y [m]');
subplot(3,2,5)
plot(tSim, residual(3,:),'b'); hold on;grid on;
ylabel('z [m]');
xlabel('time [s]');

subplot(3,2,2)
plot(tSim, residual(4,:),'b'); hold on;grid on;
title('Euler angles in residual');
ylabel('x [deg]');
subplot(3,2,4)
plot(tSim, residual(5,:),'b'); hold on;grid on;
ylabel('y [deg]');
subplot(3,2,6)
plot(tSim, residual(6,:),'b'); hold on;grid on;
ylabel('z [deg]');
xlabel('time [s]');

% ====================================
h(12) = figure('Name','Others');
subplot(3,2,1)
plot(tSim, flagRegFinished_est,'b'); hold on;grid on;
title('flag of registration finished');
ylabel('flag');

subplot(3,2,3)
plot(tSim, flagStateUpdated_est,'b'); hold on;grid on;
title('flag of state being updated');
ylabel('flag');

subplot(3,2,5)
plot(tSim, flagNewNode_est,'b'); hold on;grid on;
title('flag of new node configuration');
ylabel('flag');
xlabel('time [s]');

subplot(3,2,2)
if(flagPILTest)
    plot(tSim(1:100:end), measmHRF,'b'); hold on;grid on;
else
	plot(tSim, measmHRF,'b'); hold on;grid on; 
end
title('manual height range finder');
ylabel('m');
xlabel('time [s]');

subplot(3,2,4)
plot(tSim, numValidPoints,'b'); hold on;grid on;
title('Number of valid points from fL');
ylabel('num');
xlabel('time [s]');

subplot(3,2,6)
xlabel('time [s]');


% ====================================
h(13) = figure('Name','Global position');
subplot(3,2,1)
plot(tSim, posi_LB_L_est(1,:),'b'); hold on;grid on;
% plot(tSim, posi_LB_L_ref.data(1,1:numStepSim),':r'); hold on;grid on;
plot(tSim, posi_LB_L_ref(1:dtSimRatio:numStepSim*dtSimRatio,1)',':r'); hold on;grid on;
title('Position Estimation');
ylabel('x [m]');
legend('R.Navigation','Ref. in L frame')
subplot(3,2,3)
plot(tSim, posi_LB_L_est(2,:),'b'); hold on;grid on;
% plot(tSim, posi_LB_L_ref.data(2,1:numStepSim),':r'); hold on;grid on;
plot(tSim, posi_LB_L_ref(1:dtSimRatio:numStepSim*dtSimRatio,2)',':r'); hold on;grid on;
legend('R.Navigation','Ref. in L frame')
ylabel('y [m]');
subplot(3,2,5)
plot(tSim, posi_LB_L_est(3,:),'b'); hold on;grid on;
% plot(tSim, posi_LB_L_ref.data(3,1:numStepSim),':r'); hold on;grid on;
plot(tSim, posi_LB_L_ref(1:dtSimRatio:numStepSim*dtSimRatio,3)',':r'); hold on;grid on;
legend('R.Navigation','Ref. in L frame')
ylabel('z [m]');
xlabel('time [s]');

subplot(3,2,2)
% plot(tSim, estPos_BN(1,:)-posi_LB_L_ref.data(1,1:numStepSim)); hold on;grid on;
plot(tSim, posi_LB_L_est(1,:)-posi_LB_L_ref(1:dtSimRatio:numStepSim*dtSimRatio,1)'); hold on;grid on;
title('Position errors');
ylabel('x (nav-ref) [m]');
subplot(3,2,4)
% plot(tSim, estPos_BN(2,:)-posi_LB_L_ref.data(2,1:numStepSim)); hold on;grid on;
plot(tSim, posi_LB_L_est(2,:)-posi_LB_L_ref(1:dtSimRatio:numStepSim*dtSimRatio,2)'); hold on;grid on;
ylabel('y (nav-ref) [m]');
subplot(3,2,6)
% plot(tSim, estPos_BN(3,:)-posi_LB_L_ref.data(3,1:numStepSim)); hold on;grid on;
plot(tSim, posi_LB_L_est(3,:)-posi_LB_L_ref(1:dtSimRatio:numStepSim*dtSimRatio,3)'); hold on;grid on;
ylabel('z (nav-ref) [m]');


% ====================================
h(14) = figure('Name','Global attitude');
subplot(3,2,1)
plot(tSim, rad2deg(eulAng_LB_est(1,:)),'b'); hold on;grid on;
% plot(tSim, rad2deg(refAtt_eul_BL(1,1:numStepSim)),':r'); hold on;grid on;
plot(tSim, rad2deg(eulAng_LB_ref(1:dtSimRatio:numStepSim*dtSimRatio,1)'),':r'); hold on;grid on;
title('Attitude Euler angles estimation');
ylabel('phi [deg]');
legend('R.Navigation','Ref. in L frame')
subplot(3,2,3)
plot(tSim, rad2deg(eulAng_LB_est(2,:)),'b'); hold on;grid on;
% plot(tSim, rad2deg(refAtt_eul_BL(2,1:numStepSim)),':r'); hold on;grid on;
plot(tSim, rad2deg(eulAng_LB_ref(1:dtSimRatio:numStepSim*dtSimRatio,2)'),':r'); hold on;grid on;
ylabel('theta [deg]');
legend('R.Navigation','Ref. in L frame')
subplot(3,2,5)
plot(tSim, rad2deg(eulAng_LB_est(3,:)),'b'); hold on;grid on;
% plot(tSim, rad2deg(refAtt_eul_BL(3,1:numStepSim)),':r'); hold on;grid on;
plot(tSim, rad2deg(eulAng_LB_ref(1:dtSimRatio:numStepSim*dtSimRatio,3)'),':r'); hold on;grid on;
ylabel('psi [deg]');
legend('R.Navigation','Ref. in L frame')
xlabel('time [s]');

subplot(3,2,2)
% plot(tSim, estAtt_eul_BN(1,:)-rad2deg(refAtt_eul_BL(1,1:numStepSim))); hold on;grid on;
plot(tSim, rad2deg(eulAng_LB_est(1,:)-eulAng_LB_ref(1:dtSimRatio:numStepSim*dtSimRatio,1)')); hold on;grid on;
title('Euler angles error');
ylabel('phi (nav-ref) [deg]');
subplot(3,2,4)
% plot(tSim, estAtt_eul_BN(2,:)-rad2deg(refAtt_eul_BL(2,1:numStepSim))); hold on;grid on;
plot(tSim, rad2deg(eulAng_LB_est(2,:)-eulAng_LB_ref(1:dtSimRatio:numStepSim*dtSimRatio,2)')); hold on;grid on;
ylabel('theta (nav-ref) [deg]');
subplot(3,2,6)
% plot(tSim, estAtt_eul_BN(3,:)-rad2deg(refAtt_eul_BL(3,1:numStepSim))); hold on;grid on;
plot(tSim, rad2deg(eulAng_LB_est(3,:)-eulAng_LB_ref(1:dtSimRatio:numStepSim*dtSimRatio,3)')); hold on;grid on;
ylabel('psi (nav-ref) [deg]');
xlabel('time [s]');

return


%% SGM
h(15) = figure('Name','SGM');
colorSGM = 4;
SGM_meanPoints_list = reshape(SGM_meanP,[],3);
for i = 1:paramSGM.gridSize(1)
	for j = 1:paramSGM.gridSize(2)
        if (SGM_numPsCell(j,i)<10),continue;end
        meanP = squeeze(SGM_meanP(j,i,:));    
        covari = squeeze(SGM_covar(j,i,:));%[a11 a12 a13 a22 a23 a33]

        covMatrix = [covari(1) covari(2) covari(3);...
                    covari(2) covari(4) covari(5);...
                    covari(3) covari(5) covari(6)];
        plot_gaussian_ellipsoid(meanP, covMatrix, 2, 10, gca, colorSGM);hold on;
   end
end
% plot3(SGM_meanPoints_list(:,1),...
%     SGM_meanPoints_list(:,2),...
%     SGM_meanPoints_list(:,3),'ob');
title('Mean points(o)&Covariance in SGM');
daspect([1 1 1]);
xlabel('X');ylabel('Y');zlabel('Z');
% zlim([5 9]);
% for the "spacecraft body frame"
h(15).CurrentAxes.YDir = 'Reverse';
h(15).CurrentAxes.ZDir = 'Reverse';
h(15).CurrentAxes.XColor = 'red';
h(15).CurrentAxes.YColor = 'green';
h(15).CurrentAxes.ZColor = 'blue';
h(15).CurrentAxes.LineWidth = 3.0;
grid on;



% ====================================
h(16) = figure('Name','All SGMs');
kInstant = find(flagNewNode_est);
count = 1;
for k = 1:2:size(kInstant,1)
    disp('drawing SGM...');
    g = kInstant(k);
    p_l_n_g = posi_LN_est(:,g);
	R_l_n_g = R_LN_est(:,:,g);
    SGM_meanP_old_g = SGM_meanP_old(:,:,:,g);
    SGM_covar_old_g = SGM_covar_old(:,:,:,g);
    SGM_numPsCell_old_g = SGM_numPsCell_old(:,:,g);
 
    SGM_meanP_old_k_list = reshape(SGM_meanP_old_g,[],3);
    SGM_meanP_old_k_trans_list = SGM_meanP_old_k_list * R_l_n_g + p_l_n_g';
	SGM_meanP_old_k_trans = reshape(SGM_meanP_old_k_trans_list,size(SGM_meanP_old_g));  
    
    interval = 1;
%     if((count == 2)||(count == 4)||(count == 4)),count = count+1;continue;end
    for i = 1:interval:paramSGM.gridSize(1)
       for j = 1:interval:paramSGM.gridSize(2)
        if (SGM_numPsCell_old_g(j,i)<10),continue;end
        meanP = squeeze(SGM_meanP_old_k_trans(j,i,:));    
        covari = squeeze(SGM_covar_old_g(j,i,:));%[a11 a12 a13 a22 a23 a33]

        covMatrix = [covari(1) covari(2) covari(3);...
                    covari(2) covari(4) covari(5);...
                    covari(3) covari(5) covari(6)];
        plot_gaussian_ellipsoid(meanP, covMatrix, 2, 10,gca,count);hold on;
       end
    end
    count = count+1;
end
plot3(posi_LB_L_ref(1:numStepSim*dtSimRatio,1),...
    posi_LB_L_ref(1:numStepSim*dtSimRatio,2),...
    posi_LB_L_ref(1:numStepSim*dtSimRatio,3),'.r');grid on;
plot3(posi_LB_L_ref(kInstant*dtSimRatio,1),...
    posi_LB_L_ref(kInstant*dtSimRatio,2),...
    posi_LB_L_ref(kInstant*dtSimRatio,3),'*b');grid on;
title('All SGMs');
daspect([1 1 1]);
xlabel('X');ylabel('Y');zlabel('Z');
% xlim([0 25]);ylim([-8 8]);
% zlim([5 9]);
% for the "spacecraft body frame"
h(16).CurrentAxes.YDir = 'Reverse';
h(16).CurrentAxes.ZDir = 'Reverse';
h(16).CurrentAxes.XColor = 'red';
h(16).CurrentAxes.YColor = 'green';
h(16).CurrentAxes.ZColor = 'blue';
h(16).CurrentAxes.LineWidth = 3.0;
grid on;


% ====================================
set(0,'DefaultFigureWindowStyle','normal')

% figure(200)
% subplot(3,2,1)
% plot(tSim, estPos_BN(1,:),'b'); hold on;grid on;
% plot(tSim, refRN_Pos_BL(1,1:numStepSim),':r'); hold on;grid on;
% title('Position Estimation');
% ylabel('x [m]');
% legend('R.Navigation','Ref.')
% subplot(3,2,3)
% plot(tSim, estPos_BN(2,:),'b'); hold on;grid on;
% plot(tSim, refRN_Pos_BL(2,1:numStepSim),':r'); hold on;grid on;
% legend('R.Navigation','Ref.')
% ylabel('y [m]');
% subplot(3,2,5)
% plot(tSim, estPos_BN(3,:),'b'); hold on;grid on;
% plot(tSim, refRN_Pos_BL(3,1:numStepSim),':r'); hold on;grid on;
% legend('R.Navigation','Ref.')
% ylabel('z [m]');
% xlabel('time [s]');
% 
% subplot(3,2,2)
% plot(tSim, estAtt_eul_BN(1,:),'b'); hold on;grid on;
% plot(tSim, rad2deg(refRN_Att_eul_BL(1,1:numStepSim)),':r'); hold on;grid on;
% title('Attitude Euler angles estimation');
% ylabel('phi [deg]');
% legend('R.Navigation','Ref.')
% subplot(3,2,4)
% plot(tSim, estAtt_eul_BN(2,:),'b'); hold on;grid on;
% plot(tSim, rad2deg(refRN_Att_eul_BL(2,1:numStepSim)),':r'); hold on;grid on;
% ylabel('theta [deg]');
% legend('R.Navigation','Ref.')
% subplot(3,2,6)
% plot(tSim, estAtt_eul_BN(3,:),'b'); hold on;grid on;
% plot(tSim, rad2deg(refRN_Att_eul_BL(3,1:numStepSim)),':r'); hold on;grid on;
% ylabel('psi [deg]');
% legend('R.Navigation','Ref.')
% xlabel('time [s]');
% 


% saveas(h,'plots_SimRNSGM_result_09072022.fig');

