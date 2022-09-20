function [surfelMap_new_meanP, surfelMap_new_covar, surfelMap_new_numPsCell,...
    surfelMap_old_meanP, surfelMap_old_covar, surfelMap_old_numPsCell,...
    flagNewSurfelMap]=...
    SurfelGridMap(stateEst, flagStateUpdated,...
    fL1MeasArrayDir, fL1MeasArrayRange, fL1MeasFlagNewData, flagNewNode,...
    paramSGM, fL1Pose_B)

persistent SGM_intern; % surfelGridMap_intern
if(isempty(SGM_intern))
    SGM_intern.mapSize = paramSGM.mapSize; % x,y meters
    SGM_intern.gridResolution = paramSGM.gridResolution; % meter
    SGM_intern.gridSize = paramSGM.gridSize;
    SGM_intern.coorOffset = paramSGM.coorOffset; % [x y]
    SGM_intern.idxOffset = paramSGM.idxOffset;
    
    SGM_intern.meanPoints = zeros(paramSGM.gridSize(2), paramSGM.gridSize(1),3);
    SGM_intern.numPointsPerCell = zeros(paramSGM.gridSize(2),paramSGM.gridSize(1));
    SGM_intern.covar = zeros(paramSGM.gridSize(2),paramSGM.gridSize(1),6);% [a11 a12 a13 a22 a23 a33]
end

surfelMap_old_meanP = zeros(paramSGM.gridSize(2), paramSGM.gridSize(1),3);
surfelMap_old_numPsCell = zeros(paramSGM.gridSize(2),paramSGM.gridSize(1));
surfelMap_old_covar = zeros(paramSGM.gridSize(2),paramSGM.gridSize(1),6);% [a11 a12 a13 a22 a23 a33]

flagNewSurfelMap = 0;

% question! whether look at flagStateUpdate or flagfLMeas???
% if(flagStateUpdated < 0.5)
if(fL1MeasFlagNewData < 0.5)
	surfelMap_new_meanP = SGM_intern.meanPoints;
    surfelMap_new_numPsCell = SGM_intern.numPointsPerCell;
    surfelMap_new_covar = SGM_intern.covar;
else
	if(flagNewNode > 0.5)
% 	if(false)
        % old SGM
        surfelMap_old_meanP = SGM_intern.meanPoints;
        surfelMap_old_numPsCell = SGM_intern.numPointsPerCell;
        surfelMap_old_covar = SGM_intern.covar;

        % new node frame is declared, initialize new surfelGridMap
        SGM_intern.meanPoints = zeros(paramSGM.gridSize(2), paramSGM.gridSize(1),3);
        SGM_intern.numPointsPerCell = zeros(paramSGM.gridSize(2),paramSGM.gridSize(1));
        SGM_intern.covar = zeros(paramSGM.gridSize(2),paramSGM.gridSize(1),6);% [a11 a12 a13 a22 a23 a33]
        flagNewSurfelMap = 1;
	end
    
    downSample = false;
    if(downSample)
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
                range_ds(i,j) = sum(sum(fL1MeasArrayRange(i3-2:i3, j3-2:j3)))/ratio/ratio;
            end
        end
        % pc_base_S_ds = dirVector_ds .* range_ds;
        flashLidar1Points_temp(:,:,1) = dirVector_ds(:,:,1) .* range_ds;
        flashLidar1Points_temp(:,:,2) = dirVector_ds(:,:,2) .* range_ds;
        flashLidar1Points_temp(:,:,3) = dirVector_ds(:,:,3) .* range_ds;

    else
        fLArrayDim = size(fL1MeasArrayDir,1);
        flashLidar1Points_temp = zeros(fLArrayDim, fLArrayDim, 3);
        flashLidar1Points_temp(:,:,1) = fL1MeasArrayDir(:,:,1) .* fL1MeasArrayRange;
        flashLidar1Points_temp(:,:,2) = fL1MeasArrayDir(:,:,2) .* fL1MeasArrayRange;
        flashLidar1Points_temp(:,:,3) = fL1MeasArrayDir(:,:,3) .* fL1MeasArrayRange;
    end

    % current raw point cloud in size Mx3 in sensor unit frame
    curPc_U_raw = reshape(flashLidar1Points_temp,[],3);
    idxValidPixels = find(curPc_U_raw(:,1));
    % current point cloud with valid points
    curPc_U = curPc_U_raw(idxValidPixels,:);

    % -- transform point cloud from sensor unit frame to body frame
    fL1Att_Dcm_BU = quat2rotmliub(fL1Pose_B(4:7,1)); % rotation matrix from B to U
    
    % current point cloud in body frame
    curPc_B = curPc_U * fL1Att_Dcm_BU + repmat(fL1Pose_B(1:3,1)',size(curPc_U,1),1);
    numValidPoints = size(curPc_B,1);

    p_NB = stateEst(1:3,1);
    R_BN = quat2rotmliub(stateEst(7:10))'; % rotation from B to N
    % data points in the node frame
    dataPoints_N = curPc_B * R_BN' + repmat(p_NB',size(curPc_B,1),1);

    dataXyIndices = ceil(dataPoints_N(:,1:2) ./ SGM_intern.gridResolution);
    dataXyIndices_offset = dataXyIndices + repmat(SGM_intern.idxOffset,size(dataXyIndices,1),1);
    if(sum(dataXyIndices_offset<0,'all')>0)
       disp('points index in SGM is negative'); 
    end
    point2gridCell = dataXyIndices_offset(:,1)*SGM_intern.gridSize(2) + dataXyIndices_offset(:,2);

%     disp('merging data points into surfel grid map...');
    for i = 1:SGM_intern.gridSize(1)
       for j = 1:SGM_intern.gridSize(2)
            % surfel in the cell (j,i)
            mapMean_ji = squeeze(SGM_intern.meanPoints(j,i,:));
            mapNumPoints = SGM_intern.numPointsPerCell(j,i);
            covarValues = squeeze(SGM_intern.covar(j,i,:));
            mapCov_ji = [covarValues(1) covarValues(2) covarValues(3);...
                        covarValues(2) covarValues(4) covarValues(5);...
                        covarValues(3) covarValues(5) covarValues(6)];

            % data points in the cell (j,i)
            groupIdx = (i-1) * SGM_intern.gridSize(2) + j;
            indexSelected = find(point2gridCell == groupIdx);
            if(size(indexSelected,1)<5)
               continue;
            end
            dataPoints_ji = dataPoints_N(indexSelected,:);
            dataNumPoints = size(dataPoints_ji,1);
            dataMean_ji = mean(dataPoints_ji);%3x1
            dataCov_ji = cov(dataPoints_ji);

            % merge mapMean_ji dataMean_ji mapCov_ji dataCov_ji
            newMapMean_ji = zeros(1,1,3);
            if(mapNumPoints <1)
               % cell not detected before 
                newMapMean_ji(1,1,1:3) = dataMean_ji;
                sumPoints = dataNumPoints;
                newMapCov_ji = dataCov_ji;
            else
                % update cell
                sumPoints = mapNumPoints + dataNumPoints;

                % --- method 1
%                 newMapMean_ji(1,1,1:3) =...
%                     (mapNumPoints*mapMean_ji + dataNumPoints*dataMean_ji')/sumPoints;
%                 newMapCov_ji = ...
%                     (mapNumPoints^2*mapCov_ji + dataNumPoints^2*dataCov_ji)/sumPoints^2;

                % --- method 2
                % Question: why this method is closer to simple
                % combination method than method 1?
                newMapMean=...
                    (mapNumPoints*mapMean_ji' + dataNumPoints*dataMean_ji)/sumPoints;
                newMapMean_ji(1,1,1:3) = newMapMean;% 1x3
                newMapCov_ji =...
                    (mapNumPoints*(mapCov_ji + mapMean_ji*mapMean_ji') +...
                        dataNumPoints*(dataCov_ji + dataMean_ji'*dataMean_ji) )/...
                        sumPoints - newMapMean'*newMapMean;
            end


            % ! becareful j = y and i = x
            SGM_intern.meanPoints(j,i,:) = newMapMean_ji;
            SGM_intern.numPointsPerCell(j,i) = sumPoints;
            SGM_intern.covar(j,i,:) =...
                    [newMapCov_ji(1,1),newMapCov_ji(1,2),newMapCov_ji(1,3),...
                                        newMapCov_ji(2,2),newMapCov_ji(2,3),...
                                                            newMapCov_ji(3,3)];
       end
    end
    surfelMap_new_meanP = SGM_intern.meanPoints;
    surfelMap_new_numPsCell = SGM_intern.numPointsPerCell;
    surfelMap_new_covar = SGM_intern.covar;
    
    surfelMap_out = SGM_intern;

end
end