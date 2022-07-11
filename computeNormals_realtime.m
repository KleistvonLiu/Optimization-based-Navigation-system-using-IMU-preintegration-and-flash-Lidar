function normalsMN = computeNormals_realtime(pointsMN, paramNormal)
%Compute normals of point cloud in data structure MxNx3 for real-time ICP
% Version: 1.0      newly built
%       14.03.2021      compare results with pcnormals -> numerically the same
%
%

arrayDim = size(pointsMN,1);
normalsMN = zeros(arrayDim, arrayDim, 3);

pointsM3 = reshape(pointsMN, [],3);
idxArrayActive = find(paramNormal.arrayActive);
listPoints = pointsM3(idxArrayActive,:);


%use flash LiDAR special array structure FPA MxNx3
for i = 1:size(listPoints,1)

    idxPoint = idxArrayActive(i);
    point = listPoints(i,:);
    idxV = mod(idxPoint,arrayDim);%row
    if(idxV == 0), idxV = arrayDim;end
    idxU = ceil(idxPoint/arrayDim);%column

    %point = [pointsMN(idxV,idxU,1), pointsMN(idxV,idxU,2), pointsMN(idxV,idxU,3)];
    subOffset = floor(paramNormal.searchArrayDim/2);%

    idxV0 = max(idxV-subOffset,1);
    idxV1 = min(idxV+subOffset,arrayDim);
    idxU0 = max(idxU-subOffset,1);
    idxU1 = min(idxU+subOffset,arrayDim);

    subArrayPoints = pointsMN(idxV0:idxV1,idxU0:idxU1,:);
    subArrayActive = paramNormal.arrayActive(idxV0:idxV1,idxU0:idxU1);
    subListPoints = reshape(subArrayPoints, [], 3);
    subIdxArrayActive = find(subArrayActive);
    selectedPointsRaw = subListPoints(subIdxArrayActive,:);% subMDim*subMDim points

    diffVector = selectedPointsRaw - repmat(point,size(selectedPointsRaw,1),1);
    dist = sqrt(diffVector(:,1).^2 + diffVector(:,2).^2 + diffVector(:,3).^2);
    [sortDist, idxSortDist] = sort(dist);

    selectedPoints = zeros(paramNormal.numSPoints,3);
    if(paramNormal.useRadius)
        idxs = find(sortDist > paramNormal.radius);
        idx = idxs(1) - 1;
        if(idx < 6)
            selectedPoints = selectedPointsRaw(idxSortDist(1:paramNormal.numSPoints),:);
        else
            selectedPoints = selectedPointsRaw(idxSortDist(1:idx),:);
        end
    else
        idxSPoints = idxSortDist(1:paramNormal.numSPoints);
        selectedPoints = selectedPointsRaw(idxSPoints,:);
    end
    
    % use function pcacov()        
%     C = cov(selectedPoints);
%     % Solution 1 -> actual preference, as normal vector are primarily upwards
%     [P, lambda] = pcacov(C);
    
    % or use function pca()
    [P,~,lambda] = pca(selectedPoints);
    
    normals_temp = P(:,3);
    if(normals_temp(3) > 0)
        normals_temp = -normals_temp;
    end
    normalsMN(idxV,idxU,:) = normals_temp; % error when useRadius
end

