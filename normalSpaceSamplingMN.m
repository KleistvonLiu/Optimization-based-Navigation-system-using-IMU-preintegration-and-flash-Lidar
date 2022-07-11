function [sPixel,numSPoints] = normalSpaceSamplingMN(normalsMN, arrayActive, numToSelectedPoints)
%Normal space sampling with point cloud data structure MxNx3

% Version: 1.0
%          2.0      regarding only active pixels

%parameters
deltaTheta = 10 * pi/180; % rads
deltaPhi = 10 * pi/180; % rads
numSegTheta = 2*pi/deltaTheta; %36,attention!!!to be modified, not necessarily integer
numSegPhi = pi/deltaPhi; %18,attention!!!to be modified, not necessarily integer
numBuckets = numSegTheta * numSegPhi;
% Temp: set random seed
rng(9);

% initialization
arrayDim = size(normalsMN, 1);
sPixelNormals = false(arrayDim);
numSPoints = 0;

normalsM3 = reshape(normalsMN, [],3);
idxArrayActive = find(arrayActive);
listNormals = normalsM3(idxArrayActive,:);

%sphere coordinates of normalized normals {(MxN)x2}
sphereCoor = zeros(size(listNormals,1), 2);
sphereCoor(:,1) = atan2(listNormals(:,2),listNormals(:,1));
sphereCoor(:,2) = acos(listNormals(:,3));

%a M x N list indicating each point(index) in which bucket
%index of bucket = index in theta * numSegPhi + index in phi
points2Bucket = floor(sphereCoor(:,1)/deltaTheta) * numSegPhi + floor(sphereCoor(:,2)/deltaPhi);

%a list indicating how many points each bucket has
numPointsPerBucket = zeros(numBuckets,1);
for i = 1:numBuckets, numPointsPerBucket(i) = sum(points2Bucket==i);end

%a list indicating how many points shall be selected for each bucket
%according to numSelectedPoints
numSelectedPointsPerBucket = zeros(numBuckets,1);
pointCounter = 0;
exitCounter = 0;
while numSPoints < numToSelectedPoints%已选点少于要选的点
    numRestPoints = numToSelectedPoints - numSPoints;%还需要选的点个数
    
    % index of buckets that have rest points
    idxRestBuckets = find(numPointsPerBucket > pointCounter);
    if(numel(idxRestBuckets) < 3)%如果不超过三个bucket还有剩余点
        exitCounter = exitCounter + 1;
        if(exitCounter > 10),break;end % attention!!! parameter 10 might be tuned
    end
    
    if (numel(idxRestBuckets) > numRestPoints)%如果有剩余点的bucket个数大于 需要剩余点的个数
        % select randomly fewer buckets in idxRestBuckets%从剩余的比如50个bucket中选30个出来
        idxRestBuckets = idxRestBuckets(randperm(numel(idxRestBuckets), numRestPoints)');
    end
    
    numSelectedPointsPerBucket(idxRestBuckets) = pointCounter + 1;%相应的被选bucket的已选点数加一
    numSPoints = sum(numSelectedPointsPerBucket);
    pointCounter = pointCounter + 1;
end

%select points in each bucket
for i = 1:numBuckets
    % index of points belong to bucket i
    idxPointsBucketi = find(points2Bucket==i);
    n = numPointsPerBucket(i);
    k = numSelectedPointsPerBucket(i);
    
    %select k points in idxPointsBucketi
    idxSelectedPointsBucketi = idxPointsBucketi(randperm(n,k));
    
    %activate corresponded pixel
%     idxV = mod(idxSelectedPointsBucketi,256);
%     idxZero = find(~idxV);
%     idxV(idxZero) = 256;
%     
%     idxU = ceil(idxSelectedPointsBucketi/256);
%     
%     sPixel(idxV, idxU) = true;
    sPixelNormals(idxSelectedPointsBucketi) = true;
    
end

sPixel = logical(arrayActive .* sPixelNormals);

end

%%
%==========================================================================
%==========================================================================
% function [sPixel,numSPoints] = normalSpaceSamplingMN(pointsMN, normalsMN, numToSelectedPoints)
% %Normal space sampling with point cloud data structure MxNx3
% % Author: Bangshang LIU
% % Date: 15.06.2020
% % Version: 1.0
% 
% %parameters
% deltaTheta = 10 * pi/180; % rads
% deltaPhi = 10 * pi/180; % rads
% numSegTheta = 2*pi/deltaTheta; %36,attention!!!to be modified, not necessarily integer
% numSegPhi = pi/deltaPhi; %18,attention!!!to be modified, not necessarily integer
% numBuckets = numSegTheta * numSegPhi;
% % Temp: set random seed
% rng(9);
% 
% % initialization
% arrayDim = size(pointsMN, 1);
% sPixel = false(arrayDim);
% numSPoints = 0;
% 
% %sphere coordinates of normalized normals {MxNx2}
% sphereCoor = zeros(arrayDim, arrayDim, 2);
% sphereCoor(:,:,1) = atan2(normalsMN(:,:,2),normalsMN(:,:,1));
% sphereCoor(:,:,2) = acos(normalsMN(:,:,3));
% 
% %a M x N list indicating each point(index) in which bucket
% %index of bucket = index in theta * numSegPhi + index in phi
% points2Bucket = floor(sphereCoor(:,:,1)/deltaTheta) * numSegPhi + floor(sphereCoor(:,:,2)/deltaPhi);
% 
% %a list indicating how many points each bucket has
% numPointsPerBucket = zeros(numBuckets,1);
% for i = 1:numBuckets, numPointsPerBucket(i) = sum(sum(points2Bucket==i));end
% 
% %a list indicating how many points shall be selected for each bucket
% %according to numSelectedPoints
% numSelectedPointsPerBucket = zeros(numBuckets,1);
% pointCounter = 0;
% exitCounter = 0;
% while numSPoints < numToSelectedPoints
%     numRestPoints = numToSelectedPoints - numSPoints;
%     
%     % index of buckets that have rest points
%     idxRestBuckets = find(numPointsPerBucket > pointCounter);
%     if(numel(idxRestBuckets) < 3)
%         exitCounter = exitCounter + 1;
%         if(exitCounter > 10),break;end % attention!!! parameter 10 might be tuned
%     end
%     
%     if (numel(idxRestBuckets) > numRestPoints)
%         % select randomly fewer buckets in idxRestBuckets
%         idxRestBuckets = idxRestBuckets(randperm(numel(idxRestBuckets), numRestPoints)');
%     end
%     
%     numSelectedPointsPerBucket(idxRestBuckets) = pointCounter + 1;
%     numSPoints = sum(numSelectedPointsPerBucket);
%     pointCounter = pointCounter + 1;
% end
% 
% %select points in each bucket
% for i = 1:numBuckets
%     % index of points belong to bucket i
%     idxPointsBucketi = find(points2Bucket==i);
%     n = numPointsPerBucket(i);
%     k = numSelectedPointsPerBucket(i);
%     
%     %select k points in idxPointsBucketi
%     idxSelectedPointsBucketi = idxPointsBucketi(randperm(n,k));
%     
%     %activate corresponded pixel
% %     idxV = mod(idxSelectedPointsBucketi,256);
% %     idxZero = find(~idxV);
% %     idxV(idxZero) = 256;
% %     
% %     idxU = ceil(idxSelectedPointsBucketi/256);
% %     
% %     sPixel(idxV, idxU) = true;
%     sPixel(idxSelectedPointsBucketi) = true;
%     
% end
%     
% end
