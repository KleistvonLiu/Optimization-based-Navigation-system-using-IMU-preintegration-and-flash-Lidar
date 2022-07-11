function [transform, rmse] = pcicpFL_LJF_V2(movingPc, fixedPc, params)
% Description:
% This is a self implementation of ICP for Astrone, 
% based on Flash LiDAR geometry and function pcregrigid() or pcregistericp()
% see also Matlab pcregistericp.m or pcregrigid.m

% Structure of this ICP algorithm follows the one proposed in
% [Rusinkiewicz_EfficientVariants_ICPAlgorithm]

%coder.extrinsic('knnsearch');
%coder.extrinsic('tic');
%coder.extrinsic('toc');
%tic
%% transfer input PC from 65536*3 into 256*256*3
fixedPc = reshape(fixedPc, params.arrayDim, params.arrayDim, 3);
movingPc = reshape(movingPc, params.arrayDim, params.arrayDim,3);
%% === Parameter initialization
% Validate inputs
% [metric, doExtrapolate, inlierRatio, maxIterations, tolerance, ...
%     initialTransform, verbose, useDegree] = validateAndParseOptInputs(moving, fixed, varargin{:});
% printer = vision.internal.MessagePrinter.configure(verbose);

%params.maxIterations = 50;
% params.initialTransform = A;
% params.metric = 'PointToPoint';
% params.doExtrapolate = false;
% params.inlierRatio = 0.3;
% params.tolerance = [0.00001, 0.00009];
% params.verbose = false;
% params.useDegree = false;
% params.arrayDim = 256;


%% === Down-sampling and selection

% spatial uniform sampling
% arrayActive = repmat([1 0;0 0],128,128);

% normal space sampling
% [sPixel,numSPoints] = normalSpaceSamplingMN(normalsMN, arrayActive, numToSelectedPoints)
% arrayActive = sPixel;

% or maxLeverage sampling

% or stability sampling
% (or according to intensity???)


% arrayActive

numToSelectedPoints = 4000;
%% === Compute normal vectors of the fixed point cloud
% --- for fixedPc
%arrayActive = repmat([1 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0],64,64);
% arrayActive = repmat([1 0;0 0],128,128);
arrayActive = true(params.arrayDim);

paramNormal.arrayActive = arrayActive;
paramNormal.numSPoints = 6;
paramNormal.radius = 0.15;
paramNormal.useRadius = false;
paramNormal.searchArrayDim = 6;
%tic
normalsMN_f = computeNormals_realtime(fixedPc, paramNormal);
%toc
% tic
% fix = pointCloud(fixedPc);
% %moving = pointCloud(movingPc);
% normals_ref_temp = pcnormals(fix,6);
% normals_ref = normals_ref_temp;
% 
% % inverse direction
% for i = 1:size(normals_ref_temp,1)
%    if(normals_ref_temp(i,3)>0)
%        normals_ref(i,:) = -normals_ref_temp(i,:);
%    end
% end
% toc
fixedPc_list = reshape(fixedPc,[],3);
normals_f_list = reshape(normalsMN_f,[],3);
%normals_list = zeros(size(fixedPc_list));
% no sampling
%arrayActive = true(params.arrayDim);
% spatial uniform sampling
%arrayActive = repmat([1 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0],64,64);
% arrayActive = false(params.arrayDim);
% arrayActive(1:2:end,1:2:end) = true;
%numSPoints = sum(arrayActive);
%interval sampling
% arrayActive = true(params.arrayDim);
% index_f = find(arrayActive);
% arrayActive(index_f(1:2:end)) = false;
%normal space sampling
%tic
% arrayActive = true(params.arrayDim);
% [sPixel,numSPoints] = normalSpaceSamplingMN(normalsMN_f, arrayActive, numToSelectedPoints+2000);
% arrayActive = sPixel;
% disp('number of selected points:')
% disp(numSPoints)
%toc
% Maximum Leverage Sampling
%tic
arrayActive = true(params.arrayDim);
[arrayActive,numSPoints] = MaximumLeverageSampling(fixedPc_list,normals_f_list, arrayActive, numToSelectedPoints);
disp('number of selected points:')
disp(numSPoints)
%toc

IdxActiveList = find(arrayActive);
fixedPc_list_sampled = fixedPc_list(IdxActiveList,:); %  could be modified?
fixedPc_normals_list_sampled = normals_f_list(IdxActiveList,:); %  could be modified?

% --- for movingPc
normalsMN_m = computeNormals_realtime(movingPc, paramNormal);
%toc
movingPc_list = reshape(movingPc,[],3);
normals_m_list = reshape(normalsMN_m,[],3);
%normals_list = zeros(size(movingPc_list));
% no sampling
% arrayActive = true(params.arrayDim);
% numSPoints = params.arrayDim*params.arrayDim;
% spatial uniform sampling
%arrayActive = repmat([1 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0],64,64);
% arrayActive = false(params.arrayDim);
% arrayActive(1:2:end,1:2:end) = true;
% numSPoints = sum(arrayActive,'all');
%interval sampling
% arrayActive = true(params.arrayDim);
% index_m = find(arrayActive);
% arrayActive(index_m(1:2:end)) = false;
% normal space sampling
%tic
% arrayActive = true(params.arrayDim);
% [sPixel,numSPoints] = normalSpaceSamplingMN(normalsMN_m, arrayActive, numToSelectedPoints+2000);
% arrayActive = sPixel;
% disp('number of selected points:')
% disp(numSPoints)
%toc
% Maximum Leverage Sampling
%tic
arrayActive = true(params.arrayDim);
[arrayActive,numSPoints] = MaximumLeverageSampling(movingPc_list,normals_m_list, arrayActive, numToSelectedPoints);
disp('number of selected points:')
disp(numSPoints)
%toc

IdxActiveList = find(arrayActive);

%numSelectedPoints = 4000;%256*256;%4096
movingPc_list_sampled = movingPc_list(IdxActiveList,:); 
%movingPc_list_sampled = movingPc_list(IdxActiveList(1:numSelectedPoints),:); %  could be modified?

movingPc_normals_list_sampled = normals_m_list(IdxActiveList,:); %  could be modified?

% movingPc_list = reshape(movingPc,[],3);


% --- A copy of the input with unorganized M-by-3 data
% ptCloudP = removeInvalidPoints(moving);
% [ptCloudQ, validPtCloudIndices] = removeInvalidPoints(fixed);


% --- Compute the unit normal vector of fixed point cloud/pcCloudB if it is not provided.
% alternative: run pcnormals to compute normals of fixed point cloud Q
% if isempty(fixedPc.Normal)
%     fixedCount = fixedPc.Count;
%     % Use 6 neighboring points to estimate a normal vector. You may use
%     % pcnormals with customized parameter to compute normals upfront.
%     fixedPc.Normal = surfaceNormalImpl(fixedPc, 6);        
%     ptCloudQ.Normal = [fixedPc.Normal(validPtCloudIndices), ...
%                        fixedPc.Normal(validPtCloudIndices + fixedCount), ...
%                        fixedPc.Normal(validPtCloudIndices + fixedCount * 2)];
% end
% % Remove points if their normals are invalid
% tf = isfinite(ptCloudQ.Normal);
% validIndices = find(sum(tf, 2) == 3);
% if numel(validIndices) < ptCloudQ.Count
%     [loc, ~, nv] = subsetImpl(ptCloudQ, validIndices);
%     ptCloudQ = pointCloud(loc, 'Normal', nv);
%     if ptCloudQ.Count < 3
%         error(message('vision:pointcloud:notEnoughPoints'));
%     end
% end

%toc
%% === Data allocation

% variables allocation
% query of rotation matrix Rs and translational vector Ts
Rs = zeros(3, 3, params.maxIterations+1);
Ts = zeros(3, params.maxIterations+1);

% Quaternion and translation vector
% qs = [ones(1, params.maxIterations+1); zeros(6, params.maxIterations+1)];
qs = [zeros(3, params.maxIterations+1);...
        ones(1, params.maxIterations+1);...
		zeros(3, params.maxIterations+1)];

% The difference of quaternion and translation vector in consecutive
% iterations, for convergence criteria
dq = zeros(7, params.maxIterations+1);

% The angle between quaternion and translation vectors in consecutive
% iterations, for extrapolation
dTheta = zeros(params.maxIterations+1, 1);

% RMSE
Err = zeros(params.maxIterations+1, 1); 


% --- apply the initial guess of transformation
% We use pre-multiplication format in this algorithm.
Rs(:,:,1) = params.initialTransform(1:3, 1:3);
Ts(:,1) = params.initialTransform(1:3,4);
% qs(:,1) = [vision.internal.quaternion.rotationToQuaternion(Rs(:,:,1)); Ts(:,1)];
qs(:,1) = [rotm2quatliub(Rs(:,:,1))'; Ts(:,1)];
% now quaternion scalar part is in qs(4,i), not in qs(1,i)

% locP = ptCloudP.Location;
% apply initial transformation to ptCloudP to have locP
% TODO: to be modified because of different quaternion representation
movingPc_list_new = rigidTransform(movingPc_list_sampled, Rs(:,:,1), Ts(:,1));
% !!! apply initial rotation on normals 
movingPc_normals_list_new = rigidTransform(movingPc_normals_list_sampled, Rs(:,:,1), [0;0;0]);

stopIteration = int8(params.maxIterations);

%##Liu: number of inlier pairs in the correspondence list of point pairs sorted
%accroding to their Euclidean distance
%##Liu: inlierRatio describes the percentage of NUMBER of points in ptCloudP
upperBound = max(1, round(params.inlierRatio(1)*size(movingPc_list_sampled,1)));    

% TODO: other outlier rejection algorthms to be implemented
% e.g. compatible normals 

%toc
%% === main ICP iteration
for i = 1 : params.maxIterations
%for i = 1 : 1
%     printer.linebreak;
%     printer.print('--------------------------------------------\n');
%     printer.printMessage('vision:pointcloud:icpIteration',i);
%     printer.printMessageNoReturn('vision:pointcloud:findCorrespondenceStart');
    
	
    % ======Match corresponding point pairs =====================
	% TODO: 
	% 		1. consider intensity compatibility???
	%		2. using geometric of Flash LiDAR sensor array??? (sensor structure/specificity)
	%		(3. "projection" from P to Q???)
	

    %disp('knnsearch running time')
    %tic
 	%ptCloudQ = pointCloud(fixedPc_list_sampled);
    %[indices, dists] = multiQueryKNNSearchImpl(ptCloudQ, movingPc_list_new, 1);

	% alternative: function knnsearch in Statistics toolbox
    %[indices,dists]= knnsearch(fixedPc_list_sampled, movingPc_list_new, 'K', 1,'NSMethod','exhaustive');
    [indices,dists]= knnsearch(fixedPc_list_sampled, movingPc_list_new, 'K', 1,'NSMethod','kdtree');
    %toc
	% ======Weighting correspondences ===========================
	% TODO: some weighting algorithm???
	% 1. normal dot-product?
	% 2. inverse Euclidean distance?
	% 3. uncertainty model???
	% ======Reject outliers=============================
	%TODO:other rejection methods?
	% 1. consider normal compatibility??? paris with small value is rejected
    % cosAng = normalP * normalQ;
    % if(cosAng > 10/180*pi)
%         remain;
    % end
%     inlierIndicesM, inlierIndicesF 

%     threshold = (5/180)*pi;
%     dot_fm = dot(fixedPc_normals_list_sampled(indices,:), movingPc_normals_list_new,2);
%     keepInlierM = acos(dot_fm)<threshold;
%     inlierIndicesM = find(keepInlierM);
%     inlierIndicesF = indices(keepInlierM);
%     inlierDist = dists(keepInlierM);

	% 4. reject pairs containing points on boundaries
    % indexMN = [mod(diff_ref_nor_idx,256) fix(diff_ref_nor_idx/256)];
    % if(indexMN in edges)
    %   remove;
    % end
    %     inlierIndicesM, inlierIndicesF        

    
    % 5. reject pairs whose Euclidean distance inconsistent with neighbouring pairs
    
	% 2. reject pairs whose absolute Euclidean distance greater than threshold?
%     keepInlierM = false(size(movingPc_list_sampled,1), 1);
%     threshold = max(0.05,params.thresHold-0.01*i);
%     threshold = params.thresHold;
%     keepInlierM = dists < threshold;
%     %keepInlierM = dists < params.thresHold;
%     inlierIndicesM = find(keepInlierM);
%     inlierIndicesF = indices(keepInlierM);
%     inlierDist = dists(keepInlierM);
	% 3. reject pairs whose distance greater than standard deviation of "dists"?
    
    % Remove outliers
	% Question: is this "trimmed ICP"?
    % 9 rejection based on 3 times median and normals compatibility
    [~, idx] = sort(dists);
    med = 3*dists(idx(floor(numSPoints/2)+1));
    keepInlierM1 = dists < squeeze(med); 
    threshold = (10/180)*pi;
    dot_fm = dot(fixedPc_normals_list_sampled(indices,:), movingPc_normals_list_new,2);
    keepInlierM2 = acos(dot_fm) < threshold;
    keepInlierM = keepInlierM1 & keepInlierM2;
    inlierIndicesM = find(keepInlierM);
    inlierIndicesF = indices(keepInlierM);
    inlierDist = dists(keepInlierM);
    % 8. rejection on the basis of the distances between corresponding points
%     dp = abs(dot(movingPc_list_new - fixedPc_list_sampled(indices,:),fixedPc_normals_list_sampled(indices,:),2));
%     med = median(dp);
%     sig_mad = 1.4826*mad(dp,1);
%     keepInlierM1 = dists < med + 3*sig_mad;
%     threshold = (10/180)*pi;
%     dot_fm = dot(fixedPc_normals_list_sampled(indices,:), movingPc_normals_list_new,2);
%     keepInlierM2 = acos(dot_fm)<threshold;
%     keepInlierM = keepInlierM1 & keepInlierM2;
%     inlierIndicesM = find(keepInlierM);
%     inlierIndicesF = indices(keepInlierM);
%     inlierDist = dists(keepInlierM);
    % 6. inlier ratio 
    
%     keepInlierM = false(size(movingPc_list_sampled,1), 1); 
%     [~, idx] = sort(dists);
%     keepInlierM(idx(1:upperBound)) = true;%!!!
%     inlierIndicesM = find(keepInlierM);
%     inlierIndicesF = indices(keepInlierM);
%     inlierDist = dists(keepInlierM);
    % 7. inlier ratio and normal compatibility
%     keepInlierM1 = false(size(movingPc_list_sampled,1), 1); 
%     [~, idx] = sort(dists);
%     keepInlierM1(idx(1:upperBound)) = true;%
%     threshold = (10/180)*pi;
%     dot_fm = dot(fixedPc_normals_list_sampled(indices,:), movingPc_normals_list_new,2);
%     keepInlierM2 = acos(dot_fm)<threshold;
%     keepInlierM = keepInlierM1 & keepInlierM2;
%     inlierIndicesM = find(keepInlierM);
%     inlierIndicesF = indices(keepInlierM);
%     inlierDist = dists(keepInlierM);
%     
    % correspondence relation is stored in inlierIndicesM,
    %inlierIndicesF and their distance is in inlierDist
    %toc
%     if numel(inlierIndicesM) < 3
%         error(message('vision:pointcloud:notEnoughPoints'));
%     end

%     printer.printMessage('vision:pointcloud:stepCompleted');

    if i == 1
        % average distance among all point pairs at the first
        %iteration
        Err(i) = sqrt(sum(inlierDist)/length(inlierDist));
    end
	%toc
    
%     printer.printMessageNoReturn('vision:pointcloud:estimateTransformStart');
    
	
	% ======Minimizing errors =============================
    % Estimate transformation given correspondences
    if strcmpi(params.metric, 'PointToPoint')
        [R, T] = vision.internal.calibration.rigidTransform3D(...
            movingPc_list_new(inlierIndicesM, :), ...
            fixedPc_list_sampled(inlierIndicesF, :));
        
    else % PointToPlane
        [R, T] = pointToPlaneMetric(movingPc_list_new(inlierIndicesM, :), ...
            fixedPc_list_sampled(inlierIndicesF, :), fixedPc_normals_list_sampled(inlierIndicesF, :));
    end       
%toc
    % Bad correspondence may lead to singular matrix
%     if any(isnan(T))||any(isnan(R(:)))
%         error(message('vision:pointcloud:singularMatrix'));
%     end
    %toc
	% ======Post-processing =============================
    % Update the total transformation
    Rs(:,:,i+1) = R * Rs(:,:,i);
    Ts(:,i+1) = R * Ts(:,i) + T; % Question: why "R * Ts(:,i)"???
    
%     printer.printMessage('vision:pointcloud:stepCompleted');

    % RMSE
    movingPc_list_new = rigidTransform(movingPc_list_sampled, Rs(:,:,i+1), Ts(:,i+1));
    squaredError = sum((movingPc_list_new(inlierIndicesM, :) - fixedPc_list_sampled(inlierIndicesF, :)).^2, 2);
    Err(i+1) = sqrt(sum(squaredError)/length(squaredError));
    
    % Convert to vector representation
	% can be output directly
%     qs(:,i+1) = [vision.internal.quaternion.rotationToQuaternion(Rs(:,:,i+1)); Ts(:,i+1)];
    qs(:,i+1) = [rotm2quatliub(Rs(:,:,i+1))'; Ts(:,i+1)];
    % now quaternion scalar part is in qs(4,i), not in qs(1,i)
	%toc
	% ======Extrapolation================================
	
    % With extrapolation, we might be able to converge faster
    if params.doExtrapolate
%         printer.printMessageNoReturn('vision:pointcloud:updateTransformStart');
        extrapolateInTransformSpace;

%         printer.printMessage('vision:pointcloud:stepCompleted');
    end
    %toc
	
    % ======Check convergence=============================
	
    % Compute the mean difference in R/T from the recent three iterations.
%     [dR, dT] = getChangesInTransformation;
        dR = 0;
        dT = 0;
        count = 0;
        for k = max(i-2,1):i
            % Rotation difference in radians
            
            diff_theta = 2*qs(1:3,k+1)-2*qs(1:3,k);
            rdiff = norm(diff_theta);
            
%             rdiff = acos(dot(qs(1:4,k),qs(1:4,k+1))/(norm(qs(1:4,k))*norm(qs(1:4,k+1))));
            % Euclidean difference
            tdiff = sqrt(sum((Ts(:,k)-Ts(:,k+1)).^2));
            dR = dR + rdiff;
            dT = dT + tdiff;
            count = count + 1;
        end
        dT = dT/count;
        dR = dR/count;
    
%     printer.printMessage('vision:pointcloud:checkConverge',num2str(tdiff), num2str(rdiff), num2str(Err(i+1)));

    % Stop ICP if it already converges
    if dT <= params.tolerance(1) && dR <= params.tolerance(2)
        stopIteration = int8(i);
        fprintf('the number of iterations: %d\n',stopIteration);
        break;
    end
    %toc
    if i == params.maxIterations
        fprintf('the number of iterations: %d\n',int8(i));
    end
end
%toc
%% === Post-processing
% Make the R to be orthogonal as much as possible
R = Rs(:,:,stopIteration+1);
[U, ~, V] = svd(R);
R = U * V';


% tformMatrix = [R, zeros(3,1);...
%                Ts(:, stopIteration+1)',  1];
transform = [R,         Ts(:, stopIteration+1);...
            zeros(1,3),  1];
rmse = Err(stopIteration+1);

% printer.linebreak;
% printer.print('--------------------------------------------\n');
% printer.printMessage('vision:pointcloud:icpSummary',stopIteration, num2str(rmse));

% if nargout >= 2
%     movingReg = pctransform(movingPc, transform);
% end
%toc
%% == Nested functions
    %======================================================================
    % Nested function to perform extrapolation
    % Besl, P., & McKay, N. (1992). A method for registration of 3-D shapes. 
    % IEEE Transactions on pattern analysis and machine intelligence, p245.
    %======================================================================
    function extrapolateInTransformSpace        
        dq(:,i+1) = qs(:,i+1) - qs(:,i);
        n1 = norm(dq(:,i));
        n2 = norm(dq(:,i+1));
        dTheta(i+1) = (180/pi)*acos(dot(dq(:,i),dq(:,i+1))/(n1*n2));
        %disp(dTheta(i+1))
        angleThreshold = 10;%
        scaleFactor = 50;%
        if i > 2 && dTheta(i+1) < angleThreshold && dTheta(i) < angleThreshold
            d = [Err(i+1), Err(i), Err(i-1)];
            disp('using extrapolation');
            v = [0, -n2, -n1-n2];
            vmax = scaleFactor * n2;
            dv = extrapolate(v,d,vmax);
            if dv ~= 0
                q = qs(:,i+1) + dv * dq(:,i+1)/n2;
                q(1:4) = q(1:4)/norm(q(1:4));
                % Update transformation and data
                qs(:,i+1) = q;
%                 Rs(:,:,i+1) = vision.internal.quaternion.quaternionToRotation(q(1:4));
                Rs(:,:,i+1) = quat2rotmatrixliub(q(1:4));
                Ts(:,i+1) = q(5:7);
                movingPc_list_new = rigidTransform(movingPc_list_sampled, Rs(:,:,i+1), Ts(:,i+1));
            end
        end
    end

    %======================================================================
    % Nested function to compute the changes in rotation and translation
    %======================================================================
%     function [dR, dT] = getChangesInTransformation
%         dR = 0;
%         dT = 0;
%         count = 0;
%         for k = max(i-2,1):i
%             % Rotation difference in radians
%             rdiff = acos(dot(qs(1:4,k),qs(1:4,k+1))/(norm(qs(1:4,k))*norm(qs(1:4,k+1))));
%             % Euclidean difference
%             tdiff = sqrt(sum((Ts(:,k)-Ts(:,k+1)).^2));
%             dR = dR + rdiff;
%             dT = dT + tdiff;
%             count = count + 1;
%         end
%         dT = dT/count;
%         dR = dR/count;
%     end

end

%==========================================================================
% Parameter validation
%==========================================================================
% function [metric, doExtrapolate, inlierRatio, maxIterations, tolerance, ...
%     initialTransform, verbose, useDegree] = validateAndParseOptInputs(moving, fixed, varargin)
% 
% validateattributes(moving, {'pointCloud'}, {'scalar'}, mfilename, 'moving');
% validateattributes(fixed, {'pointCloud'}, {'scalar'}, mfilename, 'fixed');
% 
% persistent p;
% if isempty(p)
%     % Set input parser
%     defaults = struct(...
%         'Metric',           'PointToPoint', ...
%         'Extrapolate',      false, ...
%         'InlierRatio',      1.0,...
%         'MaxIterations',    20,...
%         'Tolerance',        [0.01, 0.5],...
%         'InitialTransform', affine3d(),...
%         'Verbose',          false, ...
%         'UseDegree',        true);
%     
%     p = inputParser;
%     p.CaseSensitive = false;
%     
%     p.addParameter('Metric', defaults.Metric, ...
%         @(x)validateattributes(x,{'char','string'},{'scalartext'}));
%     p.addParameter('Extrapolate', defaults.Extrapolate, ...
%         @(x)validateattributes(x,{'logical'}, {'scalar','nonempty'}));
%     p.addParameter('InlierRatio', defaults.InlierRatio, ...
%         @(x)validateattributes(x,{'single', 'double'}, {'real','nonempty','scalar','>',0,'<=',1}));
%     p.addParameter('MaxIterations', defaults.MaxIterations, ...
%         @(x)validateattributes(x,{'single', 'double'}, {'real','scalar','integer','positive'}));
%     p.addParameter('Tolerance', defaults.Tolerance, ...
%         @(x)validateattributes(x,{'single', 'double'}, {'real','nonnegative','numel', 2}));
%     p.addParameter('InitialTransform', defaults.InitialTransform, ...
%         @(x)validateattributes(x,{'affine3d'}, {'scalar'}));
%     p.addParameter('Verbose', defaults.Verbose, ...
%         @(x)validateattributes(x,{'logical'}, {'scalar','nonempty'}));
%     p.addParameter('UseDegree', defaults.UseDegree, ...
%         @(x)validateattributes(x,{'logical'}, {'scalar','nonempty'}));
%     parser = p;
% else
%     parser = p;
% end
% 
% parser.parse(varargin{:});
% 
% metric          = validatestring(parser.Results.Metric, {'PointToPoint', 'PointToPlane'}, mfilename, 'Metric');
% doExtrapolate   = parser.Results.Extrapolate;
% inlierRatio     = parser.Results.InlierRatio;
% maxIterations   = parser.Results.MaxIterations;
% tolerance       = parser.Results.Tolerance;
% useDegree       = parser.Results.UseDegree;
% if useDegree
%     % Convert from degree to radian internally
%     tolerance(2)    = tolerance(2)*pi/180;
% end
% initialTransform = parser.Results.InitialTransform;
% if ~(isRigidTransform(initialTransform))
%     error(message('vision:pointcloud:rigidTransformOnly'));
% end
% verbose = parser.Results.Verbose;
% end

%==========================================================================
% Determine if transformation is rigid transformation
%==========================================================================
function tf = isRigidTransform(tform)

singularValues = svd(tform.T(1:tform.Dimensionality,1:tform.Dimensionality));
tf = max(singularValues)-min(singularValues) < 100*eps(max(singularValues(:)));
tf = tf && abs(det(tform.T)-1) < 100*eps(class(tform.T));

end
        
%==========================================================================
function B = rigidTransform(A, R, T)
    %(R*A) A is 3*1; (R*A)' = A'*R', A' is 1*3
    B = A * R';
    B(:,1) = B(:,1) + T(1);
    B(:,2) = B(:,2) + T(2);
    B(:,3) = B(:,3) + T(3);
end

%==========================================================================
% Solve the following minimization problem:
%       min_{R, T} sum(|dot(R*p+T-q,nv)|^2)
%
% p, q, nv are all N-by-3 matrix, and nv is the unit normal at q
%
% Here the problem is solved by linear approximation to the rotation matrix
% when the angle is small.
%==========================================================================
function [R, T] = pointToPlaneMetric(p, q, nv)
	% see paper xxxPomerleau2015_AReview_PointCloudRegistrationAlgo_MobileRobotics.pdf
	% page 92 Appendix A and page 43 example 2
	% rotation with small angle approximation
	
    % Set up the linear system
    cn = [cross(p,nv,2),nv];
    C = cn'*cn;
    qp = q-p;
    b =  [sum(sum(qp.*repmat(cn(:,1),1,3).*nv, 2));
          sum(sum(qp.*repmat(cn(:,2),1,3).*nv, 2));
          sum(sum(qp.*repmat(cn(:,3),1,3).*nv, 2));
          sum(sum(qp.*repmat(cn(:,4),1,3).*nv, 2));
          sum(sum(qp.*repmat(cn(:,5),1,3).*nv, 2));
          sum(sum(qp.*repmat(cn(:,6),1,3).*nv, 2))];
    % X is [alpha, beta, gamma, Tx, Ty, Tz]
    X = C\b;

    cx = cos(X(1)); 
    cy = cos(X(2)); 
    cz = cos(X(3)); 
    sx = sin(X(1)); 
    sy = sin(X(2)); 
    sz = sin(X(3)); 

    R = [cy*cz, sx*sy*cz-cx*sz, cx*sy*cz+sx*sz;
         cy*sz, cx*cz+sx*sy*sz, cx*sy*sz-sx*cz;
           -sy,          sx*cy,          cx*cy];

    T = X(4:6);
end

%==========================================================================
% Extrapolation in quaternion space. Details are found in:
% Besl, P., & McKay, N. (1992). A method for registration of 3-D shapes. 
% IEEE Transactions on pattern analysis and machine intelligence, 239-256.
%==========================================================================
function dv = extrapolate(v,d,vmax)
    p1 = polyfit(v,d,1);    % linear fit
    p2 = polyfit(v,d,2);    % parabolic fit
    v1 = -p1(2)/p1(1);      % linear zero crossing point
    v2 = -p2(2)/(2*p2(1));  % polynomial top point

    if (issorted([0 v2 v1 vmax]) || issorted([0 v2 vmax v1]))
        % Parabolic update
        dv = v2;
    elseif (issorted([0 v1 v2 vmax]) || issorted([0 v1 vmax v2])...
            || (v2 < 0 && issorted([0 v1 vmax])))
        % Line update
        dv = v1;
    elseif (v1 > vmax && v2 > vmax)
        % Maximum update
        dv = vmax;
    else
        % No extrapolation
        dv = 0;
    end
end

%% === some functions
function quat = rotm2quatliub( R )
%Modifying Matlab function rotm2quat()
%ROTM2QUAT Convert rotation matrix to quaternion
%   Q = ROTM2QUAT(R) converts a 3D rotation matrix, R, into the corresponding
%   unit quaternion representation, Q. The input, R, is an 3-by-3-by-N matrix
%   containing N orthonormal rotation matrices.
%   The output, Q, is an N-by-4 matrix containing N quaternions. Each
%   quaternion is of the form q = [x y z w], with a scalar number as
%   the last value. Each element of Q must be a real number.
%
%   If the input matrices are not orthonormal, the function will
%   return the quaternions that correspond to the orthonormal matrices
%   closest to the imprecise matrix inputs.
%
%
%   Example:
%      % Convert a rotation matrix to a quaternion
%      R = [0 0 1; 0 1 0; -1 0 0];
%      q = rotm2quat(R)
%
%   References:
%   [1] I.Y. Bar-Itzhack, "New method for extracting the quaternion from a 
%       rotation matrix," Journal of Guidance, Control, and Dynamics, 
%       vol. 23, no. 6, pp. 1085-1087, 2000
%
%   See also quat2rotm

%   Copyright 2014-2016 The MathWorks, Inc.


%#codegen
R = R';
%so that quaternion describes also rotation from navigation frame to local frame
%robotics.internal.validation.validateRotationMatrix(R, 'rotm2quat', 'R');


% Pre-allocate output
quat = zeros(size(R,3), 4, 'like', R);

% Calculate all elements of symmetric K matrix
K11 = R(1,1,:) - R(2,2,:) - R(3,3,:);
K12 = R(1,2,:) + R(2,1,:);
K13 = R(1,3,:) + R(3,1,:);
K14 = R(3,2,:) - R(2,3,:);

K22 = R(2,2,:) - R(1,1,:) - R(3,3,:);
K23 = R(2,3,:) + R(3,2,:);
K24 = R(1,3,:) - R(3,1,:);

K33 = R(3,3,:) - R(1,1,:) - R(2,2,:);
K34 = R(2,1,:) - R(1,2,:);

K44 = R(1,1,:) + R(2,2,:) + R(3,3,:);

% Construct K matrix according to paper
K = [...
    K11,    K12,    K13,    K14;
    K12,    K22,    K23,    K24;
    K13,    K23,    K33,    K34;
    K14,    K24,    K34,    K44];

K = K ./ 3;

% For each input rotation matrix, calculate the corresponding eigenvalues
% and eigenvectors. The eigenvector corresponding to the largest eigenvalue
% is the unit quaternion representing the same rotation.
for i = 1:size(R,3)
    [eigVec,eigVal] = eig(K(:,:,i),'vector');
    [~,maxIdx] = max(real(eigVal));
    quat(i,:) =...
        real([eigVec(1,maxIdx) eigVec(2,maxIdx) eigVec(3,maxIdx) eigVec(4,maxIdx)]);
    % By convention, always keep scalar quaternion element positive. 
    % Note that this does not change the rotation that is represented
    % by the unit quaternion, since q and -q denote the same rotation.
    if quat(i,4) < 0
        quat(i,:) = -quat(i,:);
    end
end

end


function R = quat2rotmatrixliub(q)

%
%Description:
%   Converting quaternion to rotation matrix
%
%Input:     q quaternion in format [qx qy qz qw;], qw is the skalar value
%Output:    R rotation matrix (forward transformation from navigation to local frame)

% Normalize the quaternions
q = q / norm(q);
R = zeros(3);

R = [q(1)^2+q(4)^2-q(2)^2-q(3)^2 2*(q(1)*q(2)+q(3)*q(4)) 2*(q(1)*q(3)-q(2)*q(4));
    2*(q(1)*q(2)-q(3)*q(4)) q(2)^2+q(4)^2-q(1)^2-q(3)^2 2*(q(2)*q(3)+q(1)*q(4));
    2*(q(1)*q(3)+q(2)*q(4)) 2*(q(2)*q(3)-q(1)*q(4)) q(3)^2+q(4)^2-q(1)^2-q(2)^2];

end