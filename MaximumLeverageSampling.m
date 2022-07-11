function [arrayActive,numSPoints] = MaximumLeverageSampling(PC,normalsMN, arrayActive, numToSelectedPoints)
index = find(arrayActive);
arrayActive(:) = false;
PC_listed = reshape(PC,[],3);
normalsMN_listed = reshape(normalsMN,[],3);
% reduce the scale of each value
PC_listed_mean = PC_listed - repmat(mean(PC_listed,1),size(PC_listed,1),1);
% compute coefficient matrix
A = [cross(PC_listed_mean,normalsMN_listed) normalsMN_listed];
%A = []
% assign A and index as size-variable
coder.varsize('A');
coder.varsize('index');
while numel(index) > numToSelectedPoints
	% compute leverage h
    Qxx = inv(A'*A);
    h = sum((A*Qxx).*A,2);
	% reject the lowest 2 percent points
    hMin = quantile(h, 0.02);
    idx2del = find(h < hMin);
    if isempty(idx2del), [~, idx2del] = min(h); end
    A(idx2del, :)   = [];
    index(idx2del) = [];
end
arrayActive(index) = true;
numSPoints = numel(index);
end