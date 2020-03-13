function [precision, recall] = calcPR(filename,n)

data = load(filename,'-ascii');
numKeypoints = data(1,1);
tpThreshold = data(1,2);

data(1,:)=[];
nndr = data(:,1); %first column corresponds to NNDR
distance = data(:,2); %second column corresponds to L2-distance
[sorted_nndr, sort_idx] = sort(nndr, 'descend'); %sort NNDR
sorted_dist = distance(sort_idx); %sort distances corresponding to NNDR
%point = zeros(1,size(distance)/10+1);
tp = 0;fp = 0;

for i = 0:length(distance)/n-1
    if (i+1)*n<length(distance)
        tp = tp + sum(sorted_dist((i*n)+1:(i+1)*n)<tpThreshold);
        fp = (i+1)*n-tp; 
    else
        tp = tp + sum(sorted_dist((i*n)+1:length(distance))<tpThreshold);
        fp = length(distance) - tp;
    end
    
    recall(i+1) = tp/numKeypoints;
    precision(i+1) = tp/(tp+fp);
end


end