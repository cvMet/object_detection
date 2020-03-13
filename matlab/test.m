clear all;
filename = '..\PR\001_Bottlewhite1model.csv';
data = load(filename,'-ascii');
tp = 0;
fp = 0;
NOF_keypoints = data(1,1);
support_radius = data(1,2);
split_indices = find(data == 33);
split_indices = [split_indices; length(data)+1];

for i = 1:length(split_indices)-1
   sub_array(i) = {data((split_indices(i):split_indices(i+1)-1),:)}; 
end
precision = [];
recall = [];
for i = 1:length(sub_array)
    sub_array{i}(1,:)=[];
    distance = sub_array{i}(:,2);
    for i = 1:length(distance)
        if(distance(i) < support_radius)
            tp = tp + 1;
        else
            fp = fp + 1;
        end
    end
    precision = [precision tp/(tp+fp)];
    recall = [recall tp/NOF_keypoints];
end




%sub_array{1} first subarray
%sub_array{1}(1,:) first row of first subarray
% for i = 1:length(sub_array)
%     
%     
% end
