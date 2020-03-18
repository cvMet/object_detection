%The following script generates the PR-Curve for a varying tau (if NNDR is
%smaller than threshold tau the keypoint pair is considered a match). Since
%the tau variation is done in the c++ script (objdetection) this script
%splits the complete dataset into several subsets. For each subset a single
%PR-value pair is calculated. The collection of these value pairs form the
%actual PR-Curve.

clear all;
directory = '..\PR\Guo\';
folders = dir(directory);
files = dir(strcat(folders(1).folder, '\', folders(1).name, '\*.csv'));
for i=1:length(files)
    filename = strcat(files(i).folder, '\', files(i).name)
    savename = strcat(files(i).folder, '\Curves\', files(i).name, '.png')
    data = load(filename,'-ascii');
    tp = 0;
    fp = 0;
    NOF_keypoints = data(1,1);
    support_radius = data(1,2);
    %Split indices determine where the complete dataset is divided into several
    %subsets. This is necessary since the complete set represents several
    %measurements with varying tau.
    split_indices = find(data == NOF_keypoints);
    split_indices = [split_indices; length(data)+1];
    for i = 1:length(split_indices)-1
       subset(i) = {data((split_indices(i):split_indices(i+1)-1),:)}; 
    end
    precision = [1];
    recall = [0];
    %Calculate Precision and recall for every sub_array
    for i = 1:length(subset)
        tp = 0;
        fp = 0;
        subset{i}(1,:)=[];
        distance = subset{i}(:,2);
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

    %Visualization
    set(groot, 'DefaultTextInterpreter', 'LaTeX');
    set(groot, 'DefaultAxesTickLabelInterpreter', 'LaTeX');
    set(groot, 'DefaultAxesFontName', 'LaTeX');
    set(groot, 'DefaultAxesFontWeight', 'bold');
    set(groot, 'DefaultLegendInterpreter', 'LaTeX');

    idx = 1:length(recall);
    idxq = linspace(min(idx), max(idx), 10);
    interpolated_recall = interp1(idx,recall,idxq, 'linear');
    interpolated_precision = interp1(idx,1-precision,idxq, 'linear');

    figure('Name', 'PR-Curve Guo et al.','NumberTitle','off');
    plot(1-precision,recall,'-o','LineWidth',1);
    axis([0 1 0 1]) %make sure axis goes from 0 to 1
    grid on;
    set(gca,'FontSize',14);
    legend('B-SHOT');
    xlabel('1-Precision','fontsize',15);ylabel('Recall','fontsize',15);
    saveas(gcf,savename)
end
