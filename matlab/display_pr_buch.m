%The following script generates the PR-Curve based on the proposition of
%buch et al. All matches of a dataset (containing several models and scenes)
%are collected in one array. While traversing the sorted array PR-value
%pairs are computed continuously. These values are used to draw the
%PR-curve.
clear all;
directory = '..\PR\Buch\';
folders = dir(directory);
files = dir(strcat(folders(1).folder, '\', folders(1).name, '\*.csv'));
for i=1:length(files)
    filename = strcat(files(i).folder, '\', files(i).name)
    savename = strcat(files(i).folder, '\Curves\', files(i).name, '.png')
    data = load(filename,'-ascii');
    tp = 0;
    fp = 0;
    NOF_keypoints = data(length(data),1);
    support_radius = data(length(data),2);
    %Pop last row containing NOF keypoints and 0.5*supportradius
    data(length(data),:) = [];
    NNDR = data(:,1);
    euclidean_distance = data(:,2);
    %Sort keypoints by NNDR in ascending order
    [sorted_keypoints, sort_idx] = sort(NNDR, 'descend'); 
    sorted_euclidean_distance = euclidean_distance(sort_idx);
    %Calculate TP&FP -> derive precision and recall
    precision = [];
    recall = [];
    for i = 1:length(NNDR)
        if(sorted_euclidean_distance(i) < support_radius)
            tp = tp + 1;
        else
            fp = fp + 1;
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

    figure('Name', 'PR-Curve Buch et al.','NumberTitle','off');
    plot(interpolated_precision,interpolated_recall,'-o','LineWidth',1);
    axis([0 1 0 1]) %make sure axis goes from 0 to 1
    grid on;
    set(gca,'FontSize',14);
    legend('B-SHOT');
    xlabel('1-Precision','fontsize',15);ylabel('Recall','fontsize',15);
    saveas(gcf,savename)
end
