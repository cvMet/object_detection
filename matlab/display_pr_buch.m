%The following script generates the PR-Curve based on the proposition of
%buch et al. All matches of a dataset (containing several models and scenes)
%are collected in one array. While traversing the sorted array PR-value
%pairs are computed continuously. These values are used to draw the
%PR-curve.
clear all;
directory = '..\PR\Buch\Automated\';
folders = dir(directory);
files = dir(strcat(folders(1).folder, '\', folders(1).name, '\*.csv'));
rot = files(1).name(length(files(1).name)-9);
NOF_colors = 8;
marker_index = 1;
for i0=1:length(files)
    filename = strcat(files(i0).folder, '\', files(i0).name)
    savename = strcat(files(i0).folder, '\', files(i0).name, '.png')
    data = load(filename,'-ascii');
    tp = 0;
    fp = 0;
    NOF_keypoints = data(length(data),1);
    support_radius = data(length(data),2);
    %Pop last row containing NOF keypoints and 0.5*supportradius
    data(length(data),:) = [];
    NNDR = data(:,1);
    euclidean_distance = data(:,2);
    %Sort keypoints by NNDR in descending order
%     [sorted_keypoints, sort_idx] = sort(NNDR, 'descend'); 
%     sorted_euclidean_distance = euclidean_distance(sort_idx);
    %Sort keypoints by euclidean ascending order
    [sorted_keypoints, sort_idx] = sort(euclidean_distance, 'ascend'); 
    sorted_euclidean_distance = sorted_keypoints;
    %Calculate TP&FP -> derive precision and recall
    precision = [];
    recall = [];
    for i = 1:length(NNDR)
        if(sorted_euclidean_distance(i) < support_radius)
            tp = tp + 1;
        else
            fp = fp + 1;
        end
        %only add every third value to PR-curve do get better
        %distinguishable curves
        if(mod(i+2, 3) == 0)
            precision = [precision tp/(tp+fp)];
            recall = [recall tp/NOF_keypoints];
        end
    end
    %Change marker if all line colors were already used in graph
    if(mod(i0, NOF_colors) == 0)
        marker_index = marker_index + 1;
    end
    if(strcmp(rot, '4') || strcmp(rot, '1'))
        %Create Rot Legend Name
        temp = strcat(files(i0).name(length(files(i0).name)-10:length(files(i0).name)-8));
        name = '';
        for char = 1:length(temp)
        	if(temp(char) ~= '_')
                name = strcat(name, temp(char));
            end
        end
    else
       %Create Trans Legend Name
        name = strcat(files(i0).name(length(files(i0).name)-10:length(files(i0).name)-4));
    	for char = 1:3
            if(strcmp(name(1), '0')||strcmp(name(1), '_'))
               name(1) = [];
            end
        end
    end
    
    %Visualization
    set(groot, 'DefaultTextInterpreter', 'LaTeX');
    set(groot, 'DefaultAxesTickLabelInterpreter', 'LaTeX');
    set(groot, 'DefaultAxesFontName', 'LaTeX');
    set(groot, 'DefaultAxesFontWeight', 'bold');
    set(groot, 'DefaultLegendInterpreter', 'LaTeX');
    lineS = {'o', 'square', 'diamond', '*', 'p'}';
    idx = 1:length(precision);
    idxq = linspace(min(idx), max(idx), 10);
    interpolated_recall = interp1(idx,recall,idxq, 'linear');
    interpolated_precision = interp1(idx,1-precision,idxq, 'linear');

    figure(1);
    hold on
    plot(interpolated_precision,interpolated_recall,'-o','LineWidth',1, 'Displayname', name, 'Marker', lineS{marker_index});
    axis([0 1 0 1]) %make sure axis goes from 0 to 1
    grid on;
    set(gca,'FontSize',14);
    xlabel('1-Precision','fontsize',15);ylabel('Recall','fontsize',15);
    set(gcf, 'Position',  [100, 100, 800, 700])

end
    legend('show');
    saveas(gcf,savename)
