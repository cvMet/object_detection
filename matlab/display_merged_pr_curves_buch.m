%The following script generates the PR-Curve based on the proposition of
%buch et al. All matches of a dataset (containing several models and scenes)
%are collected in one array. While traversing the sorted array PR-value
%pairs are computed continuously. These values are used to draw the
%PR-curve.
clear all;
rootdir = '..\PR\Buch\Automated\20_03_20';
folder_list = rdir([rootdir, '\**\*.'], 'regexp(name, ''iss\d'')', true);
marker_index = 1;
NOF_colors = 7;
plot_count = 1;
keyword_transformation = 'rot';
keyword_object = 'WhiteBottle';
for folder = 1:length(folder_list)
    NOF_keypoints = 0;
    NNDR = [];
    euclidean_distance = [];
    files = dir(strcat(rootdir,'\', folder_list(folder).name, '\*.csv'));
    save_name = strcat(rootdir,'\', folder_list(folder).name, '\',keyword_object,keyword_transformation,'PR_merged.png');
    %check which legend (rotation or translation) should be created
    rot = files(1).name(length(files(1).name)-9);
    %load all matches into one array
    for file = 1:length(files)
        filename = strcat(files(file).folder, '\', files(file).name)
        data = load(filename,'-ascii');
        %Don't include the data from where model=scene case FPFH
        if(contains(filename, 'FPFH'))
            if(data(length(data),1) == (length(data)-1))
                continue
            end   
        end
        %Don't include the data from where model=scene case SHOT
        if(data(length(data),1) == length(data))
           continue
        end
        NOF_keypoints = NOF_keypoints + data(length(data),1);
        support_radius = data(length(data),2);
        %Pop last row containing NOF keypoints and 0.5*supportradius
        data(length(data),:) = [];
        %Extract the matches
        NNDR = [NNDR; data(:,1)];
        euclidean_distance = [euclidean_distance; data(:,2)];
    end
    %Sort keypoints by NNDR in ascending order
    [sorted_keypoints, sort_idx] = sort(NNDR, 'ascend'); 
    sorted_euclidean_distance = euclidean_distance(sort_idx);
    %Sort keypoints by euclidean ascending order
%     [sorted_keypoints, sort_idx] = sort(euclidean_distance, 'ascend'); 
%     sorted_euclidean_distance = sorted_keypoints;
   %Calculate TP&FP -> derive precision and recall
   tp = 0;
   fp = 0;
   precision = [];
   recall = [];
   for i = 1:length(NNDR)
       if(sorted_euclidean_distance(i) < support_radius)
           tp = tp + 1;
       else
           fp = fp + 1;
       end
       %only add every 5th value to PR-curve do get better
       %distinguishable curves
       if(mod(i+4, 5) == 0)
           precision = [precision tp/(tp+fp)];
           recall = [recall tp/NOF_keypoints];
       end
   end
   legend_name = strcat(folder_list(folder).name);
   %Visualization
   set(groot, 'DefaultTextInterpreter', 'LaTeX');
   set(groot, 'DefaultAxesTickLabelInterpreter', 'LaTeX');
   set(groot, 'DefaultAxesFontName', 'LaTeX');
   set(groot, 'DefaultAxesFontWeight', 'bold');
   set(groot, 'DefaultLegendInterpreter', 'LaTeX');
   lineS = {'o', '*', 'square', 'diamond', 'p'}';
   idx = 1:length(precision);
   idxq = linspace(min(idx), max(idx), 9);
   interpolated_recall = interp1(idx,recall,idxq, 'spline');
   interpolated_precision = interp1(idx,1-precision,idxq, 'spline');
   
   if(contains(filename, keyword_transformation) && contains(filename, keyword_object))
       if(mod(plot_count-1, NOF_colors) == 0)
           marker_index = marker_index + 1;
       end
       figure(1);
       hold on;
       plot(interpolated_precision,interpolated_recall,'-o','LineWidth',1, 'Displayname', legend_name, 'Marker', lineS{marker_index});
       axis([0 0.2 0 0.7]) %make sure axis goes from 0 to 1
       grid on;
       set(gca,'FontSize',14);
       xlabel('1-Precision','fontsize',15);ylabel('Recall','fontsize',15);
       set(gcf, 'Position',  [100, 100, 1200, 900])
       plot_count = plot_count+1;
   end

end
   legend('show');
   saveas(gcf,save_name)
