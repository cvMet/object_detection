%The following script generates the PR-Curve based on the proposition of
%buch et al. All matches of a dataset (containing several models and scenes)
%are plotted in one graph as individual curves DIsplay different files from
%folder seperately
clear all;
rootdir = '..\PR\Buch\09_04_20\gipfeli\3d_filtered\B_SHOT\iss5_th0.9\0_tran';
folder_list = rdir([rootdir, '\**\*.'], 'regexp(name, ''iss\d'')', true);

NOF_colors = 8;
marker_index = 1;
stepsize = 1;
x_axis = 1;
y_axis = 0.1;
PR_graph_path = strcat(rootdir,'\','PR_seperated.png');
for folder = 1:1
    files = dir(strcat(rootdir,'\', folder_list(folder).name, '\*.csv'));
    for i0=1:length(files)
        filename = strcat(files(i0).folder, '\', files(i0).name);
        iamge_name = strcat(files(i0).folder, '\', files(i0).name, '.png');
        data = load(filename,'-ascii');
        %Ignore files with no Matches but include Number of Keypoints
        if(isvector(data))
            NOF_keypoints = NOF_keypoints + data(1);
            continue
        end
        %Don't include the data if model=scene case SHOT
        if(data(length(data),1) == length(data))
            continue
        end
        NOF_keypoints = data(length(data),1);
        support_radius = data(length(data),2);
        %Pop last row containing NOF keypoints and 0.5*supportradius
        data(length(data),:) = [];
        NNDR = data(:,1);
        euclidean_distance = data(:,2);
        [sorted_keypoints, sort_idx] = sort(NNDR, 'ascend');
        sorted_euclidean_distance = euclidean_distance(sort_idx);
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
            %Define granularity of the curve by determing step size
            if(mod(i, stepsize) == 0)
                precision = [precision tp/(tp+fp)];
                recall = [recall tp/NOF_keypoints];
            end
        end
        legend_name = strcat(files(i0).name);
        %Change marker if all line colors were already used in graph
        if(mod(i0, NOF_colors) == 0)
            marker_index = marker_index + 1;
        end
        
        %Visualization
        set(groot, 'DefaultTextInterpreter', 'LaTeX');
        set(groot, 'DefaultAxesTickLabelInterpreter', 'LaTeX');
        set(groot, 'DefaultAxesFontName', 'LaTeX');
        set(groot, 'DefaultAxesFontWeight', 'bold');
        set(groot, 'DefaultLegendInterpreter', 'LaTeX');
        lineS = {'o', 'square', 'diamond', '*', 'p'}';
        idx = 1:length(precision);
        idxq = linspace(min(idx), max(idx), 3*length(precision));
        interpolated_recall = interp1(idx,recall,idxq, 'pchip');
        interpolated_precision = interp1(idx,1-precision,idxq, 'pchip');
        
        figure(1);
        hold on
        plot(interpolated_precision,interpolated_recall,'-o','LineWidth',1, 'Displayname', legend_name, 'Marker', lineS{marker_index});
        axis([0 x_axis 0 y_axis]);
        grid on;
        set(gca,'FontSize',14);
        xlabel('1-Precision','fontsize',15);ylabel('Recall','fontsize',15);
        set(gcf, 'Position',  [100, 100, 800, 700])
    end
end
legend('show');
saveas(gcf,PR_graph_path)
