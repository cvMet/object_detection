clear all;
x_axis = 1;
y_axis = 1;
rootdir = '..\PR\Buch\object_threshold_eval\weizenbroetchen_sequence\';
folder_list = rdir([rootdir, '\**\*.'], 'regexp(name, ''SHOT'')', true);
files = dir(strcat(folder_list(1).name, '\*.csv'));
groundtruth_hard = [ones(27,1);zeros(5,1);ones(15,1);zeros(50,1)];
groundtruth = [ones(15,1);zeros(55,1)];
for file = 1:length(files)
    filename = strcat(files(file).folder, '\', files(file).name);
    data = load(filename,'-ascii');
    NOF_matches = data(:,2);
    %Start with threshold=0 and increment till max#matches
    max_matches = max(NOF_matches);
    P = sum(groundtruth);
    precision = [];
    recall = [];
    for threshold = 0:(max_matches)
        tp = 0;
        fp = 0;
        for i = 1:length(NOF_matches)
            if(NOF_matches(i) >= threshold)
                if(groundtruth(i))
                    tp = tp + 1;
                else
                    fp = fp + 1;
                end
            end
        end
        precision = [precision tp/(tp+fp)];
        recall = [recall tp/P];
    end
    %Visualization
    legend_name = files(file).name;
    set(groot, 'DefaultTextInterpreter', 'LaTeX');
    set(groot, 'DefaultAxesTickLabelInterpreter', 'LaTeX');
    set(groot, 'DefaultAxesFontName', 'LaTeX');
    set(groot, 'DefaultAxesFontWeight', 'bold');
    set(groot, 'DefaultLegendInterpreter', 'LaTeX');
    figure(file);
    plot(1-precision,recall,'-o','LineWidth',1, 'Displayname', legend_name, 'color', [ 0.9100 0.4100 0.1700]);
    a = [0:max_matches]'; b = num2str(a); c = strcat('__',cellstr(b));
    text(1-precision,recall,c,'VerticalAlignment','bottom','HorizontalAlignment','left')
    axis([0 x_axis 0 y_axis]);
    grid on;
    set(gca,'FontSize',14);
    xlabel('1-Precision','fontsize',15);ylabel('Recall','fontsize',15);
    set(gcf, 'Position',  [100, 100, 1200, 900]);
    legend('show');
end

