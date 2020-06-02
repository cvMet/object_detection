clear all;
rootdir = '..\stats\';
folder_list = rdir([rootdir, '\**\*.'], 'regexp(name, ''angles'')', true);

for folder = 1:length(folder_list)
    files = dir(strcat(folder_list(folder).name, '\*.csv'));
    %load all matches into one array
    for file = 1:length(files)
        filename = strcat(files(file).folder, '\', files(file).name)
        %Search for a pattern in filename (e.g. specific rotation), ignore
        %file if pattern is not found
        filehandle = fopen(filename,'rt');
        data = textscan(filehandle,'%s%s%s%s', 'Delimiter', ',');
        fclose(filehandle);
        phi = data{3}(:);
        S = sprintf('%s ', phi{:});
        D = sscanf(S, '%f');
        phi_15 = D(1:9);
        phi_30 = D(10:18);
        figure(1)
        boxplot(abs(phi_15))
        xlabel('M26 15° Scenes')
        ylabel('Estimated Rotation [°]')
        title('ICP Pose Estimation')
        figure(2)
        boxplot(abs(phi_30))
        xlabel('M26 30° Scenes')
        ylabel('Estimated Rotation [°]')
        title('ICP Pose Estimation')

    end
end
