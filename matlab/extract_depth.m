clear 'all';
close 'all';

addpath('..\melexis')
%Choose which elements of dataset should get extracted
objects = ["Weizenbroetchen_600_6_White_520"];

for object=1:(length(objects))
    directory = strcat('..\datasets\object_threshold_eval\picture_streams\',objects(object),'\*');
    saveDir = strcat('..\datasets\object_threshold_eval\raw_depth\',objects(object),'\');
    pictureDir = strcat('..\datasets\object_threshold_eval\images\',objects(object),'\');
    numToAverage = 40;
    folders = dir(directory);
    files = dir(strcat(folders(1).folder, '\', folders(1).name, '\*.bltstream'));
    % Read from Melexis Settings
    modulationAmplitude = 40;
    for i0=1:(length(files))
        fileImg = dir(strcat(files(i0).folder, '\', files(i0).name));
        [distRaw,ampl, stdRaw, stdAmp] = EVK75024_VisualizerBLTSTREAM(strcat(fileImg(1).folder, '\', fileImg(1).name),numToAverage,modulationAmplitude);
        
        %     data1c = distRaw - background;
        %     data1c(data1c > 10) = 0;
        %     roi = [100 220, 34 215]
        %     data1f = medfilt2(data1c, [3,3]);
        %     figure(3);
        %     surf(-data1f(roi(3):sub:roi(4), roi(1):sub:roi(2)), ampl(roi(3):sub:roi(4), roi(1):sub:roi(2)));
        %     colormap(gray);
        %     view(azimut,elevation);
        %     title('filtered distance');
        %     axis equal;
        %     set(3, 'Position', Pfull);
        %    fileName = strcat(fileImg(1).folder, '\', fileImg(1).name);
        
        M = distRaw;
        fileSave = strcat(saveDir, fileImg(1).name(1:end-10), '.txt');
        writematrix(M,fileSave);
        
        M = uint8(ampl);
        fileSave = strcat(pictureDir, fileImg(1).name(1:end-10), '.bmp');
        imwrite(M,fileSave);
        %  end
    end
end

