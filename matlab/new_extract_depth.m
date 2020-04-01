clear 'all';
close 'all';

addpath('melexis')
directory = 'Aufnahmen\20_03_20\picture_streams\Gipfeli\0\Rot\*';
saveDir = 'Aufnahmen\20_03_20\filtered_depth\Gipfeli\0_Rot\';
pictureDir = 'Aufnahmen\20_03_20\pictures\';
numToAverage = 40;

folders = dir(directory);
files = dir(strcat(folders(1).folder, '\', folders(1).name, '\*.bltstream'));

% Read from Melexis Settings
modulationAmplitude = 40;

fileBgr = dir(strcat(files(1).folder, '\', files(length(files)).name));
[background,ampl_backround, stdRaw_background, stdAmp_background] = EVK75024_VisualizerBLTSTREAM(strcat(fileBgr(1).folder, '\', fileBgr(1).name),numToAverage,modulationAmplitude);
for i0=1:(length(files)-1)
    
    fileImg = dir(strcat(files(i0).folder, '\', files(i0).name)); 
    [distRaw,ampl, stdRaw, stdAmp] = EVK75024_VisualizerBLTSTREAM(strcat(fileImg(1).folder, '\', fileImg(1).name),numToAverage,modulationAmplitude);
        
%     data1c = distRaw - background;      
%     data1c(data1c > 10) = 0;          
%     roi = [100 220, 34 215]
%     data1f = medfilt2(data1c, [3,3]);
%     
%     figure(3);
%     surf(-data1f(roi(3):sub:roi(4), roi(1):sub:roi(2)), ampl(roi(3):sub:roi(4), roi(1):sub:roi(2)));
%     colormap(gray);
%     view(azimut,elevation);
%     title('filtered distance');
%     axis equal;
%     set(3, 'Position', Pfull);

%    fileName = strcat(fileImg(1).folder, '\', fileImg(1).name);  
    M = medfilt2(distRaw);
    fileSave = strcat(saveDir, '\Model\', fileImg(1).name(1:end-10), '.txt');
    writematrix(M,fileSave);
    
    B = medfilt2(background);
    fileSave = strcat(saveDir, '\Background\', fileImg(1).name(1:end-10), '.txt');
    writematrix(B,fileSave);
    
    M = uint8(ampl);
    fileSave = strcat(pictureDir, fileImg(1).name(1:end-10), '.jpeg');
    imwrite(M,fileSave);

end

   
