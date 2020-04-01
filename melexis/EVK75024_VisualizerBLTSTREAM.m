function [ distRaw, ampl,  stdRaw, stdAmp] = EVK75024_VisualizerBLTSTREAM( filename, numToAverage,modulationAmplitude)
    FileName = filename; % Example .bltstream file in DISTAMP mode
    %FileName = 'EVK75024_Raw_Hand.bltstream'; % Example .bltstream file in RAW mode

    AmplitudeScale = [0 1000];
    DistanceScale = [100 3500];

    % ENTER THE MODE IN WHICH THE RECORDING WAS MADE
    % DistAmp > OperatingMode = 0
    % RawPhases > OperatingMode = 1
    OperatingMode = 0;

    % ENTER THE MODULATION FREQUENCY ON WHICH THE RECORDING WAS MADE
    ModF = modulationAmplitude; %in MHz (only used for RawPhases to distance & amplitude calculation)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DO NOT CHANGE ANYTHING BELOW %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    fprintf('EVK75024 BLTSTREAM Visualizer Program Started !\n');

    [~, configStruct] = BTAinitConfig;
    configStruct.deviceType = 15; %bltstreamID
    configStruct.bltstreamFilename = FileName;
    configStruct.frameQueueMode = 1;
    configStruct.frameQueueLength = 3;

    [status, deviceHandle] = BTAopen(configStruct);

    [~, Frames] = BTAgetLibParam(deviceHandle, 2); %Get no. of frames in bltstream file

    figAmplitude = figure('Name','Amplitude or Confidence Image','NumberTitle','off');
    set(figAmplitude, 'Position', [850, 300, 700, 500]);
    imAmplitude = imagesc(round(rand(240, 320)*AmplitudeScale(2)), [AmplitudeScale(1) AmplitudeScale(2)]);
    set(gca,'XAxisLocation','top');
    set(gca,'XTick',0:40:320); set(gca,'YTick',0:40:240);
    set(gca,'TickDir','out');
    colormap(gray); cbr = colorbar;
    ColorBarStepSize = round((AmplitudeScale(2) - AmplitudeScale(1))/10,0);
    set(cbr,'YTick',AmplitudeScale(1):ColorBarStepSize:AmplitudeScale(2))
    axis vis3d;

    figDistance = figure('Name','Distance Image','NumberTitle','off');
    set(figDistance, 'Position', [50, 300, 700, 500]);
    imDistance = imagesc(round(rand(240, 320)*DistanceScale(2)), [DistanceScale(1) DistanceScale(2)]);
    set(gca,'XAxisLocation','top');
    set(gca,'XTick',0:40:320); set(gca,'YTick',0:40:240);
    set(gca,'TickDir','out');
    set(gca,'color','black'); %Color for NaN pixels
    colormap(flipud(jet));
    cbr = colorbar;
    ColorBarStepSize = round((DistanceScale(2) - DistanceScale(1))/10,0);
    set(cbr,'YTick',DistanceScale(1):ColorBarStepSize:DistanceScale(2))
    axis vis3d;

    figCount = numel(findobj(0,'type','figure'));
    isRunning = figCount;
    frameCount = 1;


    % HSLU
    nRows = 240;
    nCols = 320;
    baseRow = 1;
    data1s = zeros(nRows, nCols, numToAverage);
    data2s = zeros(nRows, nCols, numToAverage);
    data3s = zeros(nRows, nCols, numToAverage);


    while(isRunning == figCount && frameCount < Frames)   
        BTAsetLibParam(deviceHandle, 4, frameCount);
        [~, frameHandle, ~, ~] = BTAgetFrame(deviceHandle, 500);
        switch OperatingMode
            case 1 %Mode 24 = RawPhases
                [~, phase0, ~, ~, ~, ~] = BTAgetChannelData(frameHandle, 0);
                [~, phase180, ~, ~, ~, ~] = BTAgetChannelData(frameHandle, 1);
                [~, phase90, ~, ~, ~, ~] = BTAgetChannelData(frameHandle, 2);
                [~, phase270, ~, ~, ModF, ~] = BTAgetChannelData(frameHandle, 3);

                p0 = TwoComp_MKO(phase0);
                p180 = TwoComp_MKO(phase180);
                p90 = TwoComp_MKO(phase90);
                p270 = TwoComp_MKO(phase270);

                I = p0 - p180;
                Q = p270 - p90;

                ampData = sqrt(I.^2 + Q.^2);
                Phase = atan2(Q, I); %ATAN2 gives results [-pi pi]
                unAmbiguousRange = 0.5*299792458/ModF*1000; %in mm
                coef_rad = unAmbiguousRange / (2*pi);
                distData = (Phase+pi) * coef_rad;  % +pi to compensate negative ATAN2 values
                while sum(distData(distData<0)) ~= 0
                    distData(distData<0) = distData(distData<0) + unAmbiguousRange;
                end

            otherwise % Mode 0 = DistAmp
                [~, ampData, ~, ~, ~] = BTAgetAmplitudes(frameHandle);
                [~, distData, ~, ~, ~] = BTAgetDistances(frameHandle);  
        end
        BTAfreeFrame(frameHandle);

        set(imAmplitude,'CData',ampData);
        set(imDistance,'CData',distData);

        pause(0.06);
        isRunning = numel(findobj(0,'type','figure'));
        frameCount = frameCount + 1;
        data1s(:,:,frameCount) = distData;
        data2s(:,:,frameCount) = ampData;

        if(frameCount == 41)
            break
        end
    end

    distRaw = mean(data1s,3);
    ampl = mean(data2s,3);

    stdRaw = std(data1s, 0, 3);
    stdAmp = std(data2s, 0, 3);

    figure(1);
    surf(distRaw(50:200,100:250),ampl(50:200,100:250))
    colormap(gray);
    title('Raw Distance');

    BTAclose(deviceHandle);

    close all;
    fprintf('EVK75024 BLTSTREAM Visualizer Program Terminated !\n');

end