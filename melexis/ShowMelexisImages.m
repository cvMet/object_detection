clear 'all';
close 'all';

Sample = 1;
hasbwroi = false;

% Black & White Background
if Sample == 1
    directory = 'C:\Users\cc-isn\Downloads\Melexis\Frames\BW_i_48_mod_40_it_450us.bltstream';
    backGroundData = '';
    subtractBG = false;
    numToAverage = [40];
    hasbwroi = true;
    roi = [80 250, 30 180];
    noise_roi_w = [roi(1)+60 roi(1)+120,roi(3)+100 roi(3)+150 ]
    noise_roi_b = [roi(1)+60 roi(1)+120, roi(3)+25 roi(3)+75]
    modulationAmplitude = 40;% MHZ
% Parts
elseif Sample == 2
    directory = 'C:\Users\cc-isn\Downloads\Melexis\Frames\Parts_i_48_mod_40_it_200us.bltstream';
    backGroundData = 'C:\Users\cc-isn\Downloads\Melexis\Frames\Back_i_48_mod_40_it_200us.bltstream';
    subtractBG = true;
    numToAverage = [40];
    roi = [100 250, 50 200];
    modulationAmplitude = 40;% MHZ
end

[distRaw,ampl, stdRaw, stdAmp] = EVK75024_VisualizerBLTSTREAM(directory,numToAverage,modulationAmplitude);
if subtractBG
    [distRawBack,amplBack, stdRawBack, stdAmpBack] = EVK75024_VisualizerBLTSTREAM(backGroundData,numToAverage,modulationAmplitude);
    distRaw = distRaw-distRawBack;
    
    figure(1);
    surf(-distRaw(roi(3):roi(4), roi(1):roi(2)), ampl(roi(3):roi(4), roi(1):roi(2)));
    colormap(gray);
    title('Raw Distance');
else
    background_ = medfilt2(distRaw, [7,1]);
    background_ = medfilt2(background_, [1,7]);   
    distRaw_ = distRaw - background_
    
    figure(1);
    surf(-distRaw,ampl);
    colormap(gray)
    
    figure(2);
    roi = [80 250, 30 180];
    subplot(2,2,[1,3]);
    surf(-distRaw_(roi(3):roi(4),roi(1):roi(2)),2*ampl(roi(3):roi(4), roi(1):roi(2)));
    colormap(gray)
    zlabel('Distance [mm]')
    xlabel('[px]')
    ylabel('[px]')
    title('Raw distance');
    set(gca,'FontSize',18) 
    subplot(2,2,2);
    plot(flip(distRaw_(roi(3):roi(4),80)))
    title('Transition')
    ylabel('Distance [mm]')
    xlabel('[px]')
    set(gca,'FontSize',18) 
    
end

if hasbwroi
    figure(3);
    copy = medfilt2(distRaw, [5,1]);
    copy = medfilt2(copy, [1,5]);
    
    distRaw_ = distRaw - copy


    subplot(2,2,1);
    surf(-distRaw_(noise_roi_w(3):noise_roi_w(4), noise_roi_w(1):noise_roi_w(2)), 2*ampl(noise_roi_w(3):noise_roi_w(4), noise_roi_w(1):noise_roi_w(2)));
    colormap(gray);
    zlabel('Distance [mm]')
    xlabel('[px]')
    ylabel('[px]')
    title('Raw distance white part');
    axis([0 60 0 50 -30 30])
    set(gca,'FontSize',18)

    subplot(2,2,2);
    surf(-distRaw_(noise_roi_b(3):noise_roi_b(4), noise_roi_b(1):noise_roi_b(2)), ampl(noise_roi_b(3):noise_roi_b(4), noise_roi_b(1):noise_roi_b(2)));
    colormap(gray);
    title('Raw distance black part');
    zlabel('Distance [mm]')
    xlabel('[px]')
    ylabel('[px]')
    axis([0 60 0 50 -30 30])
    set(gca,'FontSize',18)



    std_white = std(distRaw_(noise_roi_w(3):noise_roi_w(4), noise_roi_w(1):noise_roi_w(2)),1,'all')
    std_black = std(distRaw_(noise_roi_b(3):noise_roi_b(4), noise_roi_b(1):noise_roi_b(2)),1,'all')

    x = linspace(1,51,50);
    std_white_ = std_white *ones(length(x),1);
    std_black_ = std_black *ones(length(x),1); 


    subplot(2,2,3)
    plot(-distRaw_(noise_roi_w(3):noise_roi_w(4), 160))
    hold on
    plot(x,std_white_)
    title('Profile white');
    ylabel('Distance [mm]');
    xlabel('[px]');
    axis([1 50 -30 30]);
    legend('profile','std')
    set(gca,'FontSize',18)
    hold off


    subplot(2,2,4)
    plot(-distRaw_(noise_roi_b(3):noise_roi_b(4),160))
    hold on
    plot(x,std_black_)
    title('Profile black')
    ylabel('Distance [mm]')
    xlabel('[px]')
    axis([1 50 -30 30])
    legend('profile','std')
    set(gca,'FontSize',18)
    hold off
end






    
    