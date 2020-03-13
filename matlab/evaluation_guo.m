%Evaluation of the descriptors
clear 'all';clc;
set(groot, 'DefaultTextInterpreter', 'LaTeX');
set(groot, 'DefaultAxesTickLabelInterpreter', 'LaTeX');
set(groot, 'DefaultAxesFontName', 'LaTeX');
set(groot, 'DefaultAxesFontWeight', 'bold');
set(groot, 'DefaultLegendInterpreter', 'LaTeX');

filename_shot = '..\PR\001_Bottlewhite1model.csv';

n = 1;
[precision_shot, recall_shot] = calcPR_guo(filename_shot,n);

idx = 1:length(recall_shot);
idxq = linspace(min(idx), max(idx), 10);
recall_shot = interp1(idx,recall_shot,idxq, 'linear');
precision_shot = interp1(idx,1-precision_shot,idxq, 'linear');

figure;
plot(precision_shot,recall_shot,'-o','LineWidth',1);
axis([0 1 0 1]) %make sure axis goes from 0 to 1
grid on;
set(gca,'FontSize',14);
legend('B-SHOT');
xlabel('1-Precision','fontsize',15);ylabel('Recall','fontsize',15);

