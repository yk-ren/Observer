clc
clear
close all

load trueDepth.txt
load estiDepth.txt

%% plot the figure
% t = (0:1/30:(max(size(trueDepth))-1)/30)';
% stopTime = (max(size(trueDepth))-1)/30;
number = 1317;
t = 0:1/30:number/30;

trueDepth1 = trueDepth(334:number + 334,1);
trueDepth2 = trueDepth(334:number + 334,2);    
trueDepth3 = trueDepth(334:number + 334,3);
trueDepth4 = trueDepth(334:number + 334,4);

estimatedDepth1 = estiDepth(334:number + 334,1);
estimatedDepth2 = estiDepth(334:number + 334,2);
estimatedDepth3 = estiDepth(334:number + 334,3);
estimatedDepth4 = estiDepth(334:number + 334,4);

% �������ִ�С
fontsize = 16;

%% 
figure(1)
plot(t, trueDepth1-estimatedDepth1,'r','LineWidth',1.5)
hold on
plot(t, trueDepth2-estimatedDepth2,'g','LineWidth',1.5)
plot(t, trueDepth3-estimatedDepth3,'b','LineWidth',1.5)
plot(t, trueDepth4-estimatedDepth4,'k','LineWidth',1.5)
grid off
%legend("Mass center 1","Mass center 2","Mass center 3","Mass center 4");
hold off

xlabel('Time [sec]', 'fontsize', fontsize) % �����������˵��
% y��ı�ǩ
ylabel('Estimation errors [m]', 'fontsize', fontsize) % �����������˵��
% ��ʾ
axis([0 number/30 -0.1 0.5]);

h = gca; % ��ȡ��ǰ��ͼ�����ָ��
% �趨�����С
set(h,'FontSize',fontsize); % �������ִ�С��ͬʱӰ���������ע��ͼ��������ȡ�
set(get(gca,'Children'),'linewidth',1.5);%����ͼ���߿�1.5��