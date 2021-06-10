%%
clc
clear
close all

load velocity.txt
load acceleration.txt

%%
% t = (0:1/30:(max(size(velocity))-1)/30)';
% stopTime = (max(size(velocity))-1)/30;

%% ���ý�ȡ�ĳ���
number = 1317;
t = 0:1/30:number/30;

vx = velocity(334:number + 334,1);
vy = velocity(334:number + 334,2);    
vz = velocity(334:number + 334,3);
wx = velocity(334:number + 334,4);
wy = velocity(334:number + 334,5);
wz = velocity(334:number + 334,6);

ax = acceleration(334:number + 334,1);
ay = acceleration(334:number + 334,2);
az = acceleration(334:number + 334,3);

%% ����ͼ��
% �������ִ�С
fontsize = 16;

figure(1)
subplot(211)
plot(t, vx, 'r',t, vy, 'g-.',t, vz,'b--');

% x��ı�ǩ
xlabel('Time [sec]', 'fontsize', fontsize) % �����������˵��
% y��ı�ǩ
ylabel('Linear velocity [m/s]', 'fontsize', fontsize) % �����������˵��
% ��ʾ
axis([0 number/30 -0.1 0.1]);

h = gca; % ��ȡ��ǰ��ͼ�����ָ��
% �趨�����С
set(h,'FontSize',fontsize); % �������ִ�С��ͬʱӰ���������ע��ͼ��������ȡ�
set(get(gca,'Children'),'linewidth',2);%����ͼ���߿�1.5��

subplot(212)
plot(t,wx,'r',t,wy,'g-.',t,wz,'b--');
% x��ı�ǩ
xlabel('Time [sec]', 'fontsize', fontsize) % �����������˵��
% y��ı�ǩ
ylabel('Angular velocity [rad/s]', 'fontsize', fontsize) % �����������˵��
% ��ʾ
axis([0 number/30  -0.006 0.006]);

h = gca; % ��ȡ��ǰ��ͼ�����ָ��
% �趨�����С
set(h,'FontSize',fontsize); % �������ִ�С��ͬʱӰ���������ע��ͼ��������ȡ�
set(get(gca,'Children'),'linewidth',2);%����ͼ���߿�1.5��

figure(2)
plot(t,ax,'r',t,ay,'g-.',t,az,'b--');
xlabel('Time [sec]', 'fontsize', fontsize) % �����������˵��
% y��ı�ǩ
ylabel('Linear acceleration [m/s^2]', 'fontsize', fontsize) % �����������˵��
% ��ʾ
axis([0 number/30  -0.2 0.2]);

h = gca; % ��ȡ��ǰ��ͼ�����ָ��
% �趨�����С
set(h,'FontSize',fontsize); % �������ִ�С��ͬʱӰ���������ע��ͼ��������ȡ�
set(get(gca,'Children'),'linewidth',2);%����ͼ���߿�1.5��
