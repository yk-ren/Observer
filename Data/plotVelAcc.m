%%
clc
clear
close all

load velocity.txt
load acceleration.txt

%%
% t = (0:1/30:(max(size(velocity))-1)/30)';
% stopTime = (max(size(velocity))-1)/30;

%% 设置截取的长度
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

%% 绘制图形
% 定义文字大小
fontsize = 16;

figure(1)
subplot(211)
plot(t, vx, 'r',t, vy, 'g-.',t, vz,'b--');

% x轴的标签
xlabel('Time [sec]', 'fontsize', fontsize) % 给横坐标轴加说明
% y轴的标签
ylabel('Linear velocity [m/s]', 'fontsize', fontsize) % 给纵坐标轴加说明
% 显示
axis([0 number/30 -0.1 0.1]);

h = gca; % 获取当前绘图坐标的指针
% 设定字体大小
set(h,'FontSize',fontsize); % 设置文字大小，同时影响坐标轴标注、图例、标题等。
set(get(gca,'Children'),'linewidth',2);%设置图中线宽1.5磅

subplot(212)
plot(t,wx,'r',t,wy,'g-.',t,wz,'b--');
% x轴的标签
xlabel('Time [sec]', 'fontsize', fontsize) % 给横坐标轴加说明
% y轴的标签
ylabel('Angular velocity [rad/s]', 'fontsize', fontsize) % 给纵坐标轴加说明
% 显示
axis([0 number/30  -0.006 0.006]);

h = gca; % 获取当前绘图坐标的指针
% 设定字体大小
set(h,'FontSize',fontsize); % 设置文字大小，同时影响坐标轴标注、图例、标题等。
set(get(gca,'Children'),'linewidth',2);%设置图中线宽1.5磅

figure(2)
plot(t,ax,'r',t,ay,'g-.',t,az,'b--');
xlabel('Time [sec]', 'fontsize', fontsize) % 给横坐标轴加说明
% y轴的标签
ylabel('Linear acceleration [m/s^2]', 'fontsize', fontsize) % 给纵坐标轴加说明
% 显示
axis([0 number/30  -0.2 0.2]);

h = gca; % 获取当前绘图坐标的指针
% 设定字体大小
set(h,'FontSize',fontsize); % 设置文字大小，同时影响坐标轴标注、图例、标题等。
set(get(gca,'Children'),'linewidth',2);%设置图中线宽1.5磅
