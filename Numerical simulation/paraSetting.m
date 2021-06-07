clc
clear
close all
warning off

%% 测试选择
% 测试1.该测试用于验证观测器的收敛性,是否收敛于任意大的初值
% 测试2.该测试用于验证观测器的收敛速率
% 测试3.该测试用于验证观测器不满足可观测条件时的奇异性
% 测试4.该测试用于验证观测器的鲁棒性
modelSelection = 4;

% 设定所有的观测器针对的相机焦距均为 20
focalLen = 20;

if modelSelection == 1
    % 设置仿真步长
    timeStep = 0.001;
    % 设置仿真时间
    timeStop = 1;
    
    % 设置各观测器的参数
    % A New solution to the Problem of Range Identification In  Perspective Vision Systems
    % 2005 IEEE AC; Author:  Dimitrios Karagiannis et al. 
    % 给定观测器系数
    lamda5 = 0.35;
    
    % On-line Estimation of Feature Depth for Image-Based Visual Servoing Schemes
    % 2008 IJRR; Author:  Alessandro De Luca et al. 
    % 给定观测器系数
    k81 = 15;
    k82 = 15;
    k83 = 2;

    % Range estimation from a moving camera: an Immersion and Invariance approach
    % 2009 ICRA; Author:  Fabio Morbidi et al.
    % 给定观测器系数
    M9 = 0.06;

    % Globally exponentially stable observer for vision-based range estimation
    % 2012 Mechatronics; Author: A.P. Dani et al.
    k123 = 0.6;
   
    % Proposed observer
    % 2020
    % 给定观测系数
    rou20 = 0.31;
    b20 = 35;
    
    % 对模型进行仿真
    sim('AllObserverComparisonTest1.slx');
%     sim('AllObserverComparisonTest1_08.slx')
elseif modelSelection == 2
    % 设置仿真步长
    timeStep = 0.001;
    % 设置仿真时间
    timeStop = 1;
    
    % 设置各观测器的参数
    % A New solution to the Problem of Range Identification In  Perspective Vision Systems
    % 2005 IEEE AC; Author:  Dimitrios Karagiannis et al. 
    % 给定观测器系数
    lamda5 = 0.6;
    
    % Feature Depth Observation for Image-based Visual Servoing: Theory and Experiments
    % 2008 IJRR; Author:  Alessandro De Luca et al.
    % 给定观测器系数
    k81 = 60;
    k82 = 60;
    k83 = 20;

    % Range estimation from a moving camera: an Immersion and Invariance approach
    % 2009 ICRA; Author:  Fabio Morbidi et al.
    % 给定观测器系数
    M9 = 0.095;
    
    % Globally exponentially stable observer for vision-based range estimation
    % 2012 Mechatronics; Author: A.P. Dani et al.
    k123 = 0.8;

    % Proposed observer
    % 2020
    % 给定观测系数
    rou20 = 0.41;
    b20 = 40;
    
    % 对模型进行仿真
    sim('AllObserverComparisonTest234.slx');    
elseif modelSelection == 3
    % 设置仿真步长
    timeStep = 0.001;
    % 设置仿真时间
    timeStop = 1;
    
    % 设置各观测器的参数
    % A New solution to the Problem of Range Identification In  Perspective Vision Systems
    % 2005 IEEE AC; Author:  Dimitrios Karagiannis et al. 
    % 给定观测器系数
    lamda5 = 0.45;
    
    % On-line Estimation of Feature Depth for Image-Based Visual Servoing Schemes
    % 2008 IJRR; Author:  Alessandro De Luca et al.
    % 给定观测器系数
    k81 = 50;
    k82 = 50;
    k83 = 15;

    % Range estimation from a moving camera: an Immersion and Invariance approach
    % 2009 ICRA; Author:  Fabio Morbidi et al.
    % 给定观测器系数
    M9 = 0.05;
    
    % Globally exponentially stable observer for vision-based range estimation
    % 2012 Mechatronics; Author: A.P. Dani et al.
    k123 = 0.5;    

    % Proposed observer
    % 2020, Xiangfei Li et al. 
    % 给定观测系数
    rou20 = 0.15;
    b20 = 25;
    
    % 对模型进行仿真
    sim('AllObserverComparisonTest234.slx');
elseif modelSelection == 4
    % 设置仿真步长
    timeStep = 0.01;
    % 设置仿真时间
    timeStop = 4;
    
    % 设置各观测器的参数
    % A New solution to the Problem of Range Identification In  Perspective Vision Systems
    % 2005 IEEE AC; Author:  Dimitrios Karagiannis et al. 
    % 给定观测器系数
    lamda5 = 0.15;
    
    % On-line Estimation of Feature Depth for Image-Based Visual Servoing Schemes
    % 2008 IJRR; Author:  Alessandro De Luca et al. 
    % 给定观测器系数
    k81 = 30;
    k82 = 30;
    k83 = 3.6;

    % Range estimation from a moving camera: an Immersion and Invariance approach
    % 2009 ICRA; Author:  Fabio Morbidi et al.
    % 给定观测器系数
    M9 = 0.03;

    % Globally exponentially stable observer for vision-based range estimation
    % 2012 Mechatronics; Author: A.P. Dani et al.
    k123 = 0.12;
    
    % Proposed observer
    % 2020, Xiangfei Li et al. 
    % 给定观测系数
    rou20 = 0.018;
    b20 = 40;
    
    % 对模型进行仿真
    sim('AllObserverComparisonTest234.slx');
else
    % 其他模型，用于初步测试观测器的参数
    % 暂时无观测器测试参数
end

%% 绘制图形
% 定义文字大小
fontsize = 16;

% 选择需要载入的数据
if modelSelection == 1
    % 获取各深度值
    time = allDepths.time;
    lengthData = length(time);
    trueDepth = reshape(allDepths.signals.values(:,1,:), lengthData, 1);
    depth05 = reshape(allDepths.signals.values(:,2,:), lengthData, 1);
    depth09 = reshape(allDepths.signals.values(:,3,:), lengthData, 1);
    depth12 = reshape(allDepths.signals.values(:,4,:), lengthData, 1);
    depth20 = reshape(allDepths.signals.values(:,5,:), lengthData, 1);
    
    % 绘制深度图
    figure(1)
    plot(time, trueDepth, 'k');
    hold on
    plot(time, depth05, 'color', [0.00,1.00,1.00]);   
    plot(time, depth09, 'g--',time, depth12, 'b-.',time, depth20, 'r:');   
    
    % x轴的标签
    xlabel('Time [sec]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给横坐标轴加说明
    % y轴的标签
    ylabel('Depths [m]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给纵坐标轴加说明
    % 添加图例
    legend('True depth', 'Observer in [23]','Observer in [27]', 'Observer in [29]','Proposed observer', 'Location', 'southeast', 'FontName', 'Times New Roman');

    h = gca; % 获取当前绘图坐标的指针
    % 设定字体大小
    set(h,'FontSize',fontsize, 'FontName', 'Times New Roman'); % 设置文字大小，同时影响坐标轴标注、图例、标题等。
    set(get(gca,'Children'),'linewidth',2);%设置图中线宽2磅

    figure(2)
    plot(time, trueDepth-depth05, 'color',[0.00,1.00,1.00]);
    hold on;
    plot(time, trueDepth-depth09, 'g--', time, trueDepth-depth12, 'b-.',time, trueDepth-depth20, 'r:'); 
    % x轴的标签
    xlabel('Time [sec]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给横坐标轴加说明
    % y轴的标签
    ylabel('Estimation errors [m]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给纵坐标轴加说明
    % 添加图例
    legend('Observer in [23]','Observer in [27]', 'Observer in [29]','Proposed observer', 'Location', 'north', 'FontName', 'Times New Roman');

    h = gca; % 获取当前绘图坐标的指针
    % 设定字体大小
    set(h,'FontSize',fontsize, 'FontName', 'Times New Roman'); % 设置文字大小，同时影响坐标轴标注、图例、标题等。
    set(get(gca,'Children'),'linewidth',2);%设置图中线宽2磅
        
elseif modelSelection == 2
    % 获取各深度值
    time = allDepths.time;
    lengthData = length(time);
    trueDepth = reshape(allDepths.signals.values(:,1,:), lengthData, 1);
    depth05 = reshape(allDepths.signals.values(:,2,:), lengthData, 1);
    depth08 = reshape(allDepths.signals.values(:,3,:), lengthData, 1);
    depth09 = reshape(allDepths.signals.values(:,4,:), lengthData, 1);
    depth12 = reshape(allDepths.signals.values(:,5,:), lengthData, 1);
    depth20 = reshape(allDepths.signals.values(:,6,:), lengthData, 1);
    
    % 绘制深度图
    figure(1)
    plot(time, trueDepth, 'k');
    hold on
    plot(time, depth08, 'color', [0.72,0.27,1.00]); 
    plot(time, depth05, 'color', [0.00,1.00,1.00]);
    plot(time, depth09, 'g--',time, depth12, 'b-.',time, depth20, 'r:');
    
    % x轴的标签
    xlabel('Time [sec]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给横坐标轴加说明
    % y轴的标签
    ylabel('Depths [m]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给纵坐标轴加说明
    % 添加图例
    legend('True depth', 'Observer in [22]', 'Observer in [23]','Observer in [27]', 'Observer in [29]','Proposed observer', 'Location', 'southeast', 'FontName', 'Times New Roman');

    h = gca; % 获取当前绘图坐标的指针
    % 设定字体大小
    set(h,'FontSize',fontsize, 'FontName', 'Times New Roman'); % 设置文字大小，同时影响坐标轴标注、图例、标题等。
    set(get(gca,'Children'),'linewidth',2);%设置图中线宽2磅

    figure(2)
    plot(time, trueDepth-depth08,'color', [0.72,0.27,1.00])
    hold on
    plot(time, trueDepth-depth05,'color', [0.00,1.00,1.00])    
    plot(time, trueDepth-depth09, 'g--', time, trueDepth-depth12, 'b-.',time, trueDepth-depth20, 'r:'); 
    % x轴的标签
    xlabel('Time [sec]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给横坐标轴加说明
    % y轴的标签
    ylabel('Estimation errors [m]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给纵坐标轴加说明
    % 添加图例
    legend('Observer in [22]', 'Observer in [23]','Observer in [27]', 'Observer in [29]','Proposed observer', 'Location', 'north', 'FontName', 'Times New Roman');

    h = gca; % 获取当前绘图坐标的指针
    % 设定字体大小
    set(h,'FontSize',fontsize, 'FontName', 'Times New Roman'); % 设置文字大小，同时影响坐标轴标注、图例、标题等。
    set(get(gca,'Children'),'linewidth',2);%设置图中线宽2磅
 
elseif modelSelection == 3
    % 获取各深度值
    time = allDepths.time;
    lengthData = length(time);
    trueDepth = reshape(allDepths.signals.values(:,1,:), lengthData, 1);
    depth05 = reshape(allDepths.signals.values(:,2,:), lengthData, 1);
    depth08 = reshape(allDepths.signals.values(:,3,:), lengthData, 1);
    depth09 = reshape(allDepths.signals.values(:,4,:), lengthData, 1);
    depth12 = reshape(allDepths.signals.values(:,5,:), lengthData, 1);
    depth20 = reshape(allDepths.signals.values(:,6,:), lengthData, 1);
    
    % 绘制深度图
    figure(1)
    plot(time, trueDepth, 'k');
    hold on
    plot(time, depth08, 'color', [0.72,0.27,1.00]);
    plot(time, depth05, 'color', [0.00,1.00,1.00]);
    plot(time, depth12, 'b-.',time, depth20, 'r:');
    
    % x轴的标签
    xlabel('Time [sec]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给横坐标轴加说明
    % y轴的标签
    ylabel('Depths [m]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给纵坐标轴加说明
    % 添加图例
    legend('True depth', 'Observer in [22]', 'Observer in [23]', 'Observer in [29]','Proposed observer', 'Location', 'southeast', 'FontName', 'Times New Roman');

    h = gca; % 获取当前绘图坐标的指针
    % 设定字体大小
    set(h,'FontSize',fontsize, 'FontName', 'Times New Roman'); % 设置文字大小，同时影响坐标轴标注、图例、标题等。
    set(get(gca,'Children'),'linewidth',2);%设置图中线宽2磅
    
    figure(2)
    plot(time, trueDepth - depth08, 'color', [0.72,0.27,1.00]);
    hold on;
    plot(time, trueDepth - depth05, 'color', [0.00,1.00,1.00]);
    plot(time, trueDepth - depth12, 'b-.',time, trueDepth - depth20, 'r:');
    % x轴的标签
    xlabel('Time [sec]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给横坐标轴加说明
    % y轴的标签
    ylabel('Estimation errors [m]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给纵坐标轴加说明
    % 添加图例
    legend('Observer in [22]', 'Observer in [23]','Observer in [29]','Proposed observer', 'Location', 'north', 'FontName', 'Times New Roman');

    h = gca; % 获取当前绘图坐标的指针
    % 设定字体大小
    set(h,'FontSize',fontsize, 'FontName', 'Times New Roman'); % 设置文字大小，同时影响坐标轴标注、图例、标题等。
    set(get(gca,'Children'),'linewidth',2);%设置图中线宽2磅
    ylim([-0.02,0.8]);
    
    figure(3)
    plot(time, trueDepth, 'k');
    hold on
    plot(time, depth09, 'g--');
    % x轴的标签
    xlabel('Time [sec]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给横坐标轴加说明
    % y轴的标签
    ylabel('Depths [m]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给纵坐标轴加说明
    % 添加图例
    legend('True depth', 'Observer in [27]', 'Location', 'northeast', 'FontName', 'Times New Roman');

    h = gca; % 获取当前绘图坐标的指针
    % 设定字体大小
    set(h,'FontSize',fontsize, 'FontName', 'Times New Roman'); % 设置文字大小，同时影响坐标轴标注、图例、标题等。
    set(get(gca,'Children'),'linewidth',2);%设置图中线宽1.5磅

    figure(4)
    plot(time, trueDepth-depth09, 'g--');
    % x轴的标签
    xlabel('Time [sec]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给横坐标轴加说明
    % y轴的标签
    ylabel('Estimation errors [m]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给纵坐标轴加说明
    % 添加图例
    legend('Observer in [27]', 'FontName', 'Times New Roman');

    h = gca; % 获取当前绘图坐标的指针
    % 设定字体大小
    set(h,'FontSize',fontsize, 'FontName', 'Times New Roman'); % 设置文字大小，同时影响坐标轴标注、图例、标题等。
    set(get(gca,'Children'),'linewidth',2);%设置图中线宽1.5磅

elseif modelSelection == 4
    % 获取各深度值
    time = allDepths.time;
    lengthData = length(time);
    trueDepth = reshape(allDepths.signals.values(:,1,:), lengthData, 1);
    depth05 = reshape(allDepths.signals.values(:,2,:), lengthData, 1);
    depth08 = reshape(allDepths.signals.values(:,3,:), lengthData, 1);
    depth09 = reshape(allDepths.signals.values(:,4,:), lengthData, 1);
    depth12 = reshape(allDepths.signals.values(:,5,:), lengthData, 1);
    depth20 = reshape(allDepths.signals.values(:,6,:), lengthData, 1);
    
    % 绘制深度图
    figure(1)
    plot(time, trueDepth, 'k');
    hold on
    plot(time, depth08, 'color', [0.72,0.27,1.00]);
    plot(time, depth05, 'color', [0.00,1.00,1.00]);    
    plot(time, depth09, 'g--',time, depth12, 'b-.',time, depth20, 'r:');
    timeSub = 3:0.01:3.4;
    intervals = round(100*timeSub);
    trueDepthSub = trueDepth(intervals);
    depth08Sub = depth08(intervals);
    depth05Sub = depth05(intervals);
    depth09Sub = depth09(intervals);
    depth12Sub = depth12(intervals);    
    depth20Sub = depth20(intervals);    
    
    % x轴的标签
    xlabel('Time [sec]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给横坐标轴加说明
    % y轴的标签
    ylabel('Depths [m]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给纵坐标轴加说明
    % 添加图例
    legend('True depth', 'Observer in [22]', 'Observer in [23]','Observer in [27]', 'Observer in [29]','Proposed observer', 'Location', 'southeast', 'FontName', 'Times New Roman');

    h = gca; % 获取当前绘图坐标的指针
    % 设定字体大小
    set(h,'FontSize',fontsize, 'FontName', 'Times New Roman'); % 设置文字大小，同时影响坐标轴标注、图例、标题等。
    set(get(gca,'Children'),'linewidth',2);%设置图中线宽1.5磅
    ylim([-1,3.5]);
        
    % 绘制子图
    axes('Position', [0.20, 0.67, 0.4, 0.25]);
    plot(timeSub, trueDepthSub, 'k');
    hold on
    plot(timeSub, depth08Sub, 'color', [0.72,0.27,1.00]);
    plot(timeSub, depth05Sub, 'color', [0.00,1.00,1.00]);    
    plot(timeSub, depth09Sub, 'g--',timeSub, depth12Sub, 'b-.',timeSub, depth20Sub, 'r:');
    xlim([min(timeSub), max(timeSub)]);

    h = gca; % 获取当前绘图坐标的指针
    % 设定字体大小
    set(h,'FontSize',fontsize, 'FontName', 'Times New Roman'); % 设置文字大小，同时影响坐标轴标注、图例、标题等。
    set(get(gca,'Children'),'linewidth',2);%设置图中线宽2磅

    figure(2)
    plot(time, trueDepth-depth08, 'color', [0.72,0.27,1.00]);
    hold on;
    plot(time, trueDepth-depth05, 'color', [0.00,1.00,1.00]); 
    plot(time, trueDepth-depth09, 'g--', time, trueDepth-depth12, 'b-.',time, trueDepth-depth20, 'r:'); 
    % x轴的标签
    xlabel('Time [sec]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给横坐标轴加说明
    % y轴的标签
    ylabel('Estimation errors [m]', 'fontsize', fontsize, 'FontName', 'Times New Roman') % 给纵坐标轴加说明
    % 添加图例
    legend('Observer in [22]', 'Observer in [23]','Observer in [27]', 'Observer in [29]','Proposed observer', 'Location', 'north', 'FontName', 'Times New Roman');

    h = gca; % 获取当前绘图坐标的指针
    % 设定字体大小
    set(h,'FontSize',fontsize, 'FontName', 'Times New Roman'); % 设置文字大小，同时影响坐标轴标注、图例、标题等。
    set(get(gca,'Children'),'linewidth',2);%设置图中线宽2磅
    ylim([-2,1.8]);
    
    % 绘制子图
    axes('Position', [0.25, 0.235, 0.4, 0.25]);
    plot(timeSub, trueDepthSub-depth08Sub, 'color', [0.72,0.27,1.00]);
    hold on;
    plot(timeSub, trueDepthSub-depth05Sub, 'color',[0.00,1.00,1.00]);
    plot(timeSub, trueDepthSub-depth09Sub, 'g--', timeSub, trueDepthSub-depth12Sub, 'b-.',timeSub, trueDepthSub-depth20Sub, 'r:');
    xlim([min(timeSub), max(timeSub)]);
    
    h = gca; % 获取当前绘图坐标的指针
    % 设定字体大小
    set(h,'FontSize',fontsize, 'FontName', 'Times New Roman'); % 设置文字大小，同时影响坐标轴标注、图例、标题等。
    set(get(gca,'Children'),'linewidth',2);%设置图中线宽2磅

    % 计算各观测器的均方根误差
    timeDuraTranStat = 0:0.01:0.89;
    transiStat = round(100*timeDuraTranStat+1);
    rmse05TransiStat = sqrt(mean((depth05(transiStat)-trueDepth(transiStat)).^2))
    rmse08TransiStat = sqrt(mean((depth08(transiStat)-trueDepth(transiStat)).^2))  
    rmse09TransiStat = sqrt(mean((depth09(transiStat)-trueDepth(transiStat)).^2))
    rmse12TransiStat = sqrt(mean((depth12(transiStat)-trueDepth(transiStat)).^2))  
    rmse20TransiStat = sqrt(mean((depth20(transiStat)-trueDepth(transiStat)).^2))
    
    timeDuraSteadyStat = 0.9:0.01:4;
    steadyStat = round(100*timeDuraSteadyStat+1);    
    rmse05SteadyStat = sqrt(mean((depth05(steadyStat)-trueDepth(steadyStat)).^2))
    rmse08SteadyStat = sqrt(mean((depth08(steadyStat)-trueDepth(steadyStat)).^2))   
    rmse09SteadyStat = sqrt(mean((depth09(steadyStat)-trueDepth(steadyStat)).^2))
    rmse12SteadyStat = sqrt(mean((depth12(steadyStat)-trueDepth(steadyStat)).^2))
    rmse20SteadyStat = sqrt(mean((depth20(steadyStat)-trueDepth(steadyStat)).^2))
end
