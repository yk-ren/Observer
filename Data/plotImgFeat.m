clc
clear
close all

load imgFeat.txt


%% plot the figure
number = 1317;
t = 0:1/30:number/30;

massCenter0x = imgFeat(334:number + 334,1);
massCenter0y = imgFeat(334:number + 334,2);    
massCenter1x = imgFeat(334:number + 334,3);
massCenter1y = imgFeat(334:number + 334,4);
massCenter2x = imgFeat(334:number + 334,5);
massCenter2y = imgFeat(334:number + 334,6);    
massCenter3x = imgFeat(334:number + 334,7);
massCenter3y = imgFeat(334:number + 334,8);

%%
figure(1)
subplot(211)
plot(t, massCenter0x,'r','LineWidth',1.5);
xlabel('Time (sec)');
ylabel('\itx \rmcoordinate (pixels)');
subplot(212)
plot(t, massCenter0y,'b','LineWidth',1.5);
xlabel('Time (sec)');
ylabel('\ity \rmcoordinate (pixels)');

figure(2)
subplot(211)
plot(t, massCenter1x,'r','LineWidth',1.5);
xlabel('Time (sec)');
ylabel('\itx \rmcoordinate (pixels)');
subplot(212)
plot(t, massCenter1y,'b','LineWidth',1.5);
xlabel('Time (sec)');
ylabel('\ity \rmcoordinate (pixels)');

figure(3)
subplot(211)
plot(t, massCenter2x,'r','LineWidth',1.5);
xlabel('Time (sec)');
ylabel('\itx \rmcoordinate (pixels)');
subplot(212)
plot(t, massCenter2y,'b','LineWidth',1.5);
xlabel('Time (sec)');
ylabel('\ity \rmcoordinate (pixels)');

figure(4)
subplot(211)
plot(t, massCenter3x,'r','LineWidth',1.5);
xlabel('Time (sec)');
ylabel('\itx \rmcoordinate (pixels)');
subplot(212)
plot(t, massCenter3y,'b','LineWidth',1.5);
xlabel('Time (sec)');
ylabel('\ity \rmcoordinate (pixels)');
