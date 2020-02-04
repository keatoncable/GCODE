clear
clc

%% Initialization




%% Test Initial Shape
close all

x = [0 0 5 5 15 15 10 10 5 5 0];
y = [5 20 15 20 20 5 5 0 0 5 5];
x2 = x-0.2;
y2 = y-0.2;

figure
hold on
plot(x,y)
plot(x2,y2)
xlim([-1 22])
ylim([-1 22])



