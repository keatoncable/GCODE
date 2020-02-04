clear
clc

%% Initialization




%% Test Initial Shape
close all

x = [0 0 5 5 15 15 10 10 5 5 0];
y = [5 20 15 20 20 5 5 0 0 5 5];
x2 = [0.2 0.2 5.2 5.2 14.8 14.8 9.8 9.8 5.2 5.2 0.2];
y2 = [5.2 19.5 14.5 19.8 19.8 5.2 5.2 0.2 0.2 5.2 5.2];
x3 = [0.4 0.4 5.4 5.4 14.6 14.6 9.6 9.6 5.4 5.4 0.2];
y3 = [5.4 19 14 19.6 19.6 5.4 5.4 0.4 0.4 5.4 5.4];
len = length(x);
x2store = [];
y2store = [];

% for i = 1:len
%     xcomp = x(i);
%     ycomp = y(i);
%     nozzle = 0.2;
% 
%     if xcomp - nozzle < min(x)
%         x2comp = xcomp + nozzle;
%     elseif xcomp + nozzle > max(x)
%         x2comp = xcomp - nozzle;
%     else
%         x2comp = xcomp + nozzle;
%     end
%     
%     if ycomp - nozzle < min(y)
%         y2comp = ycomp + nozzle;
%     elseif ycomp + nozzle > max(y)
%         y2comp = ycomp - nozzle;
%     else
%         y2comp = ycomp + nozzle;
%     end
% x2store = [x2store x2comp];
% y2store = [y2store y2comp];
% end

figure
hold on
plot(x,y)
plot(x2,y2)
plot(x3,y3)
xlim([-1 22])
ylim([-1 22])

z = [];
zstore = [];
mult = 0
for i = 0.2:0.2:10
    mult = 1 + mult;
    z = [];
    for j = 1:len
        zcomp = 0.2*mult;
        z = [z zcomp]
    end
zstore = [zstore ; z]
end

figure
plot3(x,y,zstore,x2,y2,zstore,x3,y3,zstore)
xlim([-1 22])
ylim([-1 22])
zlim([0 11])



