clear
clc

%% Initialization




%% Test Initial Shape
clear
clc
close all

x = [0 0 5 5 15 15 10 10 5 5 0];
y = [5 20 15 20 20 5 5 0 0 5 5];
x2 = [0.2 0.2 5.2 5.2 14.8 14.8 9.8 9.8 5.2 5.2 0.2];
y2 = [5.2 19.5 14.5 19.8 19.8 5.2 5.2 0.2 0.2 5.2 5.2];
x3 = [0.4 0.4 5.4 5.4 14.6 14.6 9.6 9.6 5.4 5.4 0.4];
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
pstore = [];

for n = 1:10
    if x(n+1)-x(n) == 0
        p = 1;
    end
    p = polyfit([x(n+1) x(n)],[y(n+1) y(n)],1);
    pstore = [pstore ; p]
end

% for k = 0.2:0.2:50
%     infill = -x + k
%     func1 =




zstore = [];
mult = 0;
for i = 0.2:0.2:10
    mult = 1 + mult;
    z = [];
    for j = 1:len
        zcomp = 0.2*mult;
        z = [z zcomp];
    end
zstore = [zstore ; z];
end

% figure
% plot3(x,y,zstore,x2,y2,zstore,x3,y3,zstore)
% xlim([-1 21])
% ylim([-1 21])
% zlim([0 11])

figure
hold on
plot(x,y)
plot(x2,y2)
plot(x3,y3)
%plot(xref,yref)
xlim([-1 21])
ylim([-1 21])


% xvec = min(x2):0.2:max(x2)
% yvec = min(y2):0.2:max(y2)
% figure
% hold on
% for i = 1:73
%     plot(xvec(i+1),yvec(i+1),xvec(i),yvec(i))
% end

vec1 = linspace(0.4,5.4,round(5/0.4));%[0.4:0.4:5.4];
vec2 = linspace(5.4,10.4,round(5/0.4));%[5.4:0.4:10.4];

vec3 = linspace(5.4,9.6,round((9.6-5.4)/0.4));%[5.4:0.4:9.6];
vec4 = linspace(0.4,5.4,round((5.4-0.4)/0.4));%[0.4:0.4:5.4];

% vec5 = [5.4:0.4:9.6];
% vec6 = [0.4:0.4:5.6];
% 
% for i = 1:length(vec1)-1
%     plot([vec1(i+1) vec1(1)],[vec2(1) vec2(i+1)])
% end
% 
% for i = 1:length(vec3)-1
%     plot([vec3(i+1) vec3(1)],[vec4(1) vec4(i+1)])
% end
% 
% for i = 1:length(vec5)-1
%     plot([vec5(i+1) vec5(1)],[vec6(1) vec6(i+1)])
% end
% % plot([0.4 0.2],[5.2 5.4])
% 
% startx = 0.8;
% starty = 5.8;

% vec1 = [0.4:0.4:5.4];
% m = (14-19)/(5.4-0.4)
% b = 40;
% vec2 = m.*vec1 +b;

x1y1 = [0.4 19];
x2y2 = [5.4 14];
v = x2y2 - x1y1
n = sqrt((5.4-0.4)^2+(14-19)^2)
u = v/n

p1 = [0.4 19] + 0.565685*[u(1) u(2)]
plot([x1y1(1) p1(1)],[x1y1(2) p1(2)])

% for i= 1:length(vec1)-1
%     plot([vec1(i+1) vec1(1)],[vec1(1) vec1(i+1)])
% end
