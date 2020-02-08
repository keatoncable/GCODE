clear
clc

%% Initialization




%% Test Initial Shape
clear
clc
close all

noz = 0.4;

x = [0.2 0.2 5.2 5.2 14.8 14.8 9.8 9.8 5.2 5.2 0.2];
y = [5.2 19.8 14.8 19.8 19.8 5.2 5.2 0.2 0.2 5.2 5.2];

xdiff = max(x)-min(x);
ydiff = max(y)-min(y);

x2 = [];
y2 = [];
x3 = [];
y3 = [];

zstore = [];
mult = 0;
for i = 0.2:0.2:10
    mult = 1 + mult;
    z = [];
    for j = 1:length(0.2:0.2:10)
        zcomp = 0.2*mult;
        z = [z zcomp];
    end
zstore = [zstore ; z];
end


for i = 1:length(x)
   xref = x(i);
   yref = y(i);
   enter = 1;
   if xref == 0.2 && yref == 19.8 || xref == 5.2 && yref == 14.8
       x2 = [x2 xref+noz];
       y2 = [y2 yref-0.8];
       enter = 0;
   elseif xref <= xdiff/2
       x2 = [x2 xref+noz];
   else
       x2 = [x2 xref-noz];
   end
   if enter == 1
        if yref <= ydiff/2
            y2 = [y2 yref+noz];
        else
            y2 = [y2 yref-noz];
        end
   end
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
%plot(x3,y3)
%plot(xref,yref)
xlim([-1 21])
ylim([-1 21])

% FIRST BOTTOM LAYER

x1y1 = [0.6 19-0.2];
x2y2 = [5.6 14-0.2];
v = x2y2 - x1y1
n = sqrt((x1y1(1)-x2y2(1))^2+((x1y1(2)-x2y2(2))^2))
u = v/n
pstore = [];

infx1 = [];
infy1 = [];

numlines0 = round((x2y2(1)-x1y1(1))/0.4);
infline1 = round(numlines0*0.3);
dist = (x2y2(1)-x1y1(1))/numlines0/sind(45);
layer = 0.2;

for i = 1:numlines0
    p1 = [x1y1(1) x1y1(2)] + dist*i*[u(1) u(2)]
    pstore = [pstore ; p1];
    if i == 1
       %plot([x1y1(1) p1(1)],[x1y1(2) p1(2)])
    else
       %plot([pstore(i-1,1) p1(1)],[pstore(i-1,2) p1(2)]) 
    end
end

line = x2y2(1)+0.2;

for i = 1:numlines0
     plot([pstore(i,1) pstore(i,1)],[line pstore(i,2)])
     
end

x1 = 6;
x2 = 9.4;
y1 = 0.8;
y2 = 19.2;
 numlines = (9.6-6)/0.4;
 dist2 = (x2-x1)/round(numlines);
 xsto1 = x1:dist2:x2;
 
 for i = 1:round(numlines)
     plot([xsto1(i) xsto1(i)],[y1 y2])
 end
 
x12 = 9.4;
x22 = 14.4;
y12 = 5.8;
y22 = 19.2;
 numlines2 = (x22-x12)/0.4;
 dist3 = (x22-x12)/round(numlines2);
 xsto2 = x12:dist3:x22;
 
 for i = 1:round(numlines2)
     plot([xsto2(i) xsto2(i)],[y12 y22])
 end
 
 %% SECOND BOTTOM LAYER
 
 
x12 = 5.8;
x22 = 9.2;
y12 = 1;
y22 = 6;
 numlines2 = (y22-y12)/0.4;
 dist3 = (y22-y12)/round(numlines2);
 ysto2 = y12:dist3:y22;
 
 for i = 1:round(numlines2)
     plot([x12 x22],[ysto2(i) ysto2(i)])
 end
 
x12 = 0.8;
x22 = 14.2;
y12 = 6;
y22 = 14;
 numlines2 = (y22-y12)/0.4;
 dist3 = (y22-y12)/round(numlines2);
 ysto2 = y12:dist3:y22;
 
 for i = 1:round(numlines2)
     plot([x12 x22],[ysto2(i) ysto2(i)])
 end
 
x12 = 5.8;
x22 = 14.2;
y12 = 14;
y22 = 19.4;
 numlines2 = (y22-y12)/0.4;
 dist3 = (y22-y12)/round(numlines2);
 ysto2 = y12:dist3:y22;
 
 for i = 1:round(numlines2)
     plot([x12 x22],[ysto2(i) ysto2(i)])
 end
 
x1y1 = [0.6 19-0.2];
x2y2 = [5.6 14-0.2];
v = x2y2 - x1y1
n = sqrt((x1y1(1)-x2y2(1))^2+((x1y1(2)-x2y2(2))^2))
u = v/n
pstore = [];

numlines0 = abs(round((x2y2(2)-x1y1(2))/0.4));
dist = (x2y2(1)-x1y1(1))/numlines0/sind(45);
layer = 0.2;

xn1 = [1 14]

for i = 1:numlines0
    p1 = [x1y1(1) x1y1(2)+0.2] + dist*i*[u(1) u(2)]
    pstore = [pstore ; p1];
    if i == 1
       %plot([x1y1(1) p1(1)],[x1y1(2) p1(2)])
    else
       %plot([pstore(i-1,1) p1(1)],[pstore(i-1,2) p1(2)]) 
    end
end

line = x1y1(1)+0.2;

for i = 1:numlines0
     plot([line pstore(i,1)-0.2],[pstore(i,2) pstore(i,2)])
end

%% Infill

for i = 1:length(x)
   xref = x(i);
   yref = y(i);
   enter = 1;
   if xref == 0.2 && yref == 19.8 || xref == 5.2 && yref == 14.8
       x2 = [x2 xref+noz];
       y2 = [y2 yref-0.8];
       enter = 0;
   elseif xref <= xdiff/2
       x2 = [x2 xref+noz];
   else
       x2 = [x2 xref-noz];
   end
   if enter == 1
        if yref <= ydiff/2
            y2 = [y2 yref+noz];
        else
            y2 = [y2 yref-noz];
        end
   end
end

figure
hold on
plot(x,y)
plot(x2,y2)
%plot(x3,y3)
%plot(xref,yref)
xlim([-1 21])
ylim([-1 21])

tot = (9.022-6)/4;
infill = tot*.3




