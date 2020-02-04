clear
clc

%% Initialization




%% Test Initial Shape
close all
v1 = [0 5 ; 0 25]
v2 = [v1(2,:) ; 5 20]
v3 = [v2(2,:) ; 5 25]
v4 = [v3(2,:) ; 15 25]
v5 = [v4(2,:) ; 15 20]
v6 = [v5(2,:) ; 10 5]
v7 = [v6(2,:) ; 10 0]
v8 = [v7(2,:) ; 5 0]
v9 = [v8(2,:) ; 5 5]
v10 = [v9(2,:) ; v1(1,:)]
x = [0 0 5 5 15 15 10 10 5 5 0];
y = [5 20 15 20 20 5 5 0 0 5 5];

figure
hold on
plot(x,y)
xlim([-1 22])
ylim([-1 22])
% plot(v1(2,1),v1(2,2))
% plot(v2)
% plot(v3)
% plot(v4)
% plot(v5)
% plot(v6)
% plot(v7)
% plot(v8)
% plot(v9)
% plot(v10)



