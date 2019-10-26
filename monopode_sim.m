clear;
clc;
sim_panel = figure("Name","Monopode Simulation",'NumberTitle','off');

setup_environment([0 5]);

init_pos = 5;
init_vel = 0;
time_span = 0:0.01:10;
opt   = odeset('Events', @LandingEvent);
[t,x] = ode45(@air_func,time_span,[init_pos init_vel],opt);

i = 1;
while(x(i) > 1.5)
    setup_environment([0 x(i)]);
    pause(0.01)
    i = i + 1;
end

function dx = air_func(t,x)
    dx = zeros(2,1);
    dx(1) = x(2);
    dx(2) = -9.81 - ( 1 * x(2) )/100;
end

function [value, isterminal, direction] = LandingEvent(t, x)
value      = (x(1) < 1.5);
isterminal = 1;
direction  = 0;
end

function [] = setup_environment(init_pos)
    clf;
    hold on;
    drawGround();
    drawRobot(init_pos,1);
    axis square;
    axis([-1 10 -1 10]);
end

function [] = drawGround()
    rectangle('Position',[-10 -1 100 1],'FaceColor',[0.4 0.3 0.1])
end

function [] = drawRobot(pos,length)
    circle(pos(1),pos(2),0.5,'r');
    leg_pos = pos + [0 -0.5];
    L1 = length/3; L2 = 2*L1;
    plot([leg_pos(1) leg_pos(1)], [leg_pos(2) leg_pos(2)-L1]);    
    plot((leg_pos(1) + sin(linspace(0,8*pi))/12 ) , linspace(leg_pos(2)-L1,leg_pos(2)-L2))
    plot([leg_pos(1) leg_pos(1)], [leg_pos(2)-L2 leg_pos(2)-L2-L1]); 
end

function circles = circle(x,y,r,c)
    th = 0:pi/50:2*pi;
    x_circle = r * cos(th) + x;
    y_circle = r * sin(th) + y;
    circles = plot(x_circle, y_circle);
    fill(x_circle, y_circle, c)
end