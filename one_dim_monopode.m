clear;
clc;
sim_panel = figure("Name","Monopode Simulation",'NumberTitle','off');

setup_environment([0 5]);
mode = 0;
time_span = 0:0.01:10;

x = ones(1,2);
x(1,1) = 5;
x(1,2) = 0;
count = 1;

while (1)
    i = 1;
    pos = x(count,1);
    vel = x(count,2);
    if (mode == 0)
        opt   = odeset('Events', @LandingEvent);
        [t,x] = ode45(@air_func,time_span,[pos vel],opt);
        mode = 1;
    elseif (mode == 1)
        opt   = odeset('Events', @TakeoffEvent);
        [t,x] = ode45(@(t,x) ground_func(t,x,x0),time_span,[pos vel],opt);
        mode = 0;
    end
    count = size(x,1);
    x0 = x(count,1);
    while(i <= count)
        setup_environment([0 x(i,1)]);
        pause(0.01)
        i = i + 1;
    end
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

function dx = ground_func(t,x,x0)
    dx = zeros(2,1);
    dx(1) = x(2);
    dx(2) = - 9.81 - 200*(x(1) - x0) - 1 * x(2);
end

function [value, isterminal, direction] = TakeoffEvent(t, x)
value      = (x(1) > 1.51);
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
    if (leg_pos(2) - length < 0)
       length =  leg_pos(2);
    end
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