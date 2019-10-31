clear;
clc;
sim_panel = figure("Name","Monopode Simulation",'NumberTitle','off');

leg_length = 1;
angle = pi/6;
drawWorld([0 5],angle,leg_length);
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
        drawWorld([0 x(i,1)], angle, leg_length);
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

function [] = drawWorld(pos,angle,length)
    clf;
    hold on;
    rectangle('Position',[-10 -1 100 1],'FaceColor',[0.4 0.3 0.1])
    drawRobot(pos,angle,length);
    axis square;
    axis([-1 10 -1 10]);
end


function [] = drawRobot(pos,angle,len)
    circle(pos(1),pos(2),0.5,'r');
    angle = (angle + pi/2);
    if (angle > pi); angle=angle-2*pi; end
    leg_pos = pos - 0.5*[cos(angle) sin(angle)];
    end_point = [len*cos(angle) len*sin(angle)];
    if (leg_pos(2) - (len*sin(angle)) < 0); end_point(2) = leg_pos(2); end
    plot([leg_pos(1) leg_pos(1)-end_point(1)], [leg_pos(2) leg_pos(2)-end_point(2)],'color','black');
end

function circles = circle(x,y,r,c)
    th = 0:pi/50:2*pi;
    x_circle = r * cos(th) + x;
    y_circle = r * sin(th) + y;
    circles = plot(x_circle, y_circle);
    fill(x_circle, y_circle, c)
end