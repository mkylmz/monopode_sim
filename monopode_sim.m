clear;
clc;
sim_panel = figure(1);

leg_length = 1; m=1; b=0.5; k=400;
angle = pi*(90/180);
drawWorld([0 5],angle,leg_length);
mode = 0;
time_span = 0:0.02:10;

trans_state = ones(1,4);
trans_state(1,1) = 5;
trans_state(1,2) = 0;
trans_state(1,3) = 0;
trans_state(1,4) = 0;
count = 1;


while (1)
    i = 1;
    if (mode == 0)
        opt   = odeset('Events', @(t,y) LandingEvent(t,y,angle,leg_length));
        [t,flight_state] = ode45(@flight_func,time_span,trans_state,opt);
        count = size(flight_state,1);
        while(i <= count)
            drawWorld([flight_state(i,3) flight_state(i,1)], angle, leg_length);
            pause(0.01)
            i = i + 1;
        end
        rdot = flight_state(count,2)*sin(angle) + flight_state(count,4)*cos(angle);
        qdot = flight_state(count,2)*cos(angle) + flight_state(count,4)*sin(angle);
        landy = flight_state(count,1)-(leg_length+0.5)*sin(angle);
        landx = flight_state(count,3)-(leg_length+0.5)*cos(angle);
        trans_state = [leg_length rdot angle qdot];
        mode = 1;
    elseif (mode == 1)
        opt   = odeset('Events', @(t,y) TakeoffEvent(t,y,leg_length));
        [t,stance_state] = ode45(@(t,state) stance(t,state,leg_length,m,b,k),time_span,trans_state,opt);
        count = size(stance_state,1);
        while(i <= count)
            xnew = landx + (stance_state(i,1)+0.5)*cos(stance_state(i,3));
            ynew = landy + (stance_state(i,1)+0.5)*sin(stance_state(i,3));
            drawWorld([xnew ynew], stance_state(i,3), stance_state(i,1));
            pause(0.01)
            i = i + 1;
        end
        trans_state = zeros(1,4);
        trans_state(1) = ynew;
        trans_state(2) = stance_state(count,2)*cos(angle) + stance_state(count,4)*sin(angle);
        trans_state(3) = xnew;
        trans_state(4) = stance_state(count,2)*sin(angle) - stance_state(count,4)*cos(angle);
        mode = 0;
    end
end

function dpos = flight_func(t,pos)
    dpos = zeros(2,1);
    dpos(1) = pos(2);
    dpos(2) = -9.81;
    dpos(3) = pos(4);
    dpos(4) = 0;
end

function [value, isterminal, direction] = LandingEvent(t, y, angle,leg_length)
value      = (y(1) < (leg_length+0.5)*sin(angle));
isterminal = 1;
direction  = 0;
end

function dstate = stance(t,state,r0,m,b,k)
    dstate = zeros(4,1);
    dstate(1) = state(2);
    dstate(2) = state(1)*(state(4)^2)-9.81*sin(state(3))-(b/m)*state(2)+(k/m)*(r0-state(1));
    dstate(3) = state(4);
    dstate(4) = (-9.81*state(1)*cos(state(3))-2*state(1)*state(2)*state(4))/(state(1)^2);
end

function [value, isterminal, direction] = TakeoffEvent(t, y, leg_length)
value      = (y(1) > leg_length);
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
    my_angle = -(pi-angle);
    leg_pos = pos + 0.5*[cos(my_angle) sin(my_angle)];
    end_point = [leg_pos(1)+len*cos(my_angle) leg_pos(2)+len*sin(my_angle)];
    plot([leg_pos(1) end_point(1)], [leg_pos(2) end_point(2)],'color','black');
end

function circles = circle(x,y,r,c)
    th = 0:pi/50:2*pi;
    x_circle = r * cos(th) + x;
    y_circle = r * sin(th) + y;
    circles = plot(x_circle, y_circle);
    fill(x_circle, y_circle, c)
end