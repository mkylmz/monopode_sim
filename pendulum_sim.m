clear;
clc;
sim_panel = figure(1);

leg_length = 3; m=1; b=0.5; k=400;
angle = pi*(60/180);
time_span = 0:0.02:30;

initial_state = [leg_length 0 angle 0];
touch_point = [5 5];

i = 1;
[t,stance_state] = ode45(@(t,state) stance(t,state,leg_length,m,b,k),time_span,initial_state);
count = size(stance_state,1);
while(i <= count)
    xnew = touch_point(1) + (stance_state(i,1)+0.5)*cos(stance_state(i,3));
    ynew = touch_point(2) + (stance_state(i,1)+0.5)*sin(stance_state(i,3));
    drawWorld([xnew ynew], stance_state(i,3), stance_state(i,1));
    pause(0.01);
    i = i + 1;
end

function dstate = stance(t,state,r0,m,b,k)
    dstate = zeros(4,1);
    dstate(1) = state(2);
    dstate(2) = state(1)*(state(4)^2)-9.81*sin(state(3))-(b/m)*state(2)+(k/m)*(r0-state(1));
    dstate(3) = state(4);
    dstate(4) = (-9.81*state(1)*cos(state(3))-2*state(1)*state(2)*state(4))/(state(1)^2);
end

function [] = drawWorld(pos,angle,length)
    clf;
    hold on;
    drawRobot(pos,angle,length);
    axis square;
    axis([-1 10 -1 10]);
end


function [] = drawRobot(pos,angle,len)
    circle(pos(1),pos(2),0.5,'r');
    my_angle = -(pi-angle);
    leg_pos = pos + 0.5*[cos(my_angle) sin(my_angle)];
    end_point = [leg_pos(1)+len*cos(my_angle) leg_pos(2)+len*sin(my_angle)];
    %if (leg_pos(2) - (len*sin(angle)) < 0); end_point(2) = leg_pos(2); end
    plot([leg_pos(1) end_point(1)], [leg_pos(2) end_point(2)],'color','black');
end

function circles = circle(x,y,r,c)
    th = 0:pi/50:2*pi;
    x_circle = r * cos(th) + x;
    y_circle = r * sin(th) + y;
    circles = plot(x_circle, y_circle);
    fill(x_circle, y_circle, c)
end