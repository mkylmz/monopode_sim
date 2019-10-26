clear;
clc;
sim_panel = figure("Name","Monopode Simulation");

init_pos = 1;
init_vel = 0;
setup_environment(init_pos);
time_span = 0:0.001:10;

[t,x] = ode45(@equa,time_span,[init_pos init_vel]);

i = 1;
while(i < 10000)
    setup_environment(x(i));
    pause(0.001)
    i = i + 1;
end

function dx = equa(t,x)
    dx = zeros(2,1);
    dx(1) = x(2);
    dx(2) = ( -40*x(1) - 0.1*x(2) )/10;
end

function [] = setup_environment(init_pos)
    clf;
    hold on;
    drawWall();
    drawSpring(init_pos);
    drawObj(init_pos);
    axis square;
    axis([-1 10 -1 10]);
end


function [] = drawWall()
    plot([0 0 10],[10 0 0]);
end

function [] = drawSpring(length)
    t1 = (length+5)/3; t2 = 2*t1;
    plot([0 t1],[1 1]);    
    plot( linspace(t1,t2) , (1 + sin(linspace(0,12*pi))/3 ))
    plot([t2 length+5],[1 1]); 
end


function [] = drawObj(length)
    X = (length+5) + [0 0 2 2 0];
    Y = [2 0 0 2 2];
    plot(X, Y);
end