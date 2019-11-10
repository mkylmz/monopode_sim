clear;
clc;

xinit = 0; yinit = 5; radius = 0.5;
angle = -92*(pi/180);leg_length = 1; m=4; b=1; k=800;
myrobot = MyRobot(xinit,yinit,radius,leg_length,angle,m,b,k);
myrobot.draw();

mode = 0;
time_span = 0:0.01:10;

tstate = FlightState(xinit,0,yinit,0);
count = 1;


while (1)
    i = 1;
    if (myrobot.angle > pi); myrobot.angle=myrobot.angle-2*pi; end
    if (myrobot.angle < -pi); myrobot.angle=2*pi+myrobot.angle; end
    if (mode == 0)
        opt   = odeset('Events', @(t,y) LandingEvent(t,y,myrobot,angle));
        [t,states] = ode45(@flight_func,time_span,[tstate.x tstate.xdot tstate.y tstate.ydot],opt);
        count = size(states,1);
        fstates = fstate_converter(states,count);
        for i =  1:count
            myrobot = myrobot.get_fstate(fstates(i));
            myrobot.angle = myrobot.angle + i*(angle-myrobot.angle)/count;
            myrobot.draw();
            pause(0.01)
        end
        tstate = fstates(i).calcPolarCoordinates(myrobot);
        mode = 1;
    elseif (mode == 1)
        opt   = odeset('Events', @(t,y) TakeoffEvent(t,y,myrobot));
        [t,states] = ode45(@(t,state) stance(t,state,myrobot),time_span,tstate(1:4),opt);
        count = size(states,1);
        sstates = sstate_converter(states,count);
        for i =  1:count
            myrobot = get_sstate(myrobot,sstates(i),tstate(5),tstate(6));
            myrobot.draw();
            pause(0.01);
        end
        tstate = calcCartesianCoordinates(sstates(i),myrobot,tstate(5),tstate(6));
        mode = 0;
    end
end

function dpos = flight_func(t,state)
    dpos = zeros(2,1);
    dpos(1) = state(2);
    dpos(2) = 0;
    dpos(3) = state(4);
    dpos(4) = -9.81;
end

function [value, isterminal, direction] = LandingEvent(t, state, myrobot,angle)
    value      = (state(3) < (myrobot.leg_length+myrobot.radius)*-sin(angle)) && state(4) < 0;
    isterminal = 1;
    direction  = 0;
end

function dstate = stance(t,state,myrobot)
    dstate = zeros(4,1);
    dstate(1) = state(2);
    dstate(2) = state(1)*(state(4)^2)-9.81*sin(state(3))-(myrobot.b/myrobot.m)*state(2)+(myrobot.k/myrobot.m)*(myrobot.leg_length-state(1));
    dstate(3) = state(4);
    dstate(4) = (-9.81*state(1)*cos(state(3))-2*state(1)*state(2)*state(4))/(state(1)^2);
end

function [value, isterminal, direction] = TakeoffEvent(t, state, myrobot)
    value      = (state(1) > myrobot.leg_length);
    isterminal = 1;
    direction  = 0;
end

function fstates = fstate_converter(arr,count)
    fstates(1:count) = FlightState(0,0,0,0);
    for i = 1:count
        fstates(i).x = arr(i,1);
        fstates(i).xdot = arr(i,2);
        fstates(i).y = arr(i,3);
        fstates(i).ydot = arr(i,4);
    end
end

function sstates = sstate_converter(arr,count)
    sstates(1:count) = StanceState(0,0,0,0);
    for i = 1:count
        sstates(i).r = arr(i,1);
        sstates(i).rdot = arr(i,2);
        sstates(i).Q = arr(i,3);
        sstates(i).Qdot = arr(i,4);
    end
end