classdef FlightState
    %FLIGHTSTATE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x {mustBeNumeric} 
        xdot {mustBeNumeric}
        y {mustBeNumeric}
        ydot {mustBeNumeric}
    end
    
    methods
        function obj = FlightState(x_val,xdot_val,y_val,ydot_val)
            %FLIGHTSTATE Construct an instance of this class
            %   Detailed explanation goes here
            obj.x = x_val;
            obj.xdot = xdot_val;
            obj.y = y_val;
            obj.ydot = ydot_val;
        end
        
        function tstate = calcPolarCoordinates(fstate,myrobot)
            %calcPolarCoordinates Summary of this method goes here
            %   Detailed explanation goes here
            rdot = - fstate.xdot*cos(myrobot.angle) - fstate.ydot*sin(myrobot.angle);
            qdot = - fstate.ydot*cos(myrobot.angle) + fstate.xdot*sin(myrobot.angle);
            landx = fstate.x+(myrobot.leg_length+myrobot.radius)*cos(myrobot.angle);
            landy = fstate.y+(myrobot.leg_length+myrobot.radius)*sin(myrobot.angle);
            tstate = [myrobot.leg_length rdot pi+myrobot.angle qdot landx landy];
        end
    end
end

