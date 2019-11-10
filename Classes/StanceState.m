classdef StanceState
    %STANCESTATE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        r {mustBeNumeric} 
        rdot {mustBeNumeric} 
        Q {mustBeNumeric} 
        Qdot {mustBeNumeric} 
    end
    
    methods
        function obj = StanceState(r_val,rdot_val,Q_val,Qdot_val)
            %STANCESTATE Construct an instance of this class
            %   Detailed explanation goes here
            obj.r = r_val;
            obj.rdot = rdot_val;
            obj.Q = Q_val;
            obj.Qdot = Qdot_val;
        end
        
        function tstate = calcCartesianCoordinates(sstate,myrobot,landx,landy)
            %calcCartesianCoordinates Summary of this method goes here
            %   Detailed explanation goes here
            x = landx + (sstate.r+myrobot.radius)*cos(sstate.Q);
            xdot = sstate.rdot*cos(sstate.Q) + sstate.Qdot*sin(sstate.Q);
            y = landy + (sstate.r+myrobot.radius)*sin(sstate.Q);
            ydot = sstate.rdot*sin(sstate.Q) - sstate.Qdot*cos(sstate.Q);
            tstate = FlightState(x, xdot, y, ydot);
        end
    end
end

