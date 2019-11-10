classdef MyRobot
    %MYROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x {mustBeNumeric} 
        y {mustBeNumeric} 
        radius {mustBeNumeric} 
        angle {mustBeNumeric} 
        leg_length {mustBeNumeric} 
        cur_length {mustBeNumeric} 
        m {mustBeNumeric} %mass
        b {mustBeNumeric} %damping
        k {mustBeNumeric} %spring stiffness
        sim_panel
    end
    
    methods
        function obj = MyRobot(x_val,y_val,radius_val,leg_val,angle_val,m_val,b_val,k_val)
            %MYROBOT Construct an instance of this class
            %   Detailed explanation goes here
            obj.x = x_val;
            obj.y = y_val;
            obj.radius = radius_val;
            obj.angle = angle_val;
            obj.leg_length = leg_val;
            obj.cur_length = leg_val;
            obj.m = m_val;
            obj.b = b_val;
            obj.k = k_val;
            obj.sim_panel = figure(1);
        end
        
        function [] = draw(myrobot)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            clf;
            hold on;
            rectangle('Position',[-10 -1 100 1],'FaceColor',[0.4 0.3 0.1])
            %Draw CoM
            th = 0:pi/50:2*pi;
            x_circle = myrobot.radius * cos(th) + myrobot.x;
            y_circle = myrobot.radius * sin(th) + myrobot.y;
            plot(x_circle, y_circle);
            fill(x_circle, y_circle, 'r')
            %Draw Leg
            my_angle = -(pi-myrobot.angle);
            leg_pos = [myrobot.x myrobot.y] + myrobot.radius*[cos(my_angle) sin(my_angle)];
            end_point = [leg_pos(1)+myrobot.cur_length*cos(my_angle) leg_pos(2)+myrobot.cur_length*sin(my_angle)];
            plot([leg_pos(1) end_point(1)], [leg_pos(2) end_point(2)],'color','black');
            %Better View
            axis square;
            axis([-1 10 -1 10]);
        end
        
        function myrobot = get_fstate(myrobot,fstate)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            myrobot.x = fstate.x;
            myrobot.y = fstate.y;
        end
        
        function myrobot = get_sstate(myrobot,sstate,landx,landy)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            myrobot.x = landx + (sstate.r+myrobot.radius)*cos(sstate.Q);
            myrobot.y = landy + (sstate.r+myrobot.radius)*sin(sstate.Q);
            myrobot.angle = sstate.Q;
            myrobot.cur_length = sstate.r; 
        end
    end
end

