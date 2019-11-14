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
        
        function [] = draw(myrobot,datas)
            %draw Summary of this method goes here
            %   Detailed explanation goes here
            clf;
            set(gcf,'Position',[200 100 1040 1100])

            % Split figure plots
            tiledlayout(12,1)
            
            %-------------------Draw Simulation----------------------------
            nexttile([4 1])
            hold on;
            rectangle('Position',[-10 -1 100 1],'FaceColor',[0.4 0.3 0.1])
            %Draw CoM
            th = 0:pi/50:2*pi;
            x_circle = myrobot.radius * cos(th) + myrobot.x;
            y_circle = myrobot.radius * sin(th) + myrobot.y;
            plot(x_circle, y_circle);
            fill(x_circle, y_circle, 'r')
            %Draw Leg
            leg_pos = [myrobot.x myrobot.y] + myrobot.radius*[cos(myrobot.angle) sin(myrobot.angle)];
            end_point = [leg_pos(1)+myrobot.cur_length*cos(myrobot.angle) leg_pos(2)+myrobot.cur_length*sin(myrobot.angle)];
            plot([leg_pos(1) end_point(1)], [leg_pos(2) end_point(2)],'color','black');
            %Better View
            axis([-1 25 -1 10]);
            
            %-------------------Plot x(m) vs t(s)--------------------------
            nexttile([2 1])
            plot(datas(:,1),datas(:,2));
            axis([0 10 -1 25]);
            
            %-------------------Plot xdot(m/s) vs t(s)--------------------------
            nexttile([2 1])
            plot(datas(:,1),datas(:,3));
            axis([0 10 -3 3]);
            
            %-------------------Plot y(m) vs t(s)--------------------------
            nexttile([2 1])
            plot(datas(:,1),datas(:,4));
            axis([0 10 0 inf]);
            
            %-------------------Plot Q(rad) vs t(s)--------------------------
            nexttile([2 1])
            plot(datas(:,1),datas(:,5));
            axis([0 10 -pi 0]);
            
        end
        
        function myrobot = get_fstate(myrobot,fstate)
            %get_fstate Summary of this method goes here
            %   Detailed explanation goes here
            myrobot.x = fstate.x;
            myrobot.y = fstate.y;
        end
        
        function myrobot = get_sstate(myrobot,sstate,landx,landy)
            %get_sstate Summary of this method goes here
            %   Detailed explanation goes here
            myrobot.x = landx + (sstate.r+myrobot.radius)*cos(sstate.Q);
            myrobot.y = landy + (sstate.r+myrobot.radius)*sin(sstate.Q);
            myrobot.angle = pi+sstate.Q;
            myrobot.cur_length = sstate.r; 
        end
    end
end

