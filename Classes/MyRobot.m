classdef MyRobot
    %MYROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x {mustBeNumeric}
        xdot {mustBeNumeric} 
        y {mustBeNumeric} 
        ydot {mustBeNumeric} 
        radius {mustBeNumeric} 
        angle {mustBeNumeric} 
        leg_length {mustBeNumeric} 
        cur_length {mustBeNumeric} 
        m {mustBeNumeric} %mass
        b {mustBeNumeric} %damping
        k {mustBeNumeric} %spring stiffness
        time {mustBeNumeric}
        step_size {mustBeNumeric}
        CoM_border
        CoM_fill
        Leg_obj
        plot1
        plot2
        plot3
        plot4
    end
    
    methods
        function obj = MyRobot(x_val,xdot_val,y_val,ydot_val,radius_val,leg_val,angle_val,m_val,b_val,k_val,step_size_val)
            %MYROBOT Construct an instance of this class
            %   Detailed explanation goes here
            obj.x = x_val;
            obj.xdot = xdot_val;
            obj.y = y_val;
            obj.ydot = ydot_val;
            obj.radius = radius_val;
            obj.angle = angle_val;
            obj.leg_length = leg_val;
            obj.cur_length = leg_val;
            obj.m = m_val;
            obj.b = b_val;
            obj.k = k_val;
            obj.time = 0;
            obj.step_size = step_size_val;
        end
        
        function myrobot = start(myrobot)
            figure(1);
            set(gcf, 'WindowState', 'maximized');

            
            % Split figure plots
            tiledlayout(6,1)
            %-------------------Draw Simulation----------------------------
            nexttile([2,1])
            hold on;
            rectangle('Position',[-10 -1 100 1],'FaceColor',[0.4 0.3 0.1])
            %Draw CoM
            th = 0:pi/50:2*pi;
            x_circle = myrobot.radius * cos(th) + myrobot.x;
            y_circle = myrobot.radius * sin(th) + myrobot.y;
            myrobot.CoM_border = plot(x_circle, y_circle);
            myrobot.CoM_fill = fill(x_circle, y_circle, 'r');
            %Draw Leg
            leg_pos = [myrobot.x myrobot.y] + myrobot.radius*[cos(myrobot.angle) sin(myrobot.angle)];
            end_point = [leg_pos(1)+myrobot.cur_length*cos(myrobot.angle) leg_pos(2)+myrobot.cur_length*sin(myrobot.angle)];
            myrobot.Leg_obj = plot([leg_pos(1) end_point(1)], [leg_pos(2) end_point(2)],'color','black');
            %Better View
            axis([-1 50 -1 10]);
            
            %-------------------Plot x(m) vs t(s)--------------------------
            nexttile(3)
            hold on;
            myrobot.plot1 = plot(myrobot.time,myrobot.x,'-k.', 'MarkerSize',2);
            axis([0 20 -1 50]);
            
            %-------------------Plot xdot(m/s) vs t(s)--------------------------
            nexttile(4)
            hold on;
            myrobot.plot2 = plot(myrobot.time,myrobot.xdot,'-k.', 'MarkerSize',2);
            axis([0 20 -3 inf]);
            
            %-------------------Plot y(m) vs t(s)--------------------------
            nexttile(5)
            hold on;
            myrobot.plot3 = plot(myrobot.time,myrobot.y,'-k.', 'MarkerSize',2);
            axis([0 20 0 inf]);
            
            %-------------------Plot Q(rad) vs t(s)--------------------------
            nexttile(6)
            hold on;
            myrobot.plot4 = plot(myrobot.time,myrobot.angle,'-k.', 'MarkerSize',2);
            axis([0 20 -pi 0]);
            
        end
        
        function myrobot = draw(myrobot)
            %draw Summary of this method goes here
            %   Detailed explanation goes here

            myrobot.time = myrobot.time + myrobot.step_size;
            
            %Draw CoM
            hold on;
            nexttile(1)
            delete(myrobot.CoM_border);
            delete(myrobot.CoM_fill);
            delete(myrobot.Leg_obj);
            th = 0:pi/50:2*pi;
            x_circle = myrobot.radius * cos(th) + myrobot.x;
            y_circle = myrobot.radius * sin(th) + myrobot.y;
            myrobot.CoM_border = plot(x_circle, y_circle);
            myrobot.CoM_fill = fill(x_circle, y_circle, 'r');
            %Draw Leg
            leg_pos = [myrobot.x myrobot.y] + myrobot.radius*[cos(myrobot.angle) sin(myrobot.angle)];
            end_point = [leg_pos(1)+myrobot.cur_length*cos(myrobot.angle) leg_pos(2)+myrobot.cur_length*sin(myrobot.angle)];
            myrobot.Leg_obj = plot([leg_pos(1) end_point(1)], [leg_pos(2) end_point(2)],'color','black');
            %Better View
            %axis([-1 50 -1 10]);
            
            %-------------------Plot x(m) vs t(s)--------------------------
            nexttile(3)
            hold on;
            title("x vs t");
            set(myrobot.plot1,'XData',[get(myrobot.plot1,'XData'),myrobot.time],'YData',[get(myrobot.plot1,'YData'),myrobot.x])
            %axis([0 10 -1 25]);
            
            %-------------------Plot xdot(m/s) vs t(s)--------------------------
            nexttile(4)
            hold on;
            title("x' vs t");
            set(myrobot.plot2,'XData',[get(myrobot.plot2,'XData'),myrobot.time],'YData',[get(myrobot.plot2,'YData'),myrobot.xdot])
            %axis([0 10 -3 3]);
            
            %-------------------Plot y(m) vs t(s)--------------------------
            nexttile(5)
            hold on;
            title("y vs t");
            set(myrobot.plot3,'XData',[get(myrobot.plot3,'XData'),myrobot.time],'YData',[get(myrobot.plot3,'YData'),myrobot.y])
            %axis([0 10 0 inf]);
            
            %-------------------Plot Q(rad) vs t(s)--------------------------
            nexttile(6)
            hold on;
            title("Q vs t");
            set(myrobot.plot4,'XData',[get(myrobot.plot4,'XData'),myrobot.time],'YData',[get(myrobot.plot4,'YData'),myrobot.angle])
            %axis([0 10 -pi 0]);
            
        end
        
        function myrobot = get_fstate(myrobot,fstate)
            %get_fstate Summary of this method goes here
            %   Detailed explanation goes here
            myrobot.x = fstate.x;
            myrobot.xdot = fstate.xdot;
            myrobot.y = fstate.y;
            myrobot.ydot = fstate.ydot;
        end
        
        function myrobot = get_sstate(myrobot,sstate,landx,landy)
            %get_sstate Summary of this method goes here
            %   Detailed explanation goes here
            myrobot.x = landx + (sstate.r+myrobot.radius)*cos(sstate.Q);
            myrobot.xdot = sstate.rdot*cos(sstate.Q) - sstate.r*sstate.Qdot*sin(sstate.Q);
            myrobot.y = landy + (sstate.r+myrobot.radius)*sin(sstate.Q);
            myrobot.ydot = sstate.rdot*sin(sstate.Q) + sstate.r*sstate.Qdot*cos(sstate.Q);
            myrobot.angle = -pi+sstate.Q;
            myrobot.cur_length = sstate.r; 
        end
    end
end

