clear;
clc;
sim_panel = figure("Name","Monopode Simulation");

spring_length = 5;
setup_environment(spring_length);








function [] = setup_environment(spring_length)

    clf;
    hold on;
    drawWall();
    drawSpring(spring_length);
    drawObj(spring_length);
    
    axis square;
    axis([-1 10 -1 10]);

end



function [] = drawWall()

    plot([0 0 10],[10 0 0]);

end

function [] = drawSpring(length)

    t1 = length/3; t2 = 2*t1;
    plot([0 t1],[1 1]);    
    plot( linspace(t1,t2) , (1 + sin(linspace(0,12*pi))/3 ))
    plot([t2 length],[1 1]); 

end


function [] = drawObj(length)

    X = length + [0 0 2 2 0];
    Y = [2 0 0 2 2];
    plot(X, Y);


end