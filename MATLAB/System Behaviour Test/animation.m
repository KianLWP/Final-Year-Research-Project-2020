function animation(y)
%Constants
L = 0.15; %COM distt
r  = 0.04;  %Defining the wheel radius
%Defining state space variables
x  = y(1);
th = y(3);


%Plot general Plot from -10 to 10 in x-axis and -1 to 1 in y-axis
plot([-1 1],[0 0],'w','LineWidth',2)
hold on

%Drawing pendulum
y = r;      %height of wheel is the radius of the wheel
rectangle('Position',[w1x-r,0,r*2,r*2],'Curvature',1,'FaceColor','r','EdgeColor','b') %Draw wheel

Pd = 0.025;    %Defining the Diameter of the pendulum
Px = x + L*sin(th);  %Defining x-position of the pendulum center
Py = y + L*cos(th);  %Defining y-position of the pendulum center

rectangle('Position',[Px-Pd/2,Py-Pd/2,Pd,Pd],'Curvature',[1,1],'FaceColor','b','EdgeColor','r') %Draw COM
plot([x Px],[y Py],'w','LineWidth',2) %Draw line between wheel and COM

%Setting plot and Figures colors
set(gca,'Color','k','XColor','w','YColor','w')  %axis colours white
set(gcf,'Color','k')                            %background black
xlim([-0.3 0.3]);                               
ylim([-0.3 0.3]);
drawnow                                         
hold off