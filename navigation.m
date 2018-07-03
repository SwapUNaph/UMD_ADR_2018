clear 
close all
clc

%rosinit
posesub = rossubscriber('/bebop/odom');

l=1;
dx0 = [l,0,0];
dy0 = [0,l,0];
dz0 = [0,0,l];

h1 = axes;

while 1
    posedata = receive(posesub, 5);
    
    pos = posedata.Pose.Pose.Position;
    ori = posedata.Pose.Pose.Orientation;
    %posedata.Twist.Twist.Linear
    %posedata.Twist.Twist.Angular

    p = [pos.X pos.Y pos.Z];

    %plot3(p(1),p(2),p(3),'bx')
    q = [ori.W ori.X ori.Y ori.Z];
    qi = quatinv(q)
    
    dx = quatrotate(qi,dx0)
    dy = quatrotate(qi,dy0)
    dz = quatrotate(qi,dz0)
    dxt = p+dx;
    dyt = p+dy;
    dzt = p+dz;
    
    heading=atan2(dx(2),dx(1))*180/pi;
    
    hold off
    plot3([p(1);dxt(1)],[p(2);dxt(2)],[p(3);dxt(3)],'b-')
    hold on
    plot3([p(1);dyt(1)],[p(2);dyt(2)],[p(3);dyt(3)],'r-')
    plot3([p(1);dzt(1)],[p(2);dzt(2)],[p(3);dzt(3)],'k-')
    
    %set(h1,'Ydir','reverse')
    %set(h1,'Zdir','reverse')
    
    grid on
    axis equal
    
    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis([-1 1 -1 1 -1 1])
    
    
end

