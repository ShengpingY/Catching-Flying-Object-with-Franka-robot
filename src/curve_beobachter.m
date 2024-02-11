A = [0 1 0 0 0 0 ; 0 0 0 0 0 0 ; 0 0 0 1 0 0 ; 0 0 0 0 0 0 ; 0 0 0 0 0 1 ; 0 0 0 0 0 0 ];
B = [0; 0; 0; 0; 0; -1];
C = [1 0 0 0 0 0 ; 0 0 1 0 0 0 ; 0 0 0 0 1 0];
D = [0; 0; 0; 0; 0; 0];

L =[19.0000 90.0000 -0.0000 -0.0000 0.0000 0.0000;
    0.0000 0.0000 19.0000 90.0000 -0.0000 -0.0000;
    0.0000 0.0000 0.0000 0.0000 19.0000 90.0000];
% Initialize variables
X = zeros(6, 1); % Initial state vector
dt = 0.01; % Time step (change according to your application)

% initialize for ros nodes and topics
matlabnode = ros2node('matlabnode');
sub = ros2subscriber(matlabnode,'/robot0/pose');
[pub,dest] = ros2publisher(matlabnode,'/matlab','geometry_msgs/Point');

%initalize coordinates which will be used to fitted
while true
    msg=receive(sub,0.0005);
    x = msg.pose.position.x;
    y = msg.pose.position.y;
    z = msg.pose.position.z;
    measurement = [x; y; z];
    dX = A*X + L'*(measurement - Y);
    X = X + dt*dX;
    Y = C*X;
    % Calculate intersection point with horizontal plane
    [pub_x, pub_y, pub_z, ~] = get_meetpoint([0 0 1], [], [X(2), X(4), X(6)], [X(1), X(3), X(5)]);
    
    % Publish target position to ROS topic
    dest.x = pub_x;
    dest.y = pub_y;
    dest.z = pub_z;
    send(pub, dest);

    % Plotting measurement and destination points
    plot3(measurement(1), measurement(2), measurement(3), 'r');
    plot3(pub_x, pub_y, pub_z, 'bo');
    
    % Add a pause to control the loop rate
    pause(0.1); % Adjust this value according to your application
end

function [ result ] = get_meetpoint( planevec, planepoint, linevec, linepoint )
    vp1 = planevec(1);
    vp2 = planevec(2);
    vp3 = planevec(3);
    n1 = planepoint(1);
    n2 = planepoint(2);
    n3 = planepoint(3);
    v1 = linevec(1);
    v2 = linevec(2);
    v3 = linevec(3);
    m1 = linepoint(1);
    m2 = linepoint(2);
    m3 = linepoint(3);
    vpt = v1 * vp1 + v2 * vp2 + v3 * vp3;
    if(vpt == 0)
        result = [];
    else
        t = ((n1 - m1) * vp1 + (n2 - m2) * vp2 + (n3 - m3) * vp3) / vpt;
        result = [m1+v1 * t, m2+v2*t, m3+v3*t, t];
    end
end