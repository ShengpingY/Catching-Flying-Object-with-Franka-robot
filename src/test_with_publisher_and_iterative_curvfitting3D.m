%%
% initialize for ros nodes and topics
matlabnode = ros2node('matlabnode');
sub = ros2subscriber(matlabnode,'/robot0/pose');
[pub,dest] = ros2publisher(matlabnode,'/matlab','geometry_msgs/Point');

%% initalize for data structure
%initalize coordinates which will be used to fitted
x_positions=[];
y_positions=[];
z_positions=[];
positions_stack = struct;
positions_stack.x = x_positions;
positions_stack.y = y_positions;
positions_stack.z = z_positions;
%length of data stack
length = 3;
% initialize for parameters of parabolic
p_zx = [0,0,0];
p_zy = [0,0,0];

% initialize target value
dx = 0;
dy = 0;
dz = 0.95;


%% main while loop which can be stopped by pressing button s
while true
    % %% check out if loop is manually stopped
    % if kbhit()
    %     key = getkey();
    %     if key == 's'
    %         break;  % 退出循环
    %     end
    % end
    %% receive data from optitrack
   
    try
        msg=receive(sub,0.0005);

         %% fit curve
        [p_zx,p_zy,positions_stack] = iterative_fit(msg,positions_stack,length);
         tic;
         %% calculate positions at dz height
         % check out if there is enough data, if so calculate the positions
         if size(positions_stack.x,2) >= 3
            dxs = roots([p_zx(1),p_zx(2),p_zx(3)-dz]);
            dys = roots([p_zy(1),p_zy(2),p_zy(3)-dz]);
            % find out direction in x and y
            dir_x = direction(positions_stack.x);
            dir_y = direction(positions_stack.y);
            % fint out the root in right position
            dx = root_judger(dir_x,dxs);
            dy = root_judger(dir_y,dys);
         end
         tz = toc
    
    %% send target position to topic /matlab
        dest.x = dx;
        dest.y = dy;
        dest.z = dz;
        send(pub,dest);

    catch
    end
% plot(dx,dy,'*')
% axis([-1.6, -0.3, -2.4, -1.5]);
% drawnow;
end



%% define function to find out the direction on single dimension
function dir = direction(x_positions)
    differences = diff(x_positions);
    if differences >= 0
        dir = true;
    else
        dir = false;
    end
end
%%
function root = root_judger(dir,roots)
    if dir == true
        root = max(roots);
    else
        root = min(roots);
    end
end

%%
function [p_zx,p_zy,positions_stack] = iterative_fit(msg,positions_stack,length)
    x_temp = positions_stack.x;
    y_temp = positions_stack.y;
    z_temp = positions_stack.z;
    x = msg.pose.position.x;
    y = msg.pose.position.y;
    z = msg.pose.position.z;
    if size(x_temp,2)>=length
            x_temp = queue_out(x_temp);
            y_temp = queue_out(y_temp);
            z_temp = queue_out(z_temp);
            x_temp = queue_in(x_temp,x);
            y_temp = queue_in(y_temp,y);
            z_temp = queue_in(z_temp,z);
            [p_zx,p_zy] = curv_3D(x_temp,y_temp,z_temp);
    
    %如果数据小于三十且大于等于三个则开始拟合
    elseif size(x_temp,2)>=3
        x_temp = queue_in(x_temp,x);
        y_temp = queue_in(y_temp,y);
        z_temp = queue_in(z_temp,z);
        tic;
        [p_zx,p_zy] = curv_3D(x_temp,y_temp,z_temp);
        ti = toc

    % 如果数据小于三个不够拟合，则继续读取数据
    else
        p_zx = [];
        p_zy = [];
        x_temp = queue_in(x_temp,x);
        y_temp = queue_in(y_temp,y);
        z_temp = queue_in(z_temp,z);
    end
    positions_stack.x = x_temp;
    positions_stack.y = y_temp;
    positions_stack.z = z_temp;
end


%%
function updatePositions(sub, x_positions, y_positions, z_positions, length)
    % Usage: updatePositions(sub, x_positions, y_positions, z_positions, length)
    % sub: ROS subscriber object
    % x_positions, y_positions, z_positions: position queues
    % length: maximum length of the position queues

    % Receive message from the ROS subscriber
    msg = receive(sub, 100);

    % Update x_positions
    if numel(x_positions) >= length
        x_positions = queue_out(x_positions);
        x_positions = queue_in(x_positions, msg.pose.position.x);
    else
        x_positions = queue_in(x_positions, msg.pose.position.x);
    end

    % Update y_positions
    if numel(y_positions) >= length
        y_positions = queue_out(y_positions);
        y_positions = queue_in(y_positions, msg.pose.position.y);
    else
        y_positions = queue_in(y_positions, msg.pose.position.y);
    end

    % Update z_positions
    if numel(z_positions) >= length
        z_positions = queue_out(z_positions);
        z_positions = queue_in(z_positions, msg.pose.position.z);
    else
        z_positions = queue_in(z_positions, msg.pose.position.z);
    end
end
%%
function Queue_res = queue_in(Queue,value)
    Queue_res = [Queue,value];
end
%%
function Queue_res = queue_out(Queue)
    if ~isempty(Queue)
        frontValue = Queue(1);
        Queue_res = Queue(2:end);
    else
        disp('empty Queue')
    end
end

function [p_zx,p_zy] = curv_3D(x_positions,y_positions,z_positions)
    p_zx = polyfit(x_positions,z_positions,2);
    p_zy = polyfit(y_positions,z_positions,2);
end