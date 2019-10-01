% rosinit

jointStates = rossubscriber('/rrbot/joint_states');
jointStatesData = receive(jointStates,10);
currentstates = [jointStatesData.Position(1);jointStatesData.Position(3)];

jointNum = 2;
% finalstates = [3.14;0];
finalstates = [1.6;-1.0];
maximun_vel = [1;1];
maximun_acc = [1;1];
dt = 0.1;

%% interpolate trajectory
dstep = maximun_vel*dt/2;
traj_length = max([abs(ceil((currentstates(1)-finalstates(1))/dstep(1))),abs(ceil((currentstates(2)-finalstates(2))/dstep(2)))]);
trajectory = repmat([finalstates(1);0;finalstates(2);0],[1,traj_length+1]);
i = 0;
sign = 1;
if(currentstates(1)>finalstates(1))
    sign = -1;
end
temp = currentstates(1):sign*dstep(1):finalstates(1);
temp_vel = 0:sign*0.1:sign*maximun_vel(1)/2;
for i = 1:size(temp,2)
    trajectory(1,i) = temp(i);
    if(i>size(temp,2)-size(temp_vel,2))
        trajectory(2,i) = temp_vel(size(temp,2)-i+1);
    elseif(i<size(temp_vel,2))
        trajectory(2,i) = temp_vel(i);
    else
        trajectory(2,i) =sign*maximun_vel(1)/2;
    end
end
trajectory(1,i+1) = finalstates(1);
trajectory(2,i+1) = 0;
i=0;
sign = 1;
if(currentstates(2)>finalstates(2))
    sign = -1;
end
temp = currentstates(2):sign*dstep(2):finalstates(2);
temp_vel = 0:sign*0.1:sign*maximun_vel(2)/2;
for i = 1:size(temp,2)
    trajectory(3,i) = temp(i);
    if(i>size(temp,2)-size(temp_vel,2))
        trajectory(4,i) = temp_vel(size(temp,2)-i+1);
    elseif(i<size(temp_vel,2))
        trajectory(4,i) = temp_vel(i);
    else
        trajectory(4,i) = sign*maximun_vel(2)/2;
    end
end
trajectory(3,i+1) = finalstates(2);
trajectory(4,i+1) = 0;

%% Publish torque command
taupub1 = rospublisher('/rrbot/joint1_effort_controller/command', 'std_msgs/Float64');
taupub2 = rospublisher('/rrbot/joint2_effort_controller/command', 'std_msgs/Float64');
taupub3 = rospublisher('/rrbot/joint1_effort_controller_copy/command', 'std_msgs/Float64');
taupub4 = rospublisher('/rrbot/joint2_effort_controller_copy/command', 'std_msgs/Float64');
tauData1 = rosmessage(taupub1);
tauData2 = rosmessage(taupub2);
tauData3 = rosmessage(taupub3);
tauData4 = rosmessage(taupub4);
traj_lenth = size(trajectory,2);
eint1 = 0;
eint2 = 0;
Kp = [250;300];
Kd = [30;30];
Kdd = [10;10];
Ki = [150;100];
% for plots 
JOINT1_d = [];
JOINT1_with_box = [];
JOINT1_without_box = [];

JOINT2_d = [];
JOINT2_with_box = [];
JOINT2_without_box = [];

qprev = jointStatesData.Position;

for i=1:50/dt
    jointStatesData = receive(jointStates,10);
    
    % for plots
    JOINT1_with_box = [JOINT1_with_box;jointStatesData.Position(1)];
%     JOINT1_without_box = [JOINT1_without_box;jointStatesData.Position(1)];
    JOINT2_with_box = [JOINT2_with_box;jointStatesData.Position(3)];
%     JOINT2_without_box = [JOINT2_without_box;jointStatesData.Position(3)];
    %
    
    [H,C,B] = twolinkarmdynamics([jointStatesData.Position(1);jointStatesData.Position(3)],[jointStatesData.Velocity(1);jointStatesData.Velocity(3)]);
    
    if(i<traj_lenth+1)
        qd = [trajectory(1,i);trajectory(3,i)];
        JOINT_temp = qd;
        qdotd = [trajectory(2,i);trajectory(4,i)];
    else
        qd = finalstates;
        JOINT_temp = qd;
        qdotd = [0;0];
    end
    % for plots 
    JOINT1_d = [JOINT1_d;JOINT_temp(1,1)];
    JOINT2_d = [JOINT2_d;JOINT_temp(2,1)];
    %
    
    e = qd - [jointStatesData.Position(1);jointStatesData.Position(3)];
%     edot = qdotd - ([jointStatesData.Position(1);jointStatesData.Position(3)]-[qprev(1);qprev(3)])./dt;
    edot = qdotd - [jointStatesData.Velocity(1);jointStatesData.Velocity(3)];
    eint1 = eint1 + e*dt;
    
    tau = H*(Kp.*e+Kd.*edot+Ki.*eint1)+C;
    
    tauData1.Data = tau(1);
    tauData2.Data = tau(2);
    
    [H,C,B] = twolinkarmdynamics([jointStatesData.Position(2);jointStatesData.Position(4)],[jointStatesData.Velocity(2);jointStatesData.Velocity(4)]);
    
    if(i<traj_lenth+1)
        qd = [trajectory(1,i);trajectory(3,i)];
        qdotd = [trajectory(2,i);trajectory(4,i)];
    else
        qd = finalstates;
        qdotd = [0;0];
    end
    
    e = qd - [jointStatesData.Position(2);jointStatesData.Position(4)];
%     edot = qdotd - ([jointStatesData.Position(2);jointStatesData.Position(4)]-[qprev(2);qprev(4)])./dt;
    edot = qdotd - [jointStatesData.Velocity(2);jointStatesData.Velocity(4)];
    eint2 = eint2 + e*dt;
    
    tau = H*(Kp.*e+Kd.*edot+Ki.*eint2)+C;
    
    tauData3.Data = tau(1);
    tauData4.Data = tau(2);
    
    send(taupub1,tauData1);  % right arm joint 1 (base)
    send(taupub3,tauData3);  % left arm joint 1 (base)  
    send(taupub2,tauData2);  % right arm joint 2
    send(taupub4,tauData4);  % left arm joint 2
     
    pause(0.08);
%     qprev = jointStatesData.Position;
end


