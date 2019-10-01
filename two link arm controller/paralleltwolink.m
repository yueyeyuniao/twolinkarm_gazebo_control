% rosinit

jointStates = rossubscriber('/rrbot/joint_states');
jointStatesData = receive(jointStates,10);
currentstates = [jointStatesData.Position(1);jointStatesData.Position(3)];

jointNum = 2;
% finalstates = [3.14;0];
finalstates = [1.6;-1.4];
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
for i = 1:size(temp,2)
    trajectory(1,i) = temp(i);
    trajectory(2,i) =sign*maximun_vel(1)/2;
end
trajectory(1,i+1) = finalstates(1);
trajectory(2,i+1) = sign*0.5;
i=0;
sign = 1;
if(currentstates(2)>finalstates(2))
    sign = -1;
end
temp = currentstates(2):sign*dstep(2):finalstates(2);
for i = 1:size(temp,2)
    trajectory(3,i) = temp(i);
    trajectory(4,i) = sign*maximun_vel(2)/2;
end
trajectory(3,i+1) = finalstates(2);
trajectory(4,i+1) = sign*0.5;

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
Kp = 80;
Kd = 10;
% Ki = 10;

for i=1:50/dt
    jointStatesData = receive(jointStates,10);
    
    [H,C,B] = twolinkarmdynamics([jointStatesData.Position(1);jointStatesData.Position(3)],[jointStatesData.Velocity(1);jointStatesData.Velocity(3)]);
    
    if(i<traj_lenth+1)
        qd = [trajectory(1,i);trajectory(3,i)];
        qdotd = [trajectory(2,i);trajectory(4,i)];
    else
        qd = finalstates;
        qdotd = [0;0];
    end
    
    e = qd - [jointStatesData.Position(1);jointStatesData.Position(3)];
    edot = qdotd - [jointStatesData.Velocity(1);jointStatesData.Velocity(3)];
    eint1 = eint1 + e*dt;
    
    tau = H*(Kp*e+Kd*edot+Ki*eint1)+C;
    
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
    edot = qdotd - [jointStatesData.Velocity(2);jointStatesData.Velocity(4)];
    eint2 = eint2 + e*dt;
    
    tau = H*(Kp*e+Kd*edot+Ki*eint2)+C;
    
    tauData3.Data = tau(1);
    tauData4.Data = tau(2);
    
    send(taupub1,tauData1);
    send(taupub3,tauData3);
    send(taupub2,tauData2);
    send(taupub4,tauData4);
    
    pause(0.08);
end
