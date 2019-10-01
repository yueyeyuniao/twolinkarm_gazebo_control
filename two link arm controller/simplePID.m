% rosinit

jointStates = rossubscriber('/rrbot/joint_states');
jointStatesData = receive(jointStates,10);
currentstates = jointStatesData.Position;

jointNum = 2;
% currentstates = [3.14;0];
finalstates = [0;1.57];
maximun_vel = [1;1];
maximun_acc = [1;1];
dt = 0.1;

%% Bang-Bang control trajectory
% trajectory = [];
% for i=1:jointNum
%     period = sqrt(abs(finalstates(i)-currentstates(i))*2/maximun_acc(i));
%     acceleration_time = maximun_vel(i)/maximun_acc(i);
%     if(period>acceleration_time)
%         constantVelocityPeriod = abs(finalstates(i)-currentstates(i))-maximun_acc(i)*acceleration_time*acceleration_time;
%         qdotd = [0:maximun_vel(i)/(acceleration_time/dt):maximun_vel(i),ones(1,ceil(constantVelocityPeriod/dt)-1)*maximun_vel(i),maximun_vel(i):-maximun_vel(i)/(acceleration_time/dt):0];
%         if(currentstates(i)>finalstates(i))
%             qdotd = -1*qdotd;
%         end
% %         if((size(trajectory,2)>size(qdotd,2))&& (i>1)) 
% %             qdotd = [qdotd, ones(1,size(trajectory,2)-size(qdotd))*qdotd(end)];
% %         end
% %         if((size(trajectory,2)<size(qdotd,2))&& (i>1)) 
% %             trajectory = [trajectory,repmat(trajectory(:,end),1,size(qdotd)-size(trajectory,2))];
% %         end
%         trajectory = [trajectory;qdotd];
%         qd = [];
%         for temp = 0:dt:acceleration_time
%             qd = [qd,0.5*maximun_acc(i)*temp^2];
%         end
%         for temp = 1:(ceil(constantVelocityPeriod/dt)-1)
%             qd = [qd,qd(end)+maximun_vel(i)*dt];
%         end
%         temp_qd = qd(end);
%         for temp = dt:dt:acceleration_time
%             qd = [qd,temp_qd+maximun_vel(i)*temp-0.5*maximun_acc(i)*temp^2];
%         end
%         if(currentstates(i)>finalstates(i))
%             qd = [currentstates(i)-qd,finalstates(i)];
%         else
%             qd = [currentstates(i)+qd,finalstates(i)];
%         end
% %         if((size(trajectory,2)>size(qd,2))&& (i>1))
% %             qd = [qd, ones(1,size(trajectory,2)-size(qd))*qd(end)];
% %         end
%         trajectory = [trajectory;qd];
%     end
% end
%

%% interpolate trajectory
dstep = maximun_vel*dt/2;
traj_length = max([abs(ceil((currentstates(1)-finalstates(1))/dstep(1))),abs(ceil((currentstates(2)-finalstates(2))/dstep(2)))]);
trajectory = repmat([finalstates(1);0;finalstates(2);0],[1,traj_length]);
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



%% Publish torque command
taupub1 = rospublisher('/rrbot/joint1_effort_controller/command', 'std_msgs/Float64');
taupub2 = rospublisher('/rrbot/joint2_effort_controller/command', 'std_msgs/Float64');
tauData1 = rosmessage(taupub1);
tauData2 = rosmessage(taupub2);
traj_lenth = size(trajectory,2);
eint = 0;
Kp = 85;
Kd = 25;
Ki = 15;
for i=1:50/dt
    jointStatesData = receive(jointStates,10);
    
    if(i<traj_lenth)
        qd = [trajectory(1,i);trajectory(2,i)];
        qdotd = [trajectory(3,i);trajectory(4,i)];
    else
        qd = finalstates;
        qdotd = [0;0];
    end
    
    e = qd - jointStatesData.Position;
    edot = qdotd - jointStatesData.Velocity;
    eint = eint + e*dt;
    
    tau = Kp*e+Kd*edot+Ki*eint;
    
    tauData1.Data = tau(1);
    tauData2.Data = tau(2);
    
    send(taupub1,tauData1);
    send(taupub2,tauData2);
    
    pause(0.08);
end
