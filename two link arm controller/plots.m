x = 1:0.1:50.9;
figure (1);
hold on
% r1=plot(x-1,smooth(JOINT1_d,0.1,'loess'),'r')
r1=plot(x-1,JOINT1_d,'r')
set(r1,'LineWidth',2);
% r1=plot(x-1,smooth(JOINT1_with_box,0.1,'loess'),'g')
r1=plot(x-1,JOINT1_with_box,'g')
set(r1,'LineWidth',2);
% r1=plot(x-1,smooth(JOINT1_without_box,0.1,'loess'),'b')
r1=plot(x-1,JOINT1_without_box,'b')
set(r1,'LineWidth',2);
hold off
legend('Joint 1 desired trajectory','Joint 1 trajectory with box','Joint 1 trajectory without box')

xlabel('time');
ylabel('Joint 1 trajectories');

figure (2);
hold on 
r2=plot(x-1,JOINT2_d,'r')
set(r2,'LineWidth',2);
r2=plot(x-1,JOINT2_with_box,'g')
set(r2,'LineWidth',2);
r2=plot(x-1,JOINT2_without_box,'b')
set(r2,'LineWidth',2);
hold off
legend('Joint 2 desired trajectory','Joint 2 trajectory with box','Joint 2 trajectory without box')

xlabel('time');
ylabel('Joint 2 trajectories');