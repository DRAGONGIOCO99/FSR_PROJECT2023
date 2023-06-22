clc
clear all
close all

load("GOOD_PATH.mat")
Track_path(:,3)=Track_path(:,3)*-1;

% tf=60;
tk=[0 10 20 30 50 80 120];

psi=[0 -pi/6 -pi/2 0 pi/6 pi/2 pi];


[csi_d1,dot_csi_d1,ddot_csi_d1,psi_d1,dot_psi_d1,ddot_psi_d1,T1]=poly7(tk(1),tk(2),Track_path(1,:),Track_path(2,:),psi(1),psi(2));
[csi_d2,dot_csi_d2,ddot_csi_d2,psi_d2,dot_psi_d2,ddot_psi_d2,T2]=poly7(tk(2),tk(3),Track_path(2,:),Track_path(3,:),psi(2),psi(3));
[csi_d3,dot_csi_d3,ddot_csi_d3,psi_d3,dot_psi_d3,ddot_psi_d3,T3]=poly7(tk(3),tk(4),Track_path(3,:),Track_path(4,:),psi(3),psi(4));
[csi_d4,dot_csi_d4,ddot_csi_d4,psi_d4,dot_psi_d4,ddot_psi_d4,T4]=poly7(tk(4),tk(5),Track_path(4,:),Track_path(5,:),psi(4),psi(5));
[csi_d5,dot_csi_d5,ddot_csi_d5,psi_d5,dot_psi_d5,ddot_psi_d5,T5]=poly7(tk(5),tk(6),Track_path(5,:),Track_path(6,:),psi(5),psi(6));
[csi_d6,dot_csi_d6,ddot_csi_d6,psi_d6,dot_psi_d6,ddot_psi_d6,T6]=poly7(tk(6),tk(7),Track_path(6,:),Track_path(7,:),psi(6),psi(7));

% 
%  figure 
%  plot(t1,psi_d);
%  grid on
% csi_d=[csi_d1 csi_d2(:,2:end)];
% dot_csi_d=[dot_csi_d1 dot_csi_d2(:,2:end)];
% ddot_csi_d=[ddot_csi_d1 ddot_csi_d2(:,2:end)];

csi_d=[csi_d1 csi_d1(:,end) csi_d2(:,3:end) csi_d2(:,end)  csi_d3(:,3:end) csi_d3(:,end)  csi_d4(:,3:end) csi_d4(:,end)  csi_d5(:,3:end) csi_d5(:,end)  csi_d6(:,3:end)];
dot_csi_d=[dot_csi_d1 dot_csi_d1(:,end) dot_csi_d2(:,3:end) dot_csi_d2(:,end) dot_csi_d3(:,3:end) dot_csi_d3(:,end) dot_csi_d4(:,3:end) dot_csi_d4(:,end) dot_csi_d5(:,3:end) dot_csi_d5(:,end) dot_csi_d6(:,3:end)];
ddot_csi_d=[ddot_csi_d1 ddot_csi_d1(:,end) ddot_csi_d2(:,3:end) ddot_csi_d2(:,end) ddot_csi_d3(:,3:end) ddot_csi_d3(:,end) ddot_csi_d4(:,3:end) ddot_csi_d4(:,end) ddot_csi_d5(:,3:end) ddot_csi_d5(:,end) ddot_csi_d6(:,3:end)];


% psi_d=[psi_d1 psi_d2(2:end)];
% dot_psi_d=[dot_psi_d1 dot_psi_d2(2:end)];
% ddot_psi_d=[ddot_psi_d1 ddot_psi_d2(2:end)];

psi_d=[psi_d1 psi_d1(end) psi_d2(3:end) psi_d2(end) psi_d3(3:end) psi_d3(end) psi_d4(3:end) psi_d4(end) psi_d5(3:end) psi_d5(end) psi_d6(3:end)];
dot_psi_d=[dot_psi_d1 dot_psi_d1(end) dot_psi_d2(3:end) dot_psi_d2(end) dot_psi_d3(3:end) dot_psi_d3(end) dot_psi_d4(3:end) dot_psi_d4(end) dot_psi_d5(3:end) dot_psi_d5(end) dot_psi_d6(3:end)];
ddot_psi_d=[ddot_psi_d1 ddot_psi_d1(end) ddot_psi_d2(3:end) ddot_psi_d2(end) ddot_psi_d3(3:end) ddot_psi_d3(end) ddot_psi_d4(3:end) ddot_psi_d4(end) ddot_psi_d5(3:end) ddot_psi_d5(end) ddot_psi_d6(3:end)];

T=[T1 T2(2:end) T3(2:end) T4(2:end) T5(2:end) T6(2:end) ];

figure
plot3(csi_d(1,:),csi_d(2,:),csi_d(3,:))
 hold on
 plot3(Track_path(:,1),Track_path(:,2),Track_path(:,3),'o','MarkerFaceColor','r')

 xlabel('x')
 ylabel('y')
 zlabel('z')
 grid on

 figure 
 plot(T,csi_d);
 grid on

 figure
 plot(T,dot_csi_d);
 grid on

 figure
 plot(T,ddot_csi_d);
 grid on

 figure
 plot(T,psi_d);  
 grid on

  figure
 plot(T,dot_psi_d);  
 grid on

  figure
 plot(T,ddot_psi_d);  
 grid on



position_d=timeseries(csi_d,T);
linear_vel_d=timeseries(dot_csi_d,T);
acc_d=timeseries(ddot_csi_d,T);

psi_ref=timeseries(psi_d,T);
dot_psi_ref=timeseries(dot_psi_d,T);
ddot_psi_ref=timeseries(ddot_psi_d,T);

%% Results Plot
figure
plot3(csi_d(1,:),csi_d(2,:),-csi_d(3,:))
 hold on
 plot3(Track_path(:,1),Track_path(:,2),-Track_path(:,3),'o','MarkerFaceColor','r')

 xlabel('x')
 ylabel('y')
 zlabel('z')
 grid on
 hold on
 plot3(out.pos_act.signals.values(:,1),out.pos_act.signals.values(:,2),-out.pos_act.signals.values(:,3))

