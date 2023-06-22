function [csi_d,dot_csi_d,ddot_csi_d,psi_d,dot_psi_d,ddot_psi_d,t1] = poly7(t_iniz,ttot,pi,pf,psi0,psif)
%% SAMPLING TIME
Ts=0.001;

%% PLANNER
t1=linspace(t_iniz,ttot,round((ttot-t_iniz)/Ts));
p= zeros(4,length(t1)); dot_p= zeros(4,length(t1)); ddot_p= zeros(4,length(t1));
%Initial and final conditions
x0= pi(1); xf=pf(1); dot_x0= 0; dot_xf=0; ddot_x0=0; ddot_xf=0; dddot_x0 = 0; dddot_xf = 0;
y0= pi(2); yf=pf(2); dot_y0= 0; dot_yf=0; ddot_y0=0; ddot_yf=0; dddot_y0 = 0; dddot_yf = 0;
z0= pi(3); zf=pf(3); dot_z0= 0; dot_zf=0; ddot_z0=0; ddot_zf=0; dddot_z0 = 0; dddot_zf = 0;

dot_psi0= 0; dot_psif=0; ddot_psi0=0; ddot_psif=0; dddot_psi0 = 0; dddot_psif = 0;
p0=[x0,y0,z0,psi0]; dot_p0=[dot_x0,dot_y0,dot_z0,dot_psi0]; ddot_p0=[ddot_x0,ddot_y0,ddot_z0,ddot_psi0]; dddot_p0=[dddot_x0,dddot_y0,dddot_z0,dddot_psi0];
pf=[xf,yf,zf,psif]; dot_pf=[dot_xf,dot_yf,dot_zf,dot_psif]; ddot_pf=[ddot_xf,ddot_yf,ddot_zf,ddot_psif]; dddot_pf=[dddot_xf,dddot_yf,dddot_zf,dddot_psif];
%5-th order polynomial
a0=zeros(1,4); a1=zeros(1,4); a2=zeros(1,4); a3=zeros(1,4); a4=zeros(1,4); a5=zeros(1,4); a6 = zeros(1,4); a7 = zeros(1,4);

for j=1:4
    A = [t_iniz^7, t_iniz^6, t_iniz^5, t_iniz^4, t_iniz^3, t_iniz^2, t_iniz, 1;
        ttot^7, ttot^6, ttot^5, ttot^4, ttot^3, ttot^2, ttot, 1;
        7*t_iniz^6, 6*t_iniz^5, 5*t_iniz^4, 4*t_iniz^3, 3*t_iniz^2, 2*t_iniz, 1, 0;
        7*ttot^6, 6*ttot^5, 5*ttot^4, 4*ttot^3, 3*ttot^2, 2*ttot, 1, 0;
        42*t_iniz^5, 30*t_iniz^4, 20*t_iniz^3, 12*t_iniz^2, 6*t_iniz, 2, 0, 0;
        42*ttot^5, 30*ttot^4, 20*ttot^3, 12*ttot^2, 6*ttot, 2, 0, 0;
        210*t_iniz^4, 120*t_iniz^3, 60*t_iniz^2, 24*t_iniz, 6, 0, 0, 0;
        210*ttot^4, 120*ttot^3, 60*ttot^2, 24*ttot, 6, 0, 0, 0];
    b = [p0(j) pf(j) dot_p0(j) dot_pf(j) ddot_p0(j) ddot_pf(j) dddot_p0(j) dddot_pf(j)]';
    a_temp = A\b;
    a7(j) = a_temp(1);
    a6(j) = a_temp(2);
    a5(j) = a_temp(3);
    a4(j) = a_temp(4);
    a3(j) = a_temp(5);
    a2(j) = a_temp(6);
    a1(j) = a_temp(7);
    a0(j) = a_temp(8);
    
    %trajectories
    p(j,:)=a7(j)*t1.^7 + a6(j)*t1.^6 + a5(j)*t1.^5 +a4(j)*t1.^4 +a3(j)*t1.^3 +a2(j)*t1.^2 +a1(j)*t1 +a0(j);
    dot_p(j,:) = 7*a7(j)*t1.^6 + 6*a6(j)*t1.^5 + 5*a5(j)*t1.^4 +4*a4(j)*t1.^3 +3*a3(j)*t1.^2 +2*a2(j)*t1 +a1(j);
    ddot_p(j,:) = 42*a7(j)*t1.^5 + 30*a6(j)*t1.^4 + 5*4*a5(j)*t1.^3 +4*3*a4(j)*t1.^2 +3*2*a3(j)*t1 +2*a2(j);
   


end
csi_d=p(1:3,:); dot_csi_d=dot_p(1:3,:); ddot_csi_d=ddot_p(1:3,:);
psi_d=p(4,:); dot_psi_d=dot_p(4,:); ddot_psi_d=ddot_p(4,:);


end