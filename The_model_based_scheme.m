%% code of paper "Adaptive Feedforward RBF Neural Network Control with 
%the Deterministic Persistence of Excitation"
%Authors:  Qiong Liu  Dongyu Li
% The details can be seen in the section "Simulation Result", "The lattice scheme"
clear 
clc
close all

%% 50 0.1

%% begin simulation 
T=1000;
size=0.01;
t=0:size:T;
nt=length(t);

K1=3;
K2=10;

x1=zeros(1,nt);
x2=zeros(1,nt);

% xd1=zeros(1,nt);
% xd2=zeros(1,nt);
xd1=sin(t);
xd2=cos(t);
xdd2=-sin(t);
e=zeros(1,nt);
de=zeros(1,nt);
dde=zeros(1,nt);
Tau=zeros(1,nt);

% parameter of dynamics
belta=0.7;
p1=0.4;
p2=-1.1;
p3=1.0;
q=1.498;
w=1.8;

ZZ=[xd1;xd2];

dxd2=0;
dx2=0;
%% the first step
for i=2:nt
e1 = xd1(i-1)-x1(i-1);
de1 = xd2(i-1)-x2(i-1);
dde1 = xdd2(i-1)-dx2;

r = de1+K1*e1;
dr = dde1+K1*de1;

e(i-1)=e1;
de(i-1)=de1;
dde(i-1)=dde1;


Tau(i)= K2*r + x1(i-1) - belta*(1-xd1(i-1)^2)*xd2(i-1) + K1*de1 + xdd2(i);
 %   Tau(i)= K2*r - belta*(1-x1(i-1)^2)*x2(i-1);
%  TTa(i)=W'*S ;
ee_Kr(i) = K2*r;
ee_appro(i) = dr+K2*r;
% Tau(i)= ee_Kr(i) - TTa(i) ;



x2(i)=x2(i-1)+size*dx2;
x1(i)=x1(i-1)+size*x2(i-1)+1/2*size^2*dx2;
%x1(i)=x1(i-1)+size*x2(i-1);

dx2= -x1(i-1) +  belta*(1-x1(i-1)^2)*x2(i-1) +Tau(i);
%dx2= -x1(i) +  belta*(1-x1(i)^2)*x2(i) +Tau(i);


% xd2(i)=xd2(i-1)+size*dxd2;
% xd1(i)=xd1(i-1)+size*xd2(i)+1/2*size^2*dxd2;
% dxd2=-p1*xd2(i-1)-p2*xd1(i-1)-p3*xd1(i-1)^3+q*cos(w*t(i-1));

% tt=(i-1)*size;
% xd2(i)=cos(tt);
% xd1(i)=sin(tt);
% dxd2=-sin(tt);

end

save("model_based")

figure
plot (t,e')
hold on 
plot(t,de')
hold on 
plot(t,dde')

figure
plot(t, ee_Kr)
hold on 
plot(t, ee_appro)

label_y="Tracking error [rad]";
legend_y = [ "error_1","error_2" ];
plot_line(t,e','t [s]',label_y,legend_y,[-13,0.4])
plot_local_detial ([0.2 0.4 0.4 0.2], t,e',[390 400])
% print('D:\GE\robot control\03-adaptive neural network with input saturation for rbf\IEEEtran -delta-modifaction 2\e_q_b_g'...
%     ,'-depsc','-r600')

plot_line(t,WW','t [s]',label_y,legend_y,[-13,0.4])


label_y=["Output of RBFNNs [N]","Errors of RBFNNs [N]"];
legend_y1=["RBF_1","RBF_2"];
legend_y2=["error_1","error_2"];
plot_2line(t,TTa',ee_Kr','t [s]',label_y,legend_y1,legend_y2,[-13,6;-13,7.5])
plot_local_detial ([0.2 0.2 0.2 0.1], t,ee_RBF',[990 1000])
% print('D:\GE\robot control\03-adaptive neural network with input saturation for rbf\IEEEtran -delta-modifaction 2\error_b_g',...
%     '-depsc','-r600')



last_100_seconds_e_q=e_q(:,length(t)-100:length(t));
%mean_e_q_last_10=mean (( last_10_seconds_e_q').^2)
max_eq_100= max(last_100_seconds_e_q')
last_100_seconds_ee_RBF=ee_Kr(:,length(t)-10000:length(t));
%mean_ee_RBF_last_10=mean (( last_10_seconds_ee_RBF).^2,2)
max_eRBF_100= max(last_100_seconds_ee_RBF')


% last_20_seconds_e_q=e_q(:,200000-10000:200000);
% mean_e_q_last_20=mean (( last_20_seconds_e_q').^2)
% max_eq_20= max(last_20_seconds_e_q')
% last_20_seconds_ee_RBF=ee_RBF(:,200000-10000:200000);
% mean_ee_RBF_last_20=mean (( last_20_seconds_ee_RBF).^2,2)
% max_eRBF_20= max(last_20_seconds_ee_RBF')
 
 
 
 figure
 plot(t,normS)
 set (gca,'position',[0.1,0.1,0.8,0.8] );
 legend('Norm S')
%  print('Norm_s_g','-depsc')
% figure
% plot(t,ddq(1,:),t,ddq(2,:))
% xlabel('t [s]'); ylabel('accleration');
% figure
% plot(t,dq(1,:),t,dq(2,:))
% xlabel('t [s]'); ylabel('velocity');
% figure; 
% subplot(2,1,1);
% plot(t,q(1,:)',t,qr(1,:)');xlabel('t [s]'); ylabel('q1 and qd1');
% title('Model based Control with the Full State Feedback');
% subplot(2,1,2);
% plot(t,e_q(1,:)');xlabel('t [s]'); ylabel('error e1');
% 
% 
% figure; 
% subplot(2,1,1);
% plot(t,q(2,:)',t,qr(2,:));xlabel('t [s]'); ylabel('q2 and qd2');
% title('Model based Control with the Full State Feedback');
% subplot(2,1,2);
% plot(t,e_q(2,:)');xlabel('t [s]'); ylabel('error e2');
% 
% 
% figure;
% plot(t,Tau(1,:)',t,Tau(2,:)');title('Adaptive Neural Netwok Control with the Full State Feedback');
% legend('\tau_1','\tau_2'); xlabel('t [s]'); ylabel('Control inputs');

WW11=WW(:,1,:);
WW11=WW11(:,:);
WW22=WW(:,2,:);
WW22=WW22(:,:);

 
label_y=["Evolving W_{700-730}";"Norm W"];
legend_y1=[""];
legend_y2=[""];

plot_2line(t,WW11',Norm_W1','t [s]',label_y,legend_y1,legend_y2,[-13,3;-13,3]);
% 
% ylabel("\textbf{Evolving $\hat{W}_{700-730}$}",'interpreter','latex')
% annotation('arrow',[0.4 0.3],[0.83 0.78]);
% text(75,4,'$\hat{W}_{bg}$','interpreter','latex');
% print('D:\GE\robot control\03-adaptive neural network with input saturation for rbf\IEEEtran -delta-modifaction 2\Norm_W1_bg',...
%     '-depsc', '-r600')



% label_y=["\textbf{Evolving $\hat{W}_{700-730}$}";"Norm W"];
legend_y1=[""];
legend_y2=[""];
plot_2line(t,WW22',Norm_W2','t [s]',label_y,legend_y1,legend_y,[-13,0.15;-13,1.5]);

% ylabel("\textbf{Evolving $\hat{W}_{700-730}$}",'interpreter','latex')
% annotation('arrow',[0.35 0.25],[0.8 0.75]);
% text(65,0.15,'$\hat{W}_{bg}$','interpreter','latex');
% print('D:\GE\robot control\03-adaptive neural network with input saturation for rbf\IEEEtran -delta-modifaction 2\Norm_W2_bg', ...
%     '-depsc', '-r600')

 
 
%  figure;
% subplot(2,1,1)
% set (gca,'position',[0.1,0.55,0.8,0.4] )
% plot(t,Norm_W1')
% xlabel('t [s]'); ylabel('Norm W1','Position',[-13,5])
% subplot(2,1,2)
% set (gca,'position',[0.1,0.1,0.8,0.4] )
% plot(t,Norm_W2');
% xlabel('t [s]');ylabel('Norm W2','Position',[-13,2]) 
%  print('Norm_W_b_g','-depsc')

 

% figure;
% plot(linspace(0,Time,STeps),eps');
% title('Adaptive Neural Netwok Control with the Full State Feedback');
% xlabel('t [s]'); ylabel('Approximation errors');% approximation error btw Neural network and the model
% 
% figure;plot(tout,eout); title('Adaptive Neural Netwok Control with the Full State Feedback');
% xlabel('t [s]'); ylabel('Norm of errors ||z_1||');   % Norm Errors
%  save("RBF_reference_hideen_node_20_nobias")

