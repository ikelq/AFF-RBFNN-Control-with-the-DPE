%% code of paper "Adaptive Feedforward RBF Neural Network Control with 
%the Deterministic Persistence of Excitation"
%Authors:  Qiong Liu  Dongyu Li
% The details can be seen in the section 5 "Simulation Results"


% date_postprocesing_plot
clear
clc
close all

load("model_based.mat")
e_MB=e; 
de_MB=de;
error_RBF_MB0=ee_Kr;
error_RBF_MB=ee_appro;


load("optimal_hideen_node_10.mat")
e_O=e;
de_O=de;

hidden_node_O=Mu;
evoloving_S_O=SS2;

output_RBF_O=TTa;
error_RBF_O0=ee_Kr;
error_RBF_O=ee_appro;

TTa_O = TTa;
Tau_O = Tau;

weight_O=WW;
for i= 1:length(weight_O)
    norm_O(i) = norm(weight_O(:,i));
end

load("reduced_lattice.mat")
e_rl=e;
de_rl=de;

hidden_node_rl=Mu;
evoloving_S_rl=SS2;

output_RBF_rl=TTa;
error_RBF_rl0=ee_Kr;
error_RBF_rl=ee_appro;

TTa_rl = TTa;
Tau_rl = Tau;

weight_rl=WW;
for i= 1:length(weight_rl)
    norm_rl(i) = norm(weight_rl(:,i));
end


load("complete_lattice.mat")
e_cl=e;
de_cl=de;

hidden_node_cl=Mu;
evoloving_S_cl=SS2;

output_RBF_cl=TTa;
error_RBF_cl0=ee_Kr;
error_RBF_cl=ee_appro;

TTa_cl = TTa;
Tau_cl = Tau;

weight_cl=WW;
for i= 1:length(weight_cl)
    norm_cl(i) = norm(weight_cl(:,i));
end

plot(t,[norm_O;norm_rl;norm_cl])

figure
plot_line(ZZ(1,:),ZZ(2,:),'x_{1d}','x_{2d}',[-1.8,0])
hold on
plot(hidden_node_cl(1,:)',hidden_node_cl(2,:)','*','linewidth',1)
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\nodes1',...
 '-depsc',  '-painters','-r600')

figure
plot_line(ZZ(1,:),ZZ(2,:),'x_{1d}','x_{2d}',[-1.8,0])
hold on
plot(hidden_node_rl(1,:)',hidden_node_rl(2,:)','*','linewidth',0.5)
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\nodes2',...
 '-depsc',  '-painters','-r600')

figure
plot_line(ZZ(1,:),ZZ(2,:),'x_{1d}','x_{2d}',[-1.8,0])
hold on
plot(hidden_node_O(1,:)',hidden_node_O(2,:)','*','linewidth',0.5)
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\nodes3',...
 '-depsc',  '-painters','-r600')



figure
plot_line_S(ts,evoloving_S_cl,'t [s]','Evoloving RBFs',[-0.66,0.5])
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\S1',...
 '-depsc',  '-painters','-r600')

figure
plot_line_S(ts,evoloving_S_rl,'t [s]','Evoloving RBFs',[-0.66,0.5])
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\S2',...
 '-depsc',  '-painters','-r600')

figure
plot_line_S(ts,evoloving_S_O,'t [s]','Evoloving RBFs',[-0.66,0.5])
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\S3',...
 '-depsc',  '-painters','-r600')



t1=1:100/size;
t2=980/size:1000/size;
t0=80/size:100/size;

%
max_error_RBF_MB_10=max(error_RBF_MB0(t0))
max_error_RBF_MB_20=max(error_RBF_MB0(t2))
max_error_RBF_MB_1=max(error_RBF_MB(t0))
max_error_RBF_MB_2=max(error_RBF_MB(t2))

max_track_MB_1=max(e_MB(t0))
max_track_MB_2=max(e_MB(t2))


%error_RBF_cl0
max_error_RBF_cl_10=max(error_RBF_cl0(t0))
max_error_RBF_cl_20=max(error_RBF_cl0(t2))
max_error_RBF_cl_1=max(error_RBF_cl(t0))
max_error_RBF_cl_2=max(error_RBF_cl(t2))

max_track_cl_1=max(e_cl(t0))
max_track_cl_2=max(e_cl(t2))

%
max_error_RBF_rl_10=max(error_RBF_rl0(t0))
max_error_RBF_rl_20=max(error_RBF_rl0(t2))
max_error_RBF_rl_1=max(error_RBF_rl(t0))
max_error_RBF_rl_2=max(error_RBF_rl(t2))

max_track_rl_1=max(e_rl(t0))
max_track_rl_2=max(e_rl(t2))

%
max_error_RBF_rl_10=max(error_RBF_O0(t0))
max_error_RBF_rl_20=max(error_RBF_O0(t2))
max_error_RBF_rl_1=max(error_RBF_O(t0))
max_error_RBF_rl_2=max(error_RBF_O(t2))

max_track_O_1=max(e_O(t0))
max_track_O_2=max(e_O(t2))


% figure
% plot(t, [error_RBF_MB',error_RBF_cl',error_RBF_rl',error_RBF_O' ])
% legend
% xd1
label_y=["Desired Trajectory","Tracking Error"];
legend_y1="";
legend_y2="";
plot_2line(t(t1),xd1(t1)',[e_MB(t1)',e_cl(t1)',e_rl(t1)',e_O(t1)'],'t [s]',label_y,...
    legend_y1,legend_y2,[-9,-0.1;-9,0.01]);
plot_local_detial ([0.5 0.35 0.2 0.1],t(t1),[e_MB(t1)',e_cl(t1)',e_rl(t1)',e_O(t1)'],[90 100]);
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\Tracking_performance_11',...
 '-depsc',  '-painters','-r600')

label_y=["Desired Trajectory","Tracking Error"];
legend_y1="";
legend_y2=["Model Based","Lattice","Reduced Lattice","Optimal"];
plot_2line(t(t2),xd1(t2)',[e_MB(t2)',e_cl(t2)',e_rl(t2)',e_O(t2)'],'t [s]',label_y,legend_y1,legend_y2,...
    [978,0;978,0]);
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\Tracking_performance_12',...
 '-depsc',  '-painters','-r600')

% controller output
label_y=["RBFNN Output", "Approximation Error"];
legend_y1="";
legend_y2="";
plot_2line(t(t1),[TTa_O(t1)',TTa_rl(t1)',TTa_cl(t1)'],[error_RBF_cl0(t1)',error_RBF_rl0(t1)',error_RBF_O0(t1)'],'t [s]',label_y,...
    legend_y1,legend_y2,[-9,-0.1;-9,4]);
plot_local_detial ([0.4 0.25 0.4 0.2],t(t1),[error_RBF_cl0(t1)',error_RBF_rl0(t1)',error_RBF_O0(t1)'],[90 100]);
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\RBFNN_Output_1',...
 '-depsc',  '-painters','-r600')

label_y=["Controller Output","RBFNN output"];
legend_y1=["Lattice","Reduced Lattice","Optimal"];
legend_y2=["Lattice","Reduced Lattice","Optimal"];
plot_2line(t(t2),[TTa_O(t2)',TTa_rl(t2)',TTa_cl(t2)'],[error_RBF_cl0(t2)',error_RBF_rl0(t2)',error_RBF_O0(t2)'],'t [s]',label_y,...
    legend_y1,legend_y2,[-9,-0.1;-9,4]);
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\RBFNN_Output_2',...
 '-depsc',  '-painters','-r600')



%xd2
label_y=["Desired trajectory","Tracking error"];
legend_y1="";
legend_y2=["Model","Original","Reduced","Optimal"];
plot_2line(t(t1),xd2(t1)',[de_cl(t1)',de_rl(t1)',de_O(t1)',de_MB(t1)'],'t [s]',label_y,...
    legend_y1,legend_y2,[-6,0;-6,0.25]);
plot_local_detial ([0.3 0.3 0.2 0.1],t(t1),[de_MB(t1)',de_cl(t1)',de_rl(t1)',de_O(t1)'],[90 100]);
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\Tracking_performance_21',...
 '-depsc',  '-painters','-r600')


label_y=["Desired trajectory  [rad]","Tracking error  [rad]"];
legend_y1="\theta_{d1}";
legend_y2=["Model","Original","Reduced","Optimal"];
plot_2line(t(t2),xd2(t2)',[de_MB(t2)',de_cl(t2)',de_rl(t2)',de_O(t2)'],'t [s]',label_y,legend_y1,legend_y2,...
    [358,0;358,0]);
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\Tracking_performance_22',...
 '-depsc',  '-painters','-r600')



%Evolving Weight

label_y=["Weights";"Norm Weights"];
legend_y1=[""];
legend_y2=[""];
plot_2line(t,weight_O',norm_O','t [s]',label_y,legend_y1,legend_y2,[-100,0;-100,1]);
ylim([0,2.5])
%plot_local_detial ([0.5 0.3 0.2 0.15], t,error_RBF_O0',[980 1000]);
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\weight_O',...
 '-depsc', '-painters', '-r600')

label_y=["Weights";"Norm Weights"];
legend_y1=[""];
legend_y2=[""];
plot_2line(t,weight_rl',norm_rl','t [s]',label_y,legend_y1,legend_y2,[-100,0;-100,1.25]);
ylim([0,2.5])
%plot_local_detial ([0.5 0.3 0.2 0.15], t,error_RBF_rl0',[980 1000]);
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\weight_rl',...
 '-depsc', '-painters', '-r600')

label_y=["Weights";"Norm Weights"];
legend_y1=[""];
legend_y2=[""];
plot_2line(t,weight_cl',norm_cl,'t [s]',label_y,legend_y1,legend_y2,[-100,0;-100,1.25]);
ylim([0,2.5])
% plot_local_detial ([0.5 0.3 0.2 0.15], t,error_RBF_cl0',[980 1000]);
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\weight_cl',...
 '-depsc', '-painters', '-r600')

% approximation error


% 
% 
% label_y=["Output of RBFNNs [N]","Errors of RBFNNs [N]","K_2e_2 [N]"];
% legend_y1=["RBF_1","RBF_2"];
% legend_y2=["error_1","error_2"];
% legend_y3=["K_{21}e_{21}","K_{22}e_{21}"];
% plot_3line(t(t1),TTa(:,t1)',ee_RBF(:,t1)',ee_Kr(:,t1)','t [s]',label_y,...
%     legend_y1,legend_y2,legend_y3,[-6,3;-6,5;-6,-10]);
% plot_local_detial ([0.5 0.12 0.2 0.1], t,ee_RBF',[90 100]);
% plot_local_detial ([0.5 0.52 0.2 0.1], t,ee_Kr',[90 100]);
% print('D:\GE\robot control\05 Feedback Feedforward Tracking Control\Approximation_performance_1',...
%  '-depsc',  '-painters','-r600')
% 
% label_y=["Output of RBFNNs [N]","Errors of RBFNNs [N]","K_2e_2  [N]"];
% legend_y1=["RBF_1","RBF_2"];
% legend_y2=["error_1","error_2"];
% legend_y3=["K_{21}e_{21}","K_{22}e_{22}"];
% plot_3line(t(t2),TTa(:,t2)',ee_RBF(:,t2)',ee_Kr(:,t2)','t [s]',...
%     label_y,legend_y1,legend_y2,legend_y3,[1894,4;1894,0;1894,0;1894,0]);
% print('D:\GE\robot control\05 Feedback Feedforward Tracking Control\Approximation_performance_2',...
%  '-depsc', '-painters', '-r600')
% 
% 
% label_y=["Evolving W_1";"Evolving W_2"];
% legend_y1=[""];
% legend_y2=[""];
% plot_2line(t(t1),WW11(:,t1)',WW22(:,t1)','t [s]',label_y,legend_y1,legend_y2,[-6,1;-6,0.1]);
% print('D:\GE\robot control\05 Feedback Feedforward Tracking Control\weight1',...
%  '-depsc', '-painters', '-r600')
% 
% label_y=["Evolving W_1";"Evolving W_2"];
% legend_y1=[""];
% legend_y2=[""];
% plot_2line(t(t2),WW11(:,t2)',WW22(:,t2)','t [s]',label_y,legend_y1,legend_y2,[1894,1.25;1894,0.2]);
% print('D:\GE\robot control\05 Feedback Feedforward Tracking Control\weight2',...
%  '-depsc',  '-painters','-r600')
% 
% 
% 




