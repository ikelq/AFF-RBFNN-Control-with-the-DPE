%% code of paper "Adaptive Feedforward RBF Neural Network Control with 
%the Deterministic Persistence of Excitation"
%Authors:  Qiong Liu  Dongyu Li
% The details can be seen in the section "Structure Design of RBFNN with a Deterministic PELevel"


% date_postprocesing_plot
clear
clc
close all

load("approximation_lattice.mat")
ee_appro_L = ee_appro;

load("approximation_reduced_lattice.mat")
ee_appro_R = ee_appro;

load("approximation_optimized.mat")
ee_appro_O = ee_appro;

% stable_error = max ( ee_appro(:,length(t)-1000:length(t)) )


T=1000;
size=0.01;
t=0:size:T;

t1=1:100/size;
t2=490/size:510/size;
t3=980/size:1000/size;

%
max_appro_L = max(ee_appro_L(t3))
max_appro_R = max(ee_appro_R(t3))
max_appro_O = max(ee_appro_O(t3))




plot3(xd1(t1),xd2(t1),F(t1+1),'linewidth',1)
xlabel('x_{1d}');
ylabel('x_{2d}');
zlabel('f')
%,'interpreter','latex'
set (gca,'position',[0.14,0.14,0.8,0.8],'fontsize',14,'linewidth',1) 
view(-30+90,30)
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\Function',...
 '-depsc',  '-painters','-r600')



legend_y2="";
plot_line_A(t(t1),[ee_appro_L(t1)',ee_appro_R(t1)',ee_appro_O(t1)'],'t [s]',...
    'Approximation error',legend_y2,[-12,-0.2])
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\approximation_performance_01',...
 '-depsc',  '-painters','-r600')

plot_line_A(t(t2),[ee_appro_L(t2)',ee_appro_R(t2)',ee_appro_O(t2)'],'t [s]',...
    'Approximation error',legend_y2,[490-2,0])
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\approximation_performance_02',...
 '-depsc',  '-painters','-r600')

legend_y2=["Lattice","Reduced Lattice","Optimized"];
plot_line_A(t(t3),[ee_appro_L(t3)',ee_appro_R(t3)',ee_appro_O(t3)'],'t [s]',...
    'Approximation error',legend_y2,[980-2,0])
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\approximation_performance_03',...
 '-depsc',  '-painters','-r600')

label_y=["Desired","Error"];
legend_y1="";
legend_y2="";
plot_2line_2(t(t1),F(t1)',[ee_appro_L(t1)',ee_appro_R(t1)',ee_appro_O(t1)'],'t [s]',label_y,...
    legend_y1,legend_y2,[-8,0;-8,-0.2]);
print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\approximation_performance_01',...
 '-depsc',  '-painters','-r600')

legend_y2="";
plot_2line(t(t2),F(t2)',[ee_appro_L(t2)',ee_appro_R(t2)',ee_appro_O(t2)'],'t [s]',label_y,...
    legend_y1,legend_y2,[-488.5,0;-488.5,0]);

print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\approximation_performance_02',...
 '-depsc',  '-painters','-r600')

legend_y2=["Lattice","Reduced Lattice","Optimized"];
plot_2line_2(t(t3),F(t3)',[ee_appro_L(t3)',ee_appro_R(t3)',ee_appro_O(t3)'],'t [s]',label_y,...
    legend_y1,legend_y2,[978.5,0;978.5,0]);

print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\LaTeX_DL_468198_240419\latex_dl\approximation_performance_03',...
 '-depsc',  '-painters','-r600')

