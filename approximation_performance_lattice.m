%% code of paper "Adaptive Feedforward RBF Neural Network Control with 
%the Deterministic Persistence of Excitation"
%Authors:  Qiong Liu  Dongyu Li
% The details can be seen in the section "Structure Design of RBFNN with a Deterministic PELevel"

%% just show the approximation performance
clear 
clc
close all
%% begin simulation 
T=1000;
size=0.01;
t=0:size:T;
nt=length(t);

xd1=sin(t);
xd2=cos(t);
xdd2=-sin(t);

%%RBFNN
gamma=10; % for updating Weight
% width1=0.4;
width1=0.3;


% find the effective hidden node
ZZ=[xd1;xd2];
% SS1=zeros(Nodeu,1);
% for i=1:length(ZZ)
%     SS=RBF(ZZ(:,i),Muu,width1,Nodeu); 
%     SS1=max(SS1,SS);
% end
% in=find(SS1>0.7);
% Mu=Muu(:,in);
% Node=length(Mu);  
% save('hidden_node1','Mu')
load('hidden_node0');

Node=length(Mu);  

PE=zeros(Node);
for i=1:2*pi*100+2
    SS2(:,i)=RBF(ZZ(:,i),Mu,width1,Node);
    ts(i)=i*0.01;
    PE=PE+ 0.01*SS2(:,i)*SS2(:,i)';
end
% plot(ts,SS2)

minl=eig(PE)

min(minl)

% figure
% plot(ZZ(1,:)',ZZ(2,:))
% hold on
% plot(Mu(1,:)',Mu(2,:)','*','linewidth',1)
% xlabel('q_{1d}')
% ylabel('q_{2d}')
% print('D:\GE\robot control\06-Adaptive Feedforward RBF Neural-Network Learning Control\nodes2',...
%  '-depsc',  '-painters','-r600')




Mu5=Mu';
for i=1:length(Mu5)
    for j=1:length(Mu5)
        dis_Mu(i,j)=(Mu5(i,:)-Mu5(j,:))*(Mu5(i,:)-Mu5(j,:))';
    end
end



W=zeros(Node,1);
WW(:,1)=W;

dw=zeros(Node,1);
dwf=dw;
normW=zeros(1,length(t));


dxd2=0;
dx2=0;
%% the first step
for i=2:nt


Z=[xd1(i-1);xd2(i-1)];
S=RBF(Z,Mu,width1,Node );                     % RBF method is used in calculating S         
normS(i)=norm(S);
F(i) = -xd1(i-1)+ 0.7*(1-xd1(i-1)^2)*xd2(i-1);

r=   F(i)-  W'*S;
dw= gamma*S*r;              % updating law as stated
    
ee_appro(i)=r;



tt=(i-1)*size;
xd2(i)=cos(tt);
xd1(i)=sin(tt);
dxd2=-sin(tt);

W=dw*size+W;                     % Weights for next iteration
Norm_W1(i)=sqrt(W'*W);                    % Norm W1 & W2
WW(:,i)=W;
end

save("approximation_lattice",'ee_appro','F')

figure
plot(t,ee_appro)

stable_error = max ( ee_appro(:,length(t)-1000:length(t)) )



