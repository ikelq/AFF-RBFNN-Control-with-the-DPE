function plot_line_A(x,y,label_x,label_y,legend_y, ylabel_position)
%set(gcf,'DefaultTextInterpreter','latex' );

figure
set(gcf,'PaperUnits','inches');
set(gcf,'PaperPosition',[0 0 5.2 4]);
% set(gcf,'PaperPositionMode','auto')


plot(x,y,'linewidth',1)
xlabel(label_x);
ylabel(label_y,'position',ylabel_position);
%,'interpreter','latex'
set (gca,'position',[0.14,0.16,0.78,0.78],'fontsize',14,'linewidth',1) 
% set(gcf,'PaperPositionMode','auto');
if legend_y ~= [""]
    %,'interpreter','latex'
   legend(legend_y)
end
