function plot_line(x,y,label_x,label_y,ylabel_position)
%set(gcf,'DefaultTextInterpreter','latex' );
set(gcf,'PaperUnits','inches');
set(gcf,'PaperPosition',[0 0 4 4]);
% set(gcf,'PaperPositionMode','auto')


plot(x,y,'linewidth',1)
xlabel(label_x);
ylabel(label_y,'position',ylabel_position);
%,'interpreter','latex'
set (gca,'position',[0.14,0.14,0.8,0.8],'fontsize',12,'linewidth',1) 
% set(gcf,'PaperPositionMode','auto');
xlim([-1.5, 1.5]);
ylim([-1.5, 1.5]);
end
