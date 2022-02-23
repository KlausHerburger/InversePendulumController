load values2.mat

t = values(1,:);
w = values(2,:);
phi = values(3,:);
F = values(4,:);
L = 0.5;

figure;
subplot(2,1,1)
plot(t,w,'LineWidth',3)
xlabel('Time [s]')
ylabel('Position [m]')
title('Carriage position')
subplot(2,1,2)
plot(t,phi,'LineWidth',3)
xlabel('Time [s]')
ylabel('Angle [rad]')
title('Pendulum angle')

%Animation
h_fig = figure(2);
set(gcf, 'Position', get(0, 'Screensize'));
title('Inverse Pendulum animation')
axis equal 
xlim([-0.2,0.6]);
ylim([-0.2,0.7]);
for idx = 1:1000:(length(t))
    figure(h_fig); cla; hold on;
    p = w(idx);
    x = -L*sin(phi(idx));
    y = L*cos(phi(idx));
    x1 = p-0.1;
    x2 = p+0.1;
    y1 = -0.05;
    y2 = 0;
    x_ = [x1, x2, x2, x1, x1];
    y_ = [y1, y1, y2, y2, y1];
    plot(x_, y_, 'k-', 'LineWidth', 5);
    plot([p p+x],[0 y],'k-','linewidth',10,'markersize',7)
end 
