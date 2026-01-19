%% --- Internal coordinates: joint positions
figure('Position',[50 400 700 250]);
for m = 1:2
    subplot(1,2,m)
    plot(tspan, Ps.q_ref(m,:), 'r', tspan, Ps.q(m,:), 'b', 'LineWidth',1.2)
    grid on
    title(['Joint position q(' num2str(m) ')'])
    xlabel('Time [s]')
    ylabel('Position [deg]')
    legend('Reference','Actual','Location','NorthEast')
end

%% --- Internal coordinates: joint velocities
figure('Position',[50 50 700 250]);
for m = 1:2
    subplot(1,2,m)
    plot(tspan, Ps.dq_ref(m,:), 'r', tspan, Ps.dq(m,:), 'b', 'LineWidth',1.2)
    grid on
    title(['Joint speed dq(' num2str(m) ')'])
    xlabel('Time [s]')
    ylabel('Speed [deg/s]')
    legend('Reference','Actual','Location','NorthEast')
end

%% --- External coordinates: Cartesian positions
figure('Position',[800 50 700 250]);
for m = 1:2
    subplot(1,2,m)
    plot(tspan, Ps.X_ref(m,:), 'r', tspan, Ps.X(m,:), 'b', 'LineWidth',1.2)
    grid on
    if m==1
        title('Cartesian position x')
        ylabel('Position [m]')
    else
        title('Cartesian position z')
        ylabel('Position [m]')
    end
    xlabel('Time [s]')
    legend('Reference','Actual','Location','NorthEast')
end

%% --- Joint torques
figure('Position',[800 400 700 250]);
for m = 1:2
    subplot(1,2,m)
    plot(tspan, Ps.Tau(m,:), 'b', 'LineWidth',1.2)
    grid on
    title(['Joint torque - joint ' num2str(m)])
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    legend(['tau ' num2str(m)],'Location','NorthEast')
end
