function [t, x] = simulation(params, x0, tspan, framerate, playback_speed, filename)
    % Solve the ODE
    t_sim = tspan(1):(playback_speed/framerate):tspan(2);
    disp("Solving ODE...");
    [t, x] = ode45(@(t,x) dynamics(t, x, controller(t, x, params), params), t_sim, x0);
    
    % Define desired joint configuration (qd); if not provided, default to [0;0]
    if isfield(params, 'qd')
        qd = params.qd;
    else
        qd = [0; 0];
    end
    
    gifFile = filename + ".gif";
    fig = figure;
    
    for k = 1:length(t)
        % Compute joint positions for the double pendulum
        q1 = x(k,1); q2 = x(k,2);
        x1 = params.l1 * sin(q1);
        y1 = -params.l1 * cos(q1);
        x2 = x1 + params.l2 * sin(q1 + q2);
        y2 = y1 - params.l2 * cos(q1 + q2);
        lt = params.l1 + params.l2;
        
        % Compute the norm of the tracking error up to the current time step
        error_norm = sqrt( (x(1:k,1) - qd(1)).^2 + (x(1:k,2) - qd(2)).^2 );
        
        % Update figure: use two subplots side-by-side
        clf;
        
        % Left subplot: Double Pendulum Animation
        subplot(1,2,1);
        hold on;
        plot([0, x1], [0, y1], 'ko-', 'LineWidth', 3, 'MarkerSize', 12, 'MarkerFaceColor', 'r');
        plot([x1, x2], [y1, y2], 'ko-', 'LineWidth', 3, 'MarkerSize', 12, 'MarkerFaceColor', 'r');
        title(sprintf('Double Pendulum Animation (Playback Speed = %d)', playback_speed));
        text(lt, lt, sprintf('t = %.2f s', t(k)), 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'right');
        xlim([-(lt+(lt/10)), (lt+(lt/10))]); ylim([-(lt+(lt/10)), (lt+(lt/10))]);
        xlabel('x (m)'); ylabel('y (m)'); axis square;
        hold off;
        
        % Right subplot: Norm of the Error (||q - q_d||) vs. Time
        subplot(1,2,2);
        plot(t(1:k), error_norm, 'b-', 'LineWidth', 2);
        title('Error Norm ||q - q_d|| vs Time');
        xlabel('Time (s)'); ylabel('Error Norm');
        xlim([t(1) t(end)]); grid on;
        axis square;
        
        drawnow;
        
        % Capture and write frame to GIF
        frame = getframe(gcf);
        im = frame2im(frame);
        [A, map] = rgb2ind(im, 256);
        if k == 1
            imwrite(A, map, gifFile, 'gif', 'LoopCount', Inf, 'DelayTime', 1/framerate);
        else
            imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 1/framerate);
        end
        
        clc;
        fprintf('Progress: %d %%\n', floor(100 * k / length(t)));
    end
    
    clc;
    fprintf('Simulation complete. GIF saved to: %s\n', fullfile(fileparts(mfilename('fullpath')), gifFile));
    close;
end