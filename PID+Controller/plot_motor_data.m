% STM32 PID Telemetry Visualizer
% Setup COM Port (CHANGE THIS TO YOUR PORT)
port = "COM3"; 
baudrate = 115200;

% --- Fixed Scale Configuration ---
% Edit these values to change the Y-axis range
limitPos = [-180, 180];    % Position range (Degrees)
limitVel = [-50, 50];      % Velocity range (RPM)
limitAcc = [-500, 500];    % Acceleration range (RPM/s^2)

% Close existing connections
if ~isempty(serialportlist("available"))
    clear device;
end

% Initialize Serial
device = serialport(port, baudrate);
configureTerminator(device, "CR/LF");

% Figure Setup (Black Background)
fig = figure('Color', 'k', 'Name', 'STM32 Motor Real-time Telemetry');

ax1 = subplot(3,1,1); title('Position (Degrees)', 'Color', 'w'); grid on; hold on;
ylim(ax1, limitPos);

ax2 = subplot(3,1,2); title('Velocity (RPM)', 'Color', 'w'); grid on; hold on;
ylim(ax2, limitVel);

ax3 = subplot(3,1,3); title('Acceleration (RPM/s^2)', 'Color', 'w'); grid on; hold on;
ylim(ax3, limitAcc);

set([ax1, ax2, ax3], 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);

% Animated Lines
linePos = animatedline(ax1, 'Color', 'cyan', 'LineWidth', 1.5);
linePosRef = animatedline(ax1, 'Color', 'w', 'LineStyle', '--', 'LineWidth', 1.0); % Position Ref
linePosGhost = animatedline(ax1, 'Color', 'm', 'LineStyle', ':', 'LineWidth', 1.2); % Ghost Position (Magenta dotted)
lineVel = animatedline(ax2, 'Color', 'yellow', 'LineWidth', 1.5);
lineRef = animatedline(ax2, 'Color', 'w', 'LineStyle', '--', 'LineWidth', 1.0); % Velocity Ref
lineAcc = animatedline(ax3, 'Color', 'red', 'LineWidth', 1.5);

% Data Buffer
startTime = datetime('now');
disp(['Streaming started on ', port, '. Close figure to stop.']);
legend(ax1, {'Actual', 'S-Curve Ref', 'Ghost Target'}, 'TextColor', 'w');
legend(ax2, {'Actual RPM', 'Target RPM'}, 'TextColor', 'w');

% บังคับให้ MATLAB วาดหน้าต่างขึ้นมาทันทีก่อนเข้าลูป
drawnow; 

while ishandle(fig)
    if device.NumBytesAvailable > 0
        dataStr = readline(device);
        nums = str2double(split(dataStr, ','));
        
        % Format: [Time, Pos, Vel, Acc, RefVel, RefPos, GhostPos] (7 values)
        if length(nums) == 7
            t = seconds(datetime('now') - startTime);
            
            val_pos = nums(2) / 100.0;
            val_vel = nums(3) / 100.0;
            val_acc = nums(4) / 100.0;
            val_ref_vel = nums(5) / 100.0;
            val_ref_pos = nums(6) / 100.0;
            val_ghost_pos = nums(7) / 100.0;
            
            if ~isnan(val_pos)
                addpoints(linePos, t, val_pos);
                addpoints(linePosRef, t, val_ref_pos);
                addpoints(linePosGhost, t, val_ghost_pos);
                addpoints(lineVel, t, val_vel);
                addpoints(lineRef, t, val_ref_vel);
                addpoints(lineAcc, t, val_acc);
                
                % Scroll window
                if t > 10
                    ax1.XLim = [t-10 t];
                    ax2.XLim = [t-10 t];
                    ax3.XLim = [t-10 t];
                end
                drawnow limitrate;
            end
        end
    else
        drawnow limitrate; 
        pause(0.01); 
    end
end

clear device;
disp('Streaming stopped.');