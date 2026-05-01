% MATLAB Script for Motor PID Real-time Monitoring
clear; clc; close all;

%% 1. Configuration
port = "COM3";          % Ensure this matches your STM32 COM port
baudrate = 115200;
window_size_ms = 5000;  % Show last 5 seconds of data

% Initialize Serial
if ~isempty(serialportlist("available"))
    s = serialport(port, baudrate);
    configureTerminator(s, "CR/LF");
    flush(s);
else
    error('COM Port not available. Check connection.');
end

%% 2. Figure Setup (Black Background)
fig = figure('Name', 'Motor PID Live Telemetry', 'Color', 'k', 'Position', [100, 100, 1000, 800]);
t = tiledlayout(3, 1, 'TileSpacing', 'compact');

% Position Plot
ax1 = nexttile; hold on; grid on;
h_pos = plot(nan, nan, 'c-', 'LineWidth', 1.5);
title('Motor Position', 'Color', 'w');
ylabel('Degrees', 'Color', 'w');

% Velocity Plot
ax2 = nexttile; hold on; grid on;
h_vel = plot(nan, nan, 'y-', 'LineWidth', 1.5);
title('Motor Velocity', 'Color', 'w');
ylabel('RPM', 'Color', 'w');

% Acceleration Plot
ax3 = nexttile; hold on; grid on;
h_acc = plot(nan, nan, 'r-', 'LineWidth', 1.5);
title('Motor Acceleration', 'Color', 'w');
ylabel('RPM/s^2', 'Color', 'w');
xlabel('Time (ms)', 'Color', 'w');

% Style All Axes
all_ax = [ax1, ax2, ax3];
set(all_ax, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);

%% 3. Real-time Data Loop
data_log = []; 
disp('Streaming started. Close the figure to stop.');

while ishandle(fig)
    if s.NumBytesAvailable > 0
        line = readline(s);
        if isempty(line), continue; end
        
        nums = str2num(line);
        % Expected format from STM32: [Time, Pos*100, Vel*100, Accel*100]
        if length(nums) == 4
            % Scale back to original units
            nums(2:4) = nums(2:4) / 100.0;
            data_log = [data_log; nums];
            
            % Limit buffer size for performance (keep last 1000 points)
            if size(data_log, 1) > 1000
                data_log = data_log(end-1000:end, :);
            end
            
            % Update Graphs
            set(h_pos, 'XData', data_log(:, 1), 'YData', data_log(:, 2));
            set(h_vel, 'XData', data_log(:, 1), 'YData', data_log(:, 3));
            set(h_acc, 'XData', data_log(:, 1), 'YData', data_log(:, 4));
            
            % Scrolling Window Logic
            current_t = data_log(end, 1);
            if current_t > window_size_ms
                xlim(all_ax, [current_t - window_size_ms, current_t + 100]);
            else
                xlim(all_ax, 'auto');
            end
            
            drawnow limitrate;
        end
    end
end

clear s;
disp('Connection closed.');
