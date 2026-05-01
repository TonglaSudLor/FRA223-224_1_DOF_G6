%% Ghost Mode PID Tuning Simple Plotter (High Speed)
% Optimized version with immediate START feedback and fast buffer draining.

clear; close all; clc;

%% Configuration
portName = 'COM3'; % Update this to your STM32's COM port
baudRate = 921600;
timeout = 0.1;

%% Initialize Data Storage
current_data = [];
previous_data_1 = []; 
previous_data_2 = []; 
is_collecting = false;
start_pos = 0;

%% Setup Plot
fig = figure('Name', 'Ghost Mode Simple Plot', 'NumberTitle', 'off', 'Color', [0.1 0.1 0.1], 'Position', [100, 100, 1000, 600]);
ax = axes(fig, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w');
hold(ax, 'on'); grid(ax, 'on'); set(ax, 'GridColor', [0.4 0.4 0.4]);

xlabel(ax, 'Time (s)'); ylabel(ax, 'Relative Position (deg)');
title(ax, 'Waiting for START flag...', 'Color', 'w');

% Fixed Axis
ylim(ax, [0 200]); xlim(ax, [0 20]);

% Line Placeholders
p_target = plot(ax, NaN, NaN, 'm--', 'LineWidth', 1.5, 'DisplayName', 'Target');
p_prev2 = plot(ax, NaN, NaN, 'Color', [1 1 0 0.15], 'LineWidth', 1.2, 'DisplayName', 'Prev Run -2');
p_prev1 = plot(ax, NaN, NaN, 'Color', [1 1 0 0.45], 'LineWidth', 1.5, 'DisplayName', 'Prev Run -1');
p_current = plot(ax, NaN, NaN, 'y', 'LineWidth', 2.0, 'DisplayName', 'Real-time');

legend(ax, 'TextColor', 'w', 'Location', 'northeast', 'Color', [0.2 0.2 0.2]);

% Status Overlay
txt_info = annotation('textbox', [0.3, 0.01, 0.4, 0.05], 'String', 'Ready', 'Color', 'w', 'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');

%% Serial Communication
try
    s = serialport(portName, baudRate, 'Timeout', timeout);
    configureTerminator(s, "CR/LF"); flush(s);
    fprintf('Connected to %s. Waiting for START...\n', portName);
    
    while ishandle(fig)
        % OPTIMIZATION: Drain the serial buffer as fast as possible
        while s.NumBytesAvailable > 0
            line = readline(s);
            if isempty(line), continue; end
            
            if contains(line, "START")
                % IMMEDIATE UI FEEDBACK
                parts = strsplit(line, ',');
                if length(parts) >= 2
                    target_val = str2double(parts{2});
                    set(p_target, 'XData', [0 20], 'YData', [target_val target_val]);
                end
                
                % Shift history
                if ~isempty(current_data)
                    previous_data_2 = previous_data_1;
                    previous_data_1 = current_data;
                    if ~isempty(previous_data_1)
                        set(p_prev1, 'XData', (previous_data_1(:,1)-previous_data_1(1,1))/1000, 'YData', previous_data_1(:,2));
                    end
                    if ~isempty(previous_data_2)
                        set(p_prev2, 'XData', (previous_data_2(:,1)-previous_data_2(1,1))/1000, 'YData', previous_data_2(:,2));
                    end
                end
                
                current_data = [];
                set(p_current, 'XData', NaN, 'YData', NaN); % Clear current plot instantly
                is_collecting = true;
                start_pos = NaN;
                set(txt_info, 'String', 'Motor Running...', 'Color', 'y');
                
            elseif contains(line, "END")
                if is_collecting
                    is_collecting = false;
                    if ~isempty(current_data)
                        t = (current_data(:,1) - current_data(1,1)) / 1000.0;
                        set(p_current, 'XData', t, 'YData', current_data(:,2));
                        set(p_target, 'XData', [0 20], 'YData', [current_data(end,3) current_data(end,3)]);
                    end
                    set(txt_info, 'String', 'Run Complete', 'Color', 'w');
                end
            elseif contains(line, "DATA")
                try
                    parts = strsplit(line, ',');
                    tick = str2double(parts{2});
                    pos_raw = str2double(parts{3}) / 100.0;
                    target_raw = str2double(parts{4}) / 100.0;
                    if isnan(start_pos)
                        start_pos = pos_raw;
                        dist = target_raw - start_pos;
                        if dist < 0, dir_mult = -1; else, dir_mult = 1; end
                    end
                    rel_pos = (pos_raw - start_pos) * dir_mult;
                    rel_target = (target_raw - start_pos) * dir_mult;
                    current_data = [current_data; [tick, rel_pos, rel_target]];
                catch
                end
            elseif contains(line, "PREVIEW")
                if ~is_collecting
                    try
                        parts = strsplit(line, ',');
                        curr_abs = str2double(parts{2}) / 100.0;
                        ghost_target = str2double(parts{3}) / 100.0;
                        set(txt_info, 'String', sprintf('Next Target: %.1f deg', abs(ghost_target - curr_abs)), 'Color', 'm');
                    catch
                    end
                end
            end
        end
        drawnow limitrate;
        pause(0.0001);
    end
catch ME
    fprintf('Error: %s\n', ME.message);
    if exist('s', 'var'), clear s; end
end
clear s;
