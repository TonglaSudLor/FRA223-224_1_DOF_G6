% Characterization Visualizer for Feed-Forward Tuning
clear; clc; close all;

port = "COM3"; % CHANGE TO YOUR PORT
baudrate = 115200;

% Setup Figure
fig = figure('Color', 'k', 'Name', 'Feed-Forward Characterization (25% PWM Test)');
ax = axes('Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
hold on; grid on;
title('Velocity Characterization', 'Color', 'w');
ylabel('RPM', 'Color', 'w');
xlabel('Time (s)', 'Color', 'w');
ylim([-50, 150]);

lineVel = animatedline(ax, 'Color', 'y', 'LineWidth', 1.5, 'DisplayName', 'Actual RPM');
lineRef = animatedline(ax, 'Color', 'c', 'LineStyle', '--', 'LineWidth', 1.0, 'DisplayName', 'Reference (100 RPM)');
legend('TextColor', 'w', 'Location', 'northeast');

% Initialize Serial
device = serialport(port, baudrate);
configureTerminator(device, "CR/LF");
flush(device);

disp(['Waiting for START flag from STM32 on ', port, '...']);
disp('Send command "1" from ESP32 to begin.');

isCapturing = false;
startTime = 0;

while ishandle(fig)
    if device.NumBytesAvailable > 0
        dataStr = readline(device);
        
        % Handle Flags
        if contains(dataStr, "START")
            isCapturing = true;
            clearpoints(lineVel);
            clearpoints(lineRef);
            startTime = datetime('now');
            disp('START received. Capturing data...');
            continue;
        elseif contains(dataStr, "FINISH")
            if isCapturing
                isCapturing = false;
                disp('FINISH received. Data capture complete.');
                disp('--- CALCULATION ---');
                [~, vData] = getpoints(lineVel);
                if ~isempty(vData)
                    avgRPM = mean(vData(end-min(20, length(vData)):end)); % Avg last 20 points
                    kff = 25 / avgRPM;
                    fprintf('Steady State RPM: %.2f\n', avgRPM);
                    fprintf('SUGGESTED Kff: %.5f\n', kff);
                    disp('-------------------');
                end
            end
            continue;
        end
        
        % Process Telemetry
        if isCapturing
            nums = str2double(split(dataStr, ','));
            if length(nums) == 6
                t = seconds(datetime('now') - startTime);
                val_vel = nums(3) / 100.0;
                val_ref = nums(5) / 100.0;
                
                if ~isnan(val_vel)
                    addpoints(lineVel, t, val_vel);
                    addpoints(lineRef, t, val_ref);
                    
                    if t > 5
                        ax.XLim = [t-5 t];
                    end
                    drawnow limitrate;
                end
            end
        end
    end
    pause(0.001);
end

clear device;
disp('Script closed.');
