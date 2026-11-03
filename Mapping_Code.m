%% 1. Configuration 
port = "COM3";              
baudrate = 115200;
num_steps = 32;    
layer = 0;
z_increment = 100;    % 100mm which is 10cm (using mm since that's what TOF measures in)

% Initialize Coordinate Arrays
X = []; 
Y = []; 
Z = [];

%% 2. Open Serial Port
device = serialport(port, baudrate);
device.Timeout = 10;        % 10s for handshake
configureTerminator(device, "CR/LF"); % Stops readline(device) from reading \r and \n in UART resulted readings
flush(device);

input("Press Enter in this window to send 's' and start scanning...");

%% 3. Trigger MCU and Data Collection
write(device, 's', 'char');

while true    
    try
        for j = 0:(num_steps - 1)  %Runs as many times as number of steps defined (32)

            % Read the line sent
            rawLine = readline(device); %Raw line to exclude \r\n
            
            data = sscanf(rawLine, '%u, %u, %u, %u, %u');
            
            % If we got text instead of numbers, keep reading until we hit data.
            while isempty(data) || length(data) < 3 %Doesn't do anything if at least 3 pieces of data given (the main ones used here for measurements)
                rawLine = readline(device);
                data = sscanf(rawLine, '%u, %u, %u, %u, %u');
            end
            
            % Parse valid data
            status = data(1);
            dist   = data(2);
            angle  = data(3); 
            
            if (status == 0)
                fprintf("Layer %d, Step %d: Distance = %d mm\n", layer, angle, dist);

            else
                fprintf("Invalid status, Layer %d, Step %d: Distance = %d mm, status = %d\n", layer, angle, dist, status);
            end
            
            % Coordinate Calculations
            theta = -(angle * 11.25) * (pi / 180);
            X(end+1) = dist * cos(theta);
            Y(end+1) = dist * sin(theta);
            Z(end+1) = layer * z_increment; 
        end

        fprintf("Layer %d done\n", layer);
        layer = layer + 1;

    catch
        fprintf("\nPJ0 pressed again, so spinning stopped\n");
        break;
    end
end

%% 4. 3D Visualization
figure('Name', '3D Room Reconstruction');

% Reshape the flat arrays into a grid (Matrix)
% Rows = 32 steps, layer = # of layers taken (number of times travelled
% 10cm forward

X_grid = reshape(X, num_steps, layer);
Y_grid = reshape(Y, num_steps, layer);
Z_grid = reshape(Z, num_steps, layer);

% Closes the circles by connecting the 32nd point back to the 1st point
X_grid(end+1, :) = X_grid(1, :);
Y_grid(end+1, :) = Y_grid(1, :);
Z_grid(end+1, :) = Z_grid(1, :);


% mesh connects points horizontally (circles) and vertically (between layers)
mesh(X_grid, Y_grid, Z_grid); 

colormap(jet); 
grid on;
axis equal;
xlabel('X (width)'); ylabel('Y (height)'); zlabel('Z (length)');
title('3D Room Reconstruction');
view(3); % 3D view

%% 5. Cleanup
clear device;
fprintf("Serial Port Closed.\n");