% % MATLAB script to read a CSV file and plot 3D position data with a heatmap
% % based on actual sensor magnitude and 2D projections in gray.
% 
% % Specify the file name (adjust as necessary)
% filename = "C:\Users\ltjth\Documents\Research\VS Data\VectorSurgeTrials_013SNAP.csv";
% 
% % Read the CSV file into a table
% data = readtable(filename);
% numRows = height(data);
% 
% % Exclude the last 1000 rows
% data = data(3700:(numRows)-3000, :);
% 
% % Extract X, Y, Z columns (dividing by 1000 to convert to meters if necessary)
% x = data{7:end, 12} ./ 1000; % Assuming X data is in the 12th column
% y = data{7:end, 13} ./ 1000; % Assuming Y data is in the 13th column
% z = data{7:end, 14} ./ 1000; % Assuming Z data is in the 14th column
% 
% % Calculate the fan center (average position)
% fanx = mean(data{7:end, 6} ./ 1000);
% fany = mean(data{7:end, 7} ./ 1000);
% fanz = mean(data{7:end, 8} ./ 1000);
% 
% % Extract rotation data (assuming columns 9, 10, 11 are roll, pitch, and yaw)
% roll = data{7:end, 9};
% pitch = data{7:end, 10};
% yaw = data{7:end, 11};
% 
% % Convert roll, pitch, yaw to direction vectors (assuming yaw is the heading)
% heading_x = cosd(yaw);
% heading_y = sind(yaw);
% heading_z = zeros(size(yaw)); % Assuming horizontal plane, adjust if necessary
% 
% % Load the sensor magnitude data from the separate file
% sensorFile = "C:\Users\ltjth\Documents\Research\VectorSurgeTrials13_Logs.csv"; % Adjust the path accordingly
% sensorData = readtable(sensorFile);
% sensor_magnitude = sensorData{10:end, 5};  % Assuming sensor magnitudes are in the 5th column
% 
% % Interpolate the sensor magnitude values to match the number of trajectory points
% num_trajectory_points = length(x);
% num_sensor_points = length(sensor_magnitude);
% sensor_interpolated = interp1(linspace(0, 1, num_sensor_points), sensor_magnitude, linspace(0, 1, num_trajectory_points), 'linear');
% 
% % Define the colormap
% colormap_choice = 'jet';  % Change this to your preferred colormap
% cmap = colormap(colormap_choice);  % Get the colormap matrix
% 
% % Normalize the interpolated sensor magnitudes for color mapping
% min_mag = min(sensor_interpolated);
% max_mag = max(sensor_interpolated);
% sensor_normalized = (sensor_interpolated - min_mag) / (max_mag - min_mag);
% 
% % Map normalized values to the colormap
% color_indices = round(sensor_normalized * (size(cmap, 1) - 1)) + 1;  % Map to colormap indices
% colors = cmap(color_indices, :);  % Get RGB values from colormap
% 
% % Create a 3D scatter plot with color based on interpolated sensor magnitude
% figure;
% 
% scatter3(x, y, z, 20, colors, 'filled');
% hold on;
% 
% % Plot the average (fan) position in red
% scatter3(fanx, fany, fanz, 100, 'filled', 'r');
% 
% 
% % Define gray color for projections
% gray_color = [0.5, 0.5, 0.5];  % Light gray color
% 
% % Add projections at the far edges of the plot
% xlim_vals = xlim;
% ylim_vals = ylim;
% zlim_vals = zlim;
% % Zoom out by adjusting axis limits
% buffer = 2; % Fractional amount to zoom out (adjust as needed)
% zlim([0, 5]);
% 
% % Plot shapes at the beginning and end of the trajectory
% scatter3(x(1), y(1), z(1), 100, 'filled', 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g'); % Start point (circle, green)
% scatter3(x(end), y(end), z(end), 100, 'filled', 'square', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r'); % End point (star, blue)
% 
% 
% 
% % XY Projection
% proj_x = x;
% proj_y = y;
% proj_z = zeros(size(x));
% scatter3(proj_x, proj_y, proj_z, 5, gray_color, 'filled', 'MarkerEdgeColor', 'none');
% scatter3(fanx, fany, 0, 10, 'filled', 'r');
% 
% % Plot arrows for heading every 60 data points
% scale_factor = 1;
% arrow_interval = 60; % Every 60 data points
% for i = 1:arrow_interval:length(x)
%     % Plot the heading as an arrow
% %     scaled_heading_x = heading_x(i) * scale_factor;
% %     scaled_heading_y = heading_y(i) * scale_factor;
% %     scaled_heading_z = heading_z(i) * scale_factor;
%     quiver3(x(i), y(i), proj_z(i), heading_x(i), heading_y(i), heading_z(i), 0.3, 'Color', [0 0 0], 'LineWidth', 2,'MaxHeadSize',10);
% end
% 
% % YZ Projection
% proj_x = x;
% proj_y = repmat(ylim_vals(2), size(z));
% proj_z = z;
% scatter3(proj_x, proj_y, proj_z, 5, gray_color, 'filled', 'MarkerEdgeColor', 'none');
% scatter3(fanx, proj_y, fanz, 10, 'filled', 'r');
% 
% % Set plot labels with units in meters
% xlabel('X (m)');
% ylabel('Y (m)');
% zlabel('Z (m)');
% %title('3D Position with Interpolated Sensor Magnitude Heatmap and 2D Projections in Gray');
% 
% % Add a colorbar for the sensor magnitude
% cbar = colorbar;
% cbar.Label.String = 'Normalized Sensor Magnitude';
% colormap(colormap_choice);  % Set the same colormap for the colorbar
% cbar.Ticks = linspace(0, 1, 5); % Optional: set colorbar ticks
% 
% % Set grid on for better visualization
% grid on;
% 
% % Display the plot
% view(3);
% hold off;

% MATLAB script to read a CSV file and plot 3D position data with a heatmap
% based on actual sensor magnitude and 2D projections in gray.

% Specify the file name (adjust as necessary)
filename = "C:\Users\bridg\Downloads\VectorSurgeTrials_013SNAP.csv";

% Read the CSV file into a table
data = readtable(filename)
numRows = height(data);

% Exclude the last 1000 rows
data = data(3700:(numRows)-3000, :);

% Extract X, Y, Z columns (dividing by 1000 to convert to meters if necessary)
x = data{7:end, 12} ./ 1000; % Assuming X data is in the 12th column
y = data{7:end, 13} ./ 1000; % Assuming Y data is in the 13th column
z = data{7:end, 14} ./ 1000; % Assuming Z data is in the 14th column

% Calculate the fan center (average position)
fanx = mean(data{7:end, 6} ./ 1000);
fany = mean(data{7:end, 7} ./ 1000);
fanz = mean(data{7:end, 8} ./ 1000);

% Extract rotation data (assuming columns 9, 10, 11 are roll, pitch, and yaw)
roll = data{7:end, 9};
pitch = data{7:end, 10};
yaw = data{7:end, 11};

% Convert roll, pitch, yaw to direction vectors (assuming yaw is the heading)
heading_x = cosd(yaw);
heading_y = sind(yaw);
heading_z = zeros(size(yaw)); % Assuming horizontal plane, adjust if necessary

% Load the sensor magnitude data from the separate file
sensorFile = "C:\Users\bridg\Downloads\VectorSurgeTrials13_Logs.csv"; % Adjust the path accordingly
sensorData = readtable(sensorFile)
sensor_magnitude = sensorData{10:end, 5}*1.95;  % Assuming sensor magnitudes are in the 5th column
sensor_angle = sensorData{10:end, 4}*1.95;

% Interpolate the sensor magnitude values to match the number of trajectory points
num_trajectory_points = length(x);
num_sensor_points = length(sensor_magnitude);
sensor_interpolated = interp1(linspace(0, 1, num_sensor_points), sensor_magnitude, linspace(0, 1, num_trajectory_points), 'linear');

% Define the colormap
colormap_choice = 'jet';  % Change this to your preferred colormap
cmap = colormap(colormap_choice);  % Get the colormap matrix

% Directly map the sensor magnitudes to the colormap
min_mag = min(sensor_magnitude);
max_mag = max(sensor_magnitude);
% Map sensor magnitudes to colormap indices
color_indices = round((sensor_interpolated - min_mag) / (max_mag - min_mag) * (size(cmap, 1) - 1)) + 1;  % Map to colormap indices
colors = cmap(color_indices, :);  % Get RGB values from colormap

% Create a 3D scatter plot with color based on interpolated sensor magnitude
figure;

scatter3(x, y, z, 20, colors, 'filled');
hold on;

% Plot the average (fan) position in red
scatter3(fanx, fany, fanz, 100, 'filled', 'r');

% Define gray color for projections
gray_color = [0.5, 0.5, 0.5];  % Light gray color

% Add projections at the far edges of the plot
xlim_vals = xlim;
ylim_vals = ylim;
zlim_vals = zlim;
% Zoom out by adjusting axis limits
buffer = 2; % Fractional amount to zoom out (adjust as needed)
zlim([0, 5]);

% Plot shapes at the beginning and end of the trajectory
scatter3(x(1), y(1), z(1), 100, 'filled', 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g'); % Start point (circle, green)
scatter3(x(end), y(end), z(end), 100, 'filled', 'square', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r'); % End point (star, blue)

% XY Projection
proj_x = x;
proj_y = y;
proj_z = zeros(size(x));
scatter3(proj_x, proj_y, proj_z, 5, gray_color, 'filled', 'MarkerEdgeColor', 'none');
% Plot arrows for heading every 60 data points
scale_factor = 1;
arrow_interval = 60; % Every 60 data points
for i = 1:arrow_interval:length(x)
    % Plot the heading as an arrow
    quiver_handles = quiver3(x(i), y(i), proj_z(i), heading_x(i), heading_y(i), heading_z(i), 0.3, 'Color', [0 0 0], 'LineWidth', 2, 'MaxHeadSize', 10);
end

% YZ Projection
proj_x = x;
proj_y = repmat(ylim_vals(2), size(z));
proj_z = z;
scatter3(proj_x, proj_y, proj_z, 5, gray_color, 'filled', 'MarkerEdgeColor', 'none');

% Set plot labels with units in meters
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

% Define the colormap and colorbar
colormap_choice = 'jet';  % Change this to your preferred colormap

% Ensure colormap reflects the range of data
cmap = colormap(colormap_choice);  % Get the colormap matrix

% Add a colorbar for the sensor magnitude
cbar = colorbar;
cbar.Label.String = 'Flow Sensor Magnitude (mT)';

% Set colorbar limits based on the actual sensor magnitude range
cbar.Limits = [min(sensor_magnitude), max(sensor_magnitude)];  
cbar.Ticks = linspace(min(sensor_magnitude), max(sensor_magnitude), 5);  % Optional: set colorbar ticks to match magnitude range

% Format the colorbar tick labels to two decimal places
tickLabels = arrayfun(@(x) sprintf('%.2f', x), cbar.Ticks, 'UniformOutput', false);
cbar.TickLabels = tickLabels;

% Set the colormap to match the colorbar limits
colormap(colormap_choice);
caxis([min(sensor_magnitude), max(sensor_magnitude)]);  % Scale the colormap to the data range

% Add legend for the quiver arrows
legend(quiver_handles, 'Drone Heading', 'Location', 'Best');
% Set grid on for better visualization

% Set grid on for better visualization
grid on;


% Set the camera view
view(-8.409420362244731,30.121657302517498);
hold off;

% Flow Map Trajectory Estimation
figure;

% Initial Data
flowX = sensor_magnitude.*cos(sensor_angle);
flowY = sensor_magnitude.*sin(sensor_angle);

% Create a grid for visualization
[Xq, Yq] = meshgrid(linspace(min(x), max(x), 100), linspace(min(y), max(y), 100));

if length(x) ~= length(flowX)
    % Option 1: Resample your flow data to match position data
    num_x_points = length(x);
    num_flowX_points = length(flowX);
    flow_Xinterpolated = interp1(linspace(0, 1, num_flowX_points), flowX, linspace(0, 1, num_x_points), 'linear');
    
    num_y_points = length(y);
    num_flowY_points = length(flowY);
    flow_Yinterpolated = interp1(linspace(0, 1, num_flowY_points), flowY, linspace(0, 1, num_y_points), 'linear');

    % Use interpolated flow values with original positions
    U = griddata(x, y, flow_Xinterpolated, Xq, Yq);
    V = griddata(x, y, flow_Yinterpolated, Xq, Yq);
    
    % Option 2 (alternative): Use a subset of position data that matches flow data
    % x_subset = x(round(linspace(1, length(x), length(flowX))));
    % y_subset = y(round(linspace(1, length(y), length(flowY))));
    % U = griddata(x_subset, y_subset, flowX, Xq, Yq);
    % V = griddata(x_subset, y_subset, flowY, Xq, Yq);
end

% Create starting points for streamlines
[startX, startY] = meshgrid(linspace(x(30), max(x), 50), linspace(y(30), max(y), 50));

% Plot Data
quiver3(x', y', zeros(size(x')), flow_Xinterpolated, flow_Yinterpolated, zeros(size(flow_Xinterpolated)))
quiver(x', y', flow_Xinterpolated, flow_Yinterpolated)
axis equal
strmplt = streamline(Xq, Yq, U, V, startX, startY);
strmplt

%MATLAB script to read a CSV file and plot 2D position data with a heatmap
%based on actual sensor magnitude.
