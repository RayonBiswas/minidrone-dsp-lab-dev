%% Route from the phone -> mini-drone waypoints
% 1. Phone: Navigate tab (ESP32 via Bluetooth or Simulate ESP32) shows the map, START, GOAL and route.
% 2. Laptop: same page, Phone -> Laptop link, so the laptop mirrors the phone's map and route.
% 3. Either device: "Route -> CSV (for the drone)" saves drone_route.csv (corner points of the A* route).
% 4. Put drone_route.csv next to this script (or in Downloads) and run it.
%
% Frame: the drone starts on START facing up the map. x_fwd = forward, y_right = right, in metres,
% the same axes as the Simulink minidrone's NED frame (z is down, so a flight height h is z = -h).
% Runs on plain MATLAB.
clc; clear; close all;

fileName  = 'drone_route.csv';
height    = 1.1;      % m, flight height
spacing   = 0.20;     % m between waypoints after densifying (0 = corners only)

%% 1. Load
if ~isfile(fileName)
    dl = fullfile(getenv('USERPROFILE'), 'Downloads');                % Windows Downloads folder
    L = [dir('drone_route*.csv'); dir(fullfile(dl, 'drone_route*.csv'))];
    if isempty(L), error('drone_route.csv not found. Tap "Route -> CSV" in the Navigate tab first.'); end
    [~, iNew] = max([L.datenum]); fileName = fullfile(L(iNew).folder, L(iNew).name);
end
T = readmatrix(fileName, 'NumHeaderLines', 4);    % 3 comment lines + header
corners = T(:, 4:5);                                % [x_fwd_m, y_right_m]
fprintf('Loaded %s: %d corner points\n', fileName, size(corners, 1));

%% 2. Densify so the position controller gets evenly spaced targets
if spacing > 0
    wp = corners(1, :);
    for k = 2:size(corners, 1)
        a = corners(k-1, :); b = corners(k, :); n = max(1, ceil(norm(b - a) / spacing));
        wp = [wp; a + (1:n)' / n .* (b - a)]; %#ok<AGROW>
    end
else
    wp = corners;
end
waypoints = [wp, -height * ones(size(wp, 1), 1)];  % [x y z] NED, z = -height
yaw = [atan2(diff(wp(:,2)), diff(wp(:,1))); 0];     % heading along each leg (rad)
yaw(end) = yaw(max(1, end-1));

fprintf('%d waypoints, path length %.2f m\n', size(waypoints, 1), sum(vecnorm(diff(wp), 2, 2)));
disp(array2table([waypoints, rad2deg(yaw)], 'VariableNames', {'x_fwd_m', 'y_right_m', 'z_m', 'yaw_deg'}));

%% 3. Plot (seen from above: forward is up, right is right)
figure('Name', 'Drone route', 'Color', 'w');
if exist('theme', 'file') || exist('theme', 'builtin'), try, theme(gcf, 'light'); catch, end, end
plot(wp(:,2), wp(:,1), '.-', 'Color', [0.3 0.5 1], 'MarkerSize', 10); hold on;
plot(corners(:,2), corners(:,1), 'ko', 'MarkerSize', 8, 'LineWidth', 1.5);
plot(0, 0, 'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g'); plot(wp(end,2), wp(end,1), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'y');
axis equal; grid on; xlabel('y right (m)'); ylabel('x forward (m)');
legend('waypoints', 'route corners', 'START', 'GOAL', 'Location', 'best'); title('Route from the phone, drone frame');

%% 4. Use in Simulink
% waypoints (N x 3) and yaw (N x 1) are now in the workspace. In the parrotMinidroneCompetition
% model, feed them to the Path Planning subsystem, e.g. a MATLAB Function block that steps to the
% next row once the drone is within ~0.1 m of the current one, or a From Workspace block:
%   route = timeseries(waypoints, (0:size(waypoints,1)-1)' * 1.0);   % one waypoint per second
save('drone_waypoints.mat', 'waypoints', 'yaw', 'corners');
fprintf('Saved drone_waypoints.mat (waypoints, yaw, corners).\n');
