function robotArm3DOF_MultiGoalPath()
    clc; close all; % Clear command window and close all figures

    % Create a figure window for visualization
    f = figure('Name','3-Link Arm Path Tracker','NumberTitle','off','Position',[100 100 800 600]);

    % Create axes inside the figure for plotting the robot arm
    ax = axes('Parent', f, 'Position', [0.3 0.1 0.65 0.8]);
    axis equal; xlim([-4 4]); ylim([-4 4]); grid on; hold on; % Set hold on for persistent plotting
    title('Click to set goals, then press Start');

    % UI Controls for Link Length Inputs
    uicontrol('Style','text','Position',[1 518 100 20],'String','L1:');
    l1Box = uicontrol('Style','edit','Position',[80 520 50 20],'String','1'); % Default L1 = 1
    uicontrol('Style','text','Position',[1 488 100 20],'String','L2:');
    l2Box = uicontrol('Style','edit','Position',[80 490 50 20],'String','1'); % Default L2 = 1
    uicontrol('Style','text','Position',[1 458 100 20],'String','L3:');
    l3Box = uicontrol('Style','edit','Position',[80 460 50 20],'String','1'); % Default L3 = 1

    % Threshold for goal distance
    uicontrol('Style','text','Position',[1 428 100 20],'String','Threshold:');
    thresholdBox = uicontrol('Style','edit','Position',[80 430 50 20],'String','0.05'); % Default threshold = 0.05

    % Obstacles Position & Size
    uicontrol('Style','text','Position',[1 398 100 20],'String','Obstacles:');
    obstaclesBox = uicontrol('Style','edit','Position',[80 400 150 20],...
        'String','[1.5, 1.5, 0.3; -2.0, 0.5, 0.4]'); % Default position

    % Step size input
    uicontrol('Style','text','Position',[1 368 100 20],'String','Alpha:');
    alphaBox = uicontrol('Style','edit','Position',[80 370 50 20],'String','0.2'); % Default step = 0.2

    % Epochs input (number of trials before moving to the next goal)
    uicontrol('Style','text','Position',[1 338 100 20],'String','Epochs:');
    epochsBox = uicontrol('Style','edit','Position',[80 340 50 20],'String','700'); % Default N = 700

    % Start Button to begin movement to goals
    uicontrol('Style','pushbutton','String','Start','Position',[60 280 120 50],...
        'Callback', @startCallback);

    % Reset Button
    uicontrol('Style','pushbutton','String','Reset','Position',[60 220 120 50],...
        'Callback', @resetCallback);

    % Start with random arm position
    theta = rand(1,3)*2*pi; % Random initial joint angles
    goals = []; % Empty list to store goal points

    % Allow user to click in the figure to add goals
    set(f, 'WindowButtonDownFcn', @clickCallback); 
    
    % --- Nested Functions ---

    % On mouse click we do the following
    function clickCallback(~, ~)
        cp = get(ax, 'CurrentPoint'); % Get coordinates of mouse click
        g = cp(1,1:2); % Extract x and y
        goals = [goals; g]; % Add new click to goals list
        plot(ax, g(1), g(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Mark the goal on the plot
        title(ax, sprintf('Added goal at (%.2f, %.2f). Total: %d', g(1), g(2), size(goals,1)));
    end

    function startCallback(~, ~)
        % If pressed start without any goal set
        if isempty(goals)
            title(ax, 'Please click to set goals first!');
            return;
        end

        % Get info from user inputs
        L1 = str2double(get(l1Box, 'String'));
        L2 = str2double(get(l2Box, 'String'));
        L3 = str2double(get(l3Box, 'String'));
        goalThreshold = str2double(get(thresholdBox, 'String'));
        obstacles_data = str2num(get(obstaclesBox, 'String')); % Rename to avoid conflict with initial empty 'obstacles'
        alpha = str2double(get(alphaBox, 'String'));
        epochs = str2double(get(epochsBox, 'String'));

        % Clear previous arm and temporary goal markers from the axes for a new simulation run
        % This also clears any previously set green goals from previous runs.
        cla(ax); 
        axis equal; xlim([-4 4]); ylim([-4 4]); grid on; hold on; % Re-establish axes properties and hold

        % Plot all obstacles ONCE at the start of the simulation
        for i = 1:size(obstacles_data,1)
            rectangle('Parent', ax, 'Position',[obstacles_data(i,1)-obstacles_data(i,3), obstacles_data(i,2)-obstacles_data(i,3), ...
                2*obstacles_data(i,3), 2*obstacles_data(i,3)], 'Curvature',[1,1],'FaceColor','k');
        end

        % Redraw the initially clicked goal markers (the red 'x's) that were set by clicks
        for i = 1:size(goals, 1)
            plot(ax, goals(i,1), goals(i,2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        end

        % Iterate through all goals
        for k = 1:size(goals,1)
            goal = goals(k,:);
            title(ax, sprintf('Moving to Goal %d of %d...', k, size(goals,1)));
            % Move arm toward the goal using local optimization
            theta = moveToGoal(theta, goal, L1, L2, L3, obstacles_data, ax, goalThreshold, alpha, epochs);
        end

        title(ax, 'Finished all goals!');
    end

    % Reset Callback Function
    function resetCallback(~, ~)
        cla(ax); % Clear all elements from the axes
        axis equal; xlim([-4 4]); ylim([-4 4]); grid on; hold on; % Re-establish axes properties
        title('Click to set goals, then press Start'); % Reset title
        goals = []; % Clear the list of goals
        theta = rand(1,3)*2*pi; % Reset arm to a new random initial position
    end

end % End of main function

% --- Helper Functions (defined outside the main function for clarity) ---

function theta = moveToGoal(theta, goal, L1, L2, L3, obstacles, ax, goalThreshold, alpha, epochs)
    % Initialize handles for dynamic plots. These will be created ONCE and then updated.
    armPlotHandle = [];
    currentGoalMarkerHandle = []; % This will be the red 'x' for the current goal

    % Ensure hold is on for the axes. This is set in startCallback too, but good to ensure.
    hold(ax, 'on'); 

    for epoch = 1:epochs
        % Forward kinematics for current theta
        [x, y, joints] = forwardKinematics(theta, L1, L2, L3);

        % Distance to goal
        dist = norm([x - goal(1), y - goal(2)]);

        % --- Optimized Drawing & Animation ---

        % Create the arm plot object only ONCE, then update its data
        if isempty(armPlotHandle) || ~isvalid(armPlotHandle)
            armPlotHandle = plot(ax, joints(:,1), joints(:,2), '-o', 'LineWidth', 2); % Create
        else
            set(armPlotHandle, 'XData', joints(:,1), 'YData', joints(:,2)); % Update
        end

        % Create the current red goal marker only ONCE, then update its data
        if isempty(currentGoalMarkerHandle) || ~isvalid(currentGoalMarkerHandle)
            currentGoalMarkerHandle = plot(ax, goal(1), goal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Create
        else
            set(currentGoalMarkerHandle, 'XData', goal(1), 'YData', goal(2)); % Update
        end
        
        % Force MATLAB to update the figure immediately
        %drawnow;
        pause(0.05); % best delay for smooth & continuous movement

        % Stop if close enough
        if dist < goalThreshold
            % Delete the temporary red marker for the current goal
            if ~isempty(currentGoalMarkerHandle) && isvalid(currentGoalMarkerHandle)
                delete(currentGoalMarkerHandle);
            end
            % Delete the arm plot handle when the goal is reached
            if ~isempty(armPlotHandle) && isvalid(armPlotHandle)
                delete(armPlotHandle);
            end
            % Plot the goal in green, and this time, it will persist
            plot(ax, goal(1), goal(2), 'gx', 'MarkerSize', 12, 'LineWidth', 3); 
            break; % Exit the loop for the current goal
        end

        % Try a small random update in joint angles
        theta_new = theta + (rand(1,3)-0.5)*alpha;
        [x_new, y_new, joints_new] = forwardKinematics(theta_new, L1, L2, L3);
        new_dist = norm([x_new - goal(1), y_new - goal(2)]);

        % Accept new configuration if it reduces distance and avoids collision
        if new_dist < dist && ~checkCollision(joints_new, obstacles)
            theta = theta_new;
        end
    end
end

function [x, y, joints] = forwardKinematics(theta, L1, L2, L3)
    % Calculate joint positions using cumulative angles
    x1 = L1 * cos(theta(1));
    y1 = L1 * sin(theta(1));
    x2 = x1 + L2 * cos(theta(1)+theta(2));
    y2 = y1 + L2 * sin(theta(1)+theta(2));
    x3 = x2 + L3 * cos(theta(1)+theta(2)+theta(3));
    y3 = y2 + L3 * sin(theta(1)+theta(2)+theta(3));
    x = x3; y = y3; % End effector position (last joint position is the sum of previouses + itself
    joints = [0 0; x1 y1; x2 y2; x3 y3]; % Joint coordinates for plotting
end

function collided = checkCollision(joints, obstacles)
    collided = false;
    for i = 1:size(joints,1)-1 % Each pair of points define a segment >> N-1
        p1 = joints(i,:); % The position vector of joint i
        p2 = joints(i+1,:); % The position vector of joint i+1
        for j = 1:size(obstacles,1)
            center = obstacles(j,1:2); % center position of obstacle
            r = obstacles(j,3); % radius of obstacle
            if lineCircleIntersect(p1, p2, center, r)
                collided = true; % Stop checking once a collision is found
                return;
            end
        end
    end
end

function hit = lineCircleIntersect(p1, p2, center, r)
    % Check if a line segment from p1 to p2 intersects a circle
    d = p2 - p1;        % Line direction vector (Segment's direction)
    f = p1 - center;    % Vector from circle center to line start
    a = dot(d,d);       % Quadratic coefficient a
    b = 2 * dot(f,d);   % Quadratic coefficient b
    c = dot(f,f) - r^2; % Quadratic coefficient c
    % Check for real solutions >> there exist a point on the infinite line direction and the circle at the same time.
    discriminant = b^2 - 4*a*c;
    if discriminant < 0
        hit = false; % No intersection
    else
        discriminant = sqrt(discriminant);
        t1 = (-b - discriminant)/(2*a);
        t2 = (-b + discriminant)/(2*a);
        hit = (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1); % If t in [0,1], intersects segment
    end
end