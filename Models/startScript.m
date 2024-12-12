

% Model Initialization
% Start Location [X Z Y]
dronestartlocation = [-1.5 0 4];

% Final Location [X Z Y]
helipadLocation = [14 0.29 4];

% Linear Velocity [Vx Vz Vy]
linearVelocity = [0.6 0.05 .2]; 

% Angular Velocity during Optical Flow navigation
% represented as [pitch yaw roll]
angularVelocity = [0 0 -0.01]; 

% X-coordinate to switch from Optical flow controller to PID Controller
switchingXLimit = 12.0;

% Angular Velocity during landing
angularVelocityforLanding = [0 0 0.01];

% Hover Height before landing
finalHoverHeight = 0.8;

% Threshold distance to execute the PID controller.
delta = 0.1;

% Sample Time
Ts = 0.1;

% Additional parameter checks and validation (optional)
if any(dronestartlocation < -20 | dronestartlocation > 20)
    error('Drone start location out of bounds. Ensure coordinates are within the arena.');
end

if any(helipadLocation < -20 | helipadLocation > 20)
    error('Helipad location out of bounds. Ensure coordinates are within the arena.');
end

% Display the initialized parameters for verification.
disp('Initialization complete with the following parameters:');
disp(['Drone Start Location: ', num2str(dronestartlocation)]);
disp(['Helipad Location: ', num2str(helipadLocation)]);
disp(['Linear Velocity: ', num2str(linearVelocity)]);
disp(['Angular Velocity (Navigation): ', num2str(angularVelocity)]);
disp(['Angular Velocity (Landing): ', num2str(angularVelocityforLanding)]);
disp(['Switching X Limit: ', num2str(switchingXLimit)]);
disp(['Final Hover Height: ', num2str(finalHoverHeight)]);
disp(['Delta (PID threshold): ', num2str(delta)]);
disp(['Sample Time (Ts): ', num2str(Ts)]);

% A* Algorithm Implementation

% Define a grid map with obstacles (you need to customize this based on your setup)
gridMap = zeros(100, 100); % Example 100x100 grid, 0 = free space, 1 = obstacle
obstacleCoordinates = [1, 3], [1, 4], [2, 1], [2, 7], [3, 4],
[4, 2], [4, 6], [4, 4], [4, 4], [10, 1],
[4, 7], [5, 3], [6, 6], [7, 2], [8, 5], [9, 4]
;
for i = 1:size(obstacleCoordinates, 1)
    gridMap(obstacleCoordinates(i, 1), obstacleCoordinates(i, 2)) = 1;
end

% Start and goal positions in grid coordinates (convert from real-world coordinates as needed)
startPos = [10, 10]; % Example starting position
goalPos = [90, 90]; % Example goal position

% Run A* algorithm
path = aStarAlgorithm(gridMap, startPos, goalPos);

% Display the path (optional)
disp('Calculated path:');
disp(path);

function path = aStarAlgorithm(gridMap, startPos, goalPos)
    % Implementation of the A* algorithm
    % (You can implement this using existing MATLAB functions or write your own)
    
    % Initialize open and closed lists
    openList = [];
    closedList = [];
    
    % Initialize starting node
    startNode = struct('position', startPos, 'cost', 0, 'heuristic', heuristic(startPos, goalPos), 'parent', []);
    openList = [openList; startNode];
    
    while ~isempty(openList)
        % Get the node with the lowest cost + heuristic
        [~, idx] = min([openList.cost] + [openList.heuristic]);
        currentNode = openList(idx);
        openList(idx) = [];
        
        % Check if we reached the goal
        if isequal(currentNode.position, goalPos)
            path = reconstructPath(currentNode);
            return;
        end
        
        % Add current node to closed list
        closedList = [closedList; currentNode];
        
        % Generate neighbors (implement this based on gridMap)
        neighbors = getNeighbors(gridMap, currentNode.position);
        
        for i = 1:size(neighbors, 1)
            neighborPos = neighbors(i, :);
            if isInList(closedList, neighborPos)
                continue;
            end
            if gridMap(neighborPos(1), neighborPos(2)) == 1 % Obstacle check
                continue;
            end
            
            % Calculate cost and heuristic
            newCost = currentNode.cost + 1; % Example cost, modify as needed
            heuristicValue = heuristic(neighborPos, goalPos);
            
            % Check if the neighbor is already in the open list
            if ~isInList(openList, neighborPos) || newCost < getNodeCost(openList, neighborPos)
                neighborNode = struct('position', neighborPos, 'cost', newCost, 'heuristic', heuristicValue, 'parent', currentNode);
                openList = [openList; neighborNode];
            end
        end
    end
    
    % No path found
    path = [];
end

function cost = heuristic(pos, goalPos)
    % Heuristic function (Manhattan distance)
    cost = abs(pos(1) - goalPos(1)) + abs(pos(2) - goalPos(2));
end

function neighbors = getNeighbors(gridMap, position)
    % Generate valid neighboring coordinates
    [rows, cols] = size(gridMap);
    x = position(1);
    y = position(2);
    neighbors = [];
    
    for dx = -1:1
        for dy = -1:1
            if dx == 0 && dy == 0
                continue;
            end
            newX = x + dx;
            newY = y + dy;
            if newX > 0 && newX <= rows && newY > 0 && newY <= cols
                neighbors = [neighbors; newX, newY];
            end
        end
    end
end

function isPresent = isInList(list, position)
    isPresent = any(arrayfun(@(x) isequal(x.position, position), list));
end

function cost = getNodeCost(list, position)
    idx = find(arrayfun(@(x) isequal(x.position, position), list));
    if ~isempty(idx)
        cost = list(idx).cost;
    else
        cost = Inf;
    end
end

function path = reconstructPath(node)
    path = [];
    while ~isempty(node)
        path = [node.position; path];
        node = node.parent;
    end
end



