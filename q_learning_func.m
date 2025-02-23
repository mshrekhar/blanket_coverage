function [cov, iter] = q_learning_func(n)
    g = 10;
    % Grid size
    rows = g + 1;
    cols = g + 1;
    % List of coordinates
    coordinates = zeros((rows-1)*(cols-1), 2);
    k = 1;
    r_c = 10;
    
    % Generate coordinates
    for r = 1:rows
        for c = 1:cols
            coordinates(k, :) = [r, c];
            k = k + 1;
        end
    end
    
    numStates = numel(coordinates);
    numActions = 4;
    left_bottom = [1,1];
    left_top = [1, cols];
    right_bottom = [rows, 1];
    right_top = [rows, cols];
    a = 2:g;
    [X, Y] = meshgrid(a, 1);
    bottom_edge = [X(:), Y(:)];
    [X, Y] = meshgrid(1, a);
    left_edge = [X(:), Y(:)];
    [X, Y] = meshgrid(a, (g+1));
    top_edge = [X(:), Y(:)];
    [X, Y] = meshgrid((g+1), a);
    right_edge = [X(:), Y(:)];
    actions = [1, 2, 3, 4]; % 1 - forward; 2 - back; 3 - right; 4 - left
    actions_lb = [1, 3];
    actions_lt = [2, 3];
    actions_rb = [1, 4];
    actions_rt = [2, 4];
    actions_be = [1, 3, 4];
    actions_te = [2, 3, 4];
    actions_le = [1, 2, 3];
    actions_re = [1, 2, 4];
    alpha = 0.8; % learning rate
    gamma = 0.9; % discount factor
    grid_size = g;
    num_robots = n;
    epsilon = 0.2;
    robots = randi([2, 10], num_robots, 2); % Assigning random positions to robots
    initial = robots; % Storing the initial positions of the robots
    Q = zeros(rows, cols, numActions); % creating a Q table
    total_coverage_area = 0;
    total_coverage_list = zeros(1,200);
    
    side_arr = [];
    for i = 2:grid_size
        side_arr(end+1) = i; % For creating an array of possible values for positions on the edge of the grid
    end
    
    r_s = 1; % sensing radius for single robot
    t = 0;
    
    while t < 200 % No. of iterations
        t = t + 1;
        total_coverage_area = 0;
        
        for i = 1:num_robots
            state = robots(i,:);
            dist_r = 0;
            tot_reward = 0;
            
            if ismember(left_bottom, state)
                if rand() < epsilon % for selecting a random action
                    randomIndex = randi(numel(actions_lb));
                    action = actions_lb(randomIndex);
                else
                    action = max(Q(state(1), state(2), :)); % Exploit (best action)
                end
                R = 10;
            elseif ismember(right_bottom, state)
                if rand() < epsilon
                    randomIndex = randi(numel(actions_rb));
                    action = actions_rb(randomIndex);
                else
                    action = max(Q(state(1), state(2), :));
                end
                R = 10;
            elseif ismember(right_top, state)
                if rand() < epsilon
                    randomIndex = randi(numel(actions_rt));
                    action = actions_rt(randomIndex);
                else
                    action = max(Q(state(1), state(2), :));
                end
                R = 10;
            elseif ismember(left_top, state)
                if rand() < epsilon
                    randomIndex = randi(numel(actions_lt));
                    action = actions_lt(randomIndex);
                else
                    action = max(Q(state(1), state(2), :));
                end
                R = 10;
            elseif any(bottom_edge == state)
                if rand() < epsilon
                    randomIndex = randi(numel(actions_be));
                    action = actions_be(randomIndex);
                else
                    action = max(Q(state(1), state(2), :));
                end
                R = 20;
            elseif any(left_edge == state)
                if rand() < epsilon
                    randomIndex = randi(numel(actions_le));
                    action = actions_le(randomIndex);
                else
                    action = max(Q(state(1), state(2), :));
                end
                R = 20;
            elseif any(right_edge == state)
                if rand() < epsilon
                    randomIndex = randi(numel(actions_re));
                    action = actions_re(randomIndex);
                else
                    action = max(Q(state(1), state(2), :));
                end
                R = 20;
            elseif any(top_edge == state)
                if rand() < epsilon
                    randomIndex = randi(numel(actions_te));
                    action = actions_te(randomIndex);
                else
                    action = max(Q(state(1), state(2), :));
                end
                disp(action);
                R = 20;
            else
                if rand() < epsilon
                    randomIndex = randi(numel(actions));
                    action = actions(randomIndex);
                else
                    action = max(Q(state(1), state(2), :));
                end
                R = 40;
            end
            
            for j = 1:num_robots
                if j ~= i % skip the current robot
                    r = robots(j,:) - robots(i,:);
                    d = norm(r);
                    if d <= r_c
                        if d > 1
                            dist_r = dist_r + 0.2*(r_c - d);
                        else
                            dist_r = -(n/2) + dist_r;
                        end
                    else
                        dist_r = dist_r;
                    end
                end
            end
            
            reward = R + dist_r;
            
            if action == 1
                if state(2) + 1 >= cols
                    nextState = state; % Stay in the same state if
                else
                    nextState = [state(1), state(2) + 1];
                end
                robots(i,:) = nextState;
                Q(state(1), state(2), action) = (1 - alpha) * Q(state(1), state(2), action) + alpha * (reward + gamma * max(Q(nextState(1), nextState(2), :))); % Q updation
            elseif action == 2
                if state(2) - 1 <= 1
                    nextState = state;
                else
                    nextState = [state(1), state(2) - 1];
                end
                robots(i,:) = nextState;
                Q(state(1), state(2), action) = (1 - alpha) * Q(state(1), state(2), action) + alpha * (reward + gamma * max(Q(nextState(1), nextState(2), :)));
            elseif action == 3
                if state(1) + 1 >= rows
                    nextState = state;
                else
                    nextState = [state(1) + 1, state(2)];
                end
                robots(i,:) = nextState;
                Q(state(1), state(2), action) = (1 - alpha) * Q(state(1), state(2), action) + alpha * (reward + gamma * max(Q(nextState(1), nextState(2), :)));
            elseif action == 4
                if state(1) - 1 <= 1
                    nextState = state;
                else
                    nextState = [state(1) - 1, state(2)];
                end
                robots(i,:) = nextState;
                Q(state(1), state(2), action) = (1 - alpha) * Q(state(1), state(2), action) + alpha * (reward + gamma * max(Q(nextState(1), nextState(2), :)));
            else
                Q(state(1), state(2), :) = alpha * reward;
            end
            
            if robots(i,1) - r_s < 1 || robots(i,1) + r_s > (grid_size + 1) || robots(i,2) - r_s < 1 || robots(i,2) + r_s > (grid_size + 1)
                % Calculate the intersection area with the area of interest
                x = robots(i,1);
                y = robots(i, 2);
                if (x == 1 || x == (grid_size + 1)) && (y == 1 || y == (grid_size + 1))
                    intersection_area = pi * (r_s^2 - (r_s^2)/4);
                    coverage_area = pi * r_s^2 - intersection_area;
                elseif x == 1 || x == (grid_size + 1) && ismember(y, side_arr)
                    intersection_area = pi * r_s^2/2;
                    coverage_area = pi * r_s^2 - intersection_area;
                elseif y == 1 || y == (grid_size + 1) && ismember(x, side_arr)
                    intersection_area = pi * r_s^2/2;
                    coverage_area = pi * r_s^2 - intersection_area;
                else
                    coverage_area = pi * r_s^2;
                end
            else
                coverage_area = pi * r_s^2;
            end
            
            % Add the coverage area of the current sensor node to the total coverage area
            total_coverage_area = total_coverage_area + coverage_area;
        end
        
        overlapArea = 0;
        for i = 1:num_robots
            for j = i+1:num_robots
                distance = norm(robots(i, :) - robots(j, :));
                if distance <= 2*r_s
                    % Circles overlap, calculate area of overlap
                    if r_s >= distance + r_s
                        overlapArea = pi * r_s^2 + overlapArea;
                    else
                        alpha1 = acos((2 * r_s * distance) / (2 * r_s * distance));
                        alpha2 = acos((2 * r_s * distance) / (2 * r_s * distance));
                        overlapArea = r_s^2 * alpha1 + r_s^2 * alpha2 - r_s * distance * sin(alpha1) + overlapArea;
                    end
                end
            end
        end
        
        final = total_coverage_area - overlapArea;
        total_coverage_percentage = (final / (g*g)) * 100;
        total_coverage_list(t) = total_coverage_percentage;
    end
    
    consecutiveCount = 0;
    % Flag to indicate if a value is repeated 20 times or more consecutively
    isRepeated = false;
    % Index of the first occurrence of the repeated value
    firstIndex = 1;
    
    % Iterate through the array
    for i = 1:numel(total_coverage_list)
        if i == 1 || total_coverage_list(i) ~= total_coverage_list(i-1)
            % Reset the consecutive count for a new value
            consecutiveCount = 1;
        else
            % Increment the consecutive count
            consecutiveCount = consecutiveCount + 1;
            % Check if consecutive count reaches 20 or more
            if consecutiveCount >= 30 && ~isRepeated
                isRepeated = true;
                firstIndex = i - consecutiveCount + 1;
            end
        end
    end
    
    cov = total_coverage_list(firstIndex);
    iter = firstIndex;
end
