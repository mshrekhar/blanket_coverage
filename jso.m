function [cov, iter] = jso(n) 
grid_size = 10; 
rows = grid_size; 
cols = grid_size; 
gamma = 0.1; % motion coefficient  
beta = 3; % distribution coefficient  
r_s = 1; % sensing radius 
num_robots = n; 
r_c = 10; % communication radius 
swarm = []; 
side_arr=[]; 
for i = 1:(grid_size-1)  
side_arr(end+1) = i; 
end 
robots = randi([1, 9], num_robots, 2); % Assigning random positions to robots 

 
initial = robots; 
left_bottom = [0,0]; 
left_top = [0, cols]; 
right_bottom = [rows, 0]; 
right_top = [rows, cols]; 
 
a = 2:grid_size; 
[X, Y] = meshgrid(a, 0); 
bottom_edge = [X(:), Y(:)]; 
[X, Y] = meshgrid(0, a); 
left_edge = [X(:), Y(:)]; 
[X, Y] = meshgrid(a, grid_size); 
top_edge = [X(:), Y(:)]; 
[X, Y] = meshgrid(grid_size, a); 
right_edge = [X(:), Y(:)]; 
 
total_coverage_list = zeros(1,200); 
t = 0; 
while t < 200 %No.of interations 
    t = t+1; 
    total_coverage_area = 0; 
    c_iterat = abs((1 - (t/200))*2*(rand() - 1));  
    for i = 1:num_robots 
        reward = 0; 
        reward_arr=[]; 
        swarm = []; 
        pos = robots(i, :); 
        if ismember(left_bottom, pos) 
            R = 10; 
        elseif ismember(right_bottom, pos) 
            R = 10; 
        elseif ismember(right_top, pos) 
            R = 10;  
        elseif ismember(left_top, pos) 
            R = 10; 
        elseif any(bottom_edge == pos)  
            R = 20; 
        elseif any(left_edge == pos) 
            R = 20; 
        elseif any(right_edge == pos) 
            R = 20; 
        elseif any(top_edge == pos) 
            R = 20; 
        else 
            R = 40; 
        end 
        for j = i+1:num_robots 
            dist_r = 0; 
            if j ~= i 
            dist = norm(robots(i, :) - robots(j, :)); 
                if dist <=r_c 
                    swarm = [swarm; robots(j, 1), robots(j, 2)]; 
                    if dist > 1  
                        dist_r = dist_r + 0.2*(r_c - dist); 
                    else 
                        dist_r = -10 + dist_r; 
                    end 
                    reward = R + dist_r; 
                    reward_arr(end + 1) = reward; 
 
                end 
            end 
        end 
 
    [max_val, max_iter] = max(reward_arr); 
    index = max_iter; 
    mean_loc = mean(swarm); 
    count = 0; 
    for j = i + 1:num_robots 
        if j~=i 
            dist = norm(robots(i, :) - robots(j, :)); 
            if dist <=r_c 
                 count = count + 1; 
            end 
        end 
        end 
        if count == 0 
            robots(i, :) = robots(i, :); 
        else 
             
            if c_iterat >= 0.2 
                if rand()>(1-c_iterat) % for moving about own position 
                    robots(i,:) = robots(i,:) + gamma*rand()*(grid_size-0); 
                    if robots(i,1)<0  
                        robots(i,:) = robots(i,:) - gamma*rand()*(grid_size-0); 
                    elseif robots(i,1)>grid_size  
                        robots(i,:) = robots(i,:) - gamma*rand()*(grid_size-0); 
                    elseif robots(i,2)<0 
                        robots(i,:) = robots(i,:) - gamma*rand()*(grid_size-0); 
                    elseif robots(i,2)>grid_size 
                        robots(i,:) = robots(i,:) - gamma*rand()*(grid_size-0); 
                    else  
                        robots(i, :) = robots(i,:); 
                    end 
                     
                else 
                    indices = size(swarm, 1); 
                    randomIndex = randi(indices); % Generate a random index  
                    randomPosition = swarm(randomIndex, :); 
                    rewardIndex = randomIndex; 
                    r_i = reward_arr(1); 
                    r_j = reward_arr(rewardIndex); 
                    %Updation rule for moving towards random robot 
                    if r_i >= r_j 
                         dir = randomPosition - robots(i, :); 
                    else 
                         dir = robots(i, :) - randomPosition; 
                    end 
                    step = rand().*dir; 
                    robots(i, :) = robots(i, :) + step; 
                    if robots(i,1)<0  
                        robots(i,:) = robots(i,:) - step; 
                    elseif robots(i,1)>grid_size  
                        robots(i,:) = robots(i,:) - step; 
                    elseif robots(i,2)<0 
                        robots(i,:) = robots(i,:) - step; 
                    elseif robots(i,2)>grid_size 
                        robots(i,:) = robots(i,:) - step; 
                    else  
 
                        robots(i, :) = robots(i,:); 
                    end 
                end 
            else 
            % Updation rule for exploitation 
            trend = rand().*(swarm(index, :)-(beta.*rand().*mean_loc)); 
            robots(i, :) = robots(i, :) + trend; 
            if robots(i,1)<0  
                   robots(i,:) = robots(i,:) - trend; 
            elseif robots(i,1)>grid_size  
                   robots(i,:) = robots(i,:) - trend; 
            elseif robots(i,2)<0 
                   robots(i,:) = robots(i,:) - trend; 
            elseif robots(i,2)>grid_size 
                   robots(i,:) = robots(i,:) - trend; 
            else  
                   robots(i, :) = robots(i,:); 
            end 
            end 
        end 
        robots(i,1) = int32(robots(i, 1)); 
        robots(i,2) = int32(robots(i, 2)); 
     if robots(i,1) - r_s < 0 || robots(i,1) + r_s > grid_size || robots(i,2) - r_s < 0 || robots(i,2) + r_s > grid_size 
        % Calculate the intersection area with the area of interest 
        x = robots(i,1); 
        y = robots(i, 2); 
        if (x == 0 || x == grid_size) && (y == 0 || y== grid_size) 
            intersection_area = pi * (r_s^2 - (r_s^2)/4); 
            coverage_area = pi * r_s^2 - intersection_area; 
        elseif x == 0 || x == grid_size && ismember(y, side_arr) 
            intersection_area = pi * r_s^2/2; 
            coverage_area = pi * r_s^2 - intersection_area; 
        elseif y == 0 || y ==grid_size && ismember(x, side_arr) 
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
    % calculation of overlapped area 
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
    total_coverage_percentage = (final / (grid_size*grid_size)) * 100; 
    total_coverage_list(t)=total_coverage_percentage; 
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