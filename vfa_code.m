function [cov, iter] = vfa_code(n)
    % Define the workspace and robots
    g = 10;
    grid_size = g;
    resolution = 1; % resolution of the grid (in meters)
    num_robots = n;
    rows = g;
    cols = g;
    robots = randi([1, 9], num_robots, 2);  % Assigning random positions to robots
    initial = robots;  % Storing the initial positions of the robots
    total_coverage_area = 0;
    total_coverage_list = zeros(1,200);
    side_arr = [];
    for i = 1:(grid_size-1)
        side_arr(end+1) = i;  % For creating an array of possible values for positions on the edge of the grid
    end
    r_s = 1;  % sensing radius for single robot
    r_c = 10;  % communication radius for single robot

    % Define the parameters for the virtual forces
    d_th = 1.5;  % minimum distance to be maintained between robots
    maxForce = 0.1;  % Maximum force applied by virtual forces
    maxSpeed = 1;  % Maximum robot speed
    t = 0;

    while t < 200  % No. of iterations
        t = t + 1;
        total_coverage_area = 0;
        count = 0;
        for i = 1:num_robots
            for j = 1:num_robots
                if j ~= i
                    dist = norm(robots(i, :) - robots(j, :));
                    if dist <= r_c
                        count = count + 1;
                    end
                end
            end
        end
        wa = (d_th / r_c) * (count^(-1));  % Attractive force coefficient
        wr = count^(1);  % Repulsive force coefficient

        % Calculate the virtual forces for each robot
        for i = 1:num_robots
            force = [0, 0];
            for j = 1:num_robots
                if j ~= i  % skip the current robot
                    v1 = robots(i,:);
                    v2 = robots(j,:);
                    r = robots(j,:) - robots(i,:);
                    d = norm(r);  % distance between the robots
                    angle = atan2(v2(2) - v1(2), v2(1) - v1(1));  % angle between robots
                    if d < r_c
                        if d < d_th
                            angle = angle + pi;
                            % Apply repulsive force
                            [x, y] = pol2cart(angle, (wr / d));  % converting from polar to cartesian form
                            r_force = [x, y];
                            force = force - r_force;
                        else
                            % Apply attractive force
                            [x, y] = pol2cart(angle, (wa * (d - d_th)));  % converting from polar to cartesian form
                            a_force = [x, y];
                            force = force + a_force;
                        end
                    end
                end
            end
            if norm(force) > maxForce
                force = force / norm(force) * maxForce;  % ensuring that force does not exceed maxForce
            end
            robots(i,:) = robots(i,:) + maxSpeed * force;
            if robots(i,1) > 0 && robots(i,1) <= grid_size && robots(i,2) > 0 && robots(i,2) <= grid_size  % ensuring that coordinates are in the grid
                x = int32(round(robots(i,1) / resolution));
                y = int32(round(robots(i,2) / resolution));
                robots(i,1) = x;
                robots(i,2) = y;
            else
                robots(i,:) = robots(i,:) - maxSpeed * force;
                x = int32(round(robots(i,1) / resolution));
                y = int32(round(robots(i,2) / resolution));
                robots(i,1) = x;
                robots(i,2) = y;
            end

            % Adjust the coverage area if the circular sensor extends beyond the area of interest boundaries
            if robots(i,1) - r_s < 0 || robots(i,1) + r_s > grid_size || robots(i,2) - r_s < 0 || robots(i,2) + r_s > grid_size
                % Calculate the intersection area with the area of interest
                x = robots(i,1);
                y = robots(i, 2);
                if (x == 0 || x == grid_size) && (y == 0 || y == grid_size)  % if robot on corner point the coverage area is given by
                    intersection_area = pi * (r_s^2 - (r_s^2)/4);
                    coverage_area = pi * r_s^2 - intersection_area;
                elseif (x == 0 || x == grid_size) && ismember(y, side_arr)  % if robot on the left or right edge the coverage area is given by
                    intersection_area = pi * r_s^2/2;
                    coverage_area = pi * r_s^2 - intersection_area;
                elseif (y == 0 || y == grid_size) && ismember(x, side_arr)  % if robot on the top or bottom edge the coverage area is given by
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
        % To calculate total overlapped area for all the robots
        for i = 1:num_robots
            for j = i+1:num_robots
                distance = norm(robots(i, :) - robots(j, :));
                if distance <= d_th
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
