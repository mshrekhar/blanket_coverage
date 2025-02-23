clc
clear all
close all

final_val1 = [];
final_val2 = [];
final_val3 = [];

for i = 2:40
    for j = 1:50
        [cov, iter] = vfa_code(i);
        x = [cov, iter];
        final_val1((end+1), :) = x;
    end
end

for i = 2:40
    for j = 1:50
        [cov, iter] = q_learning_func(i);
        x = [cov, iter];
        final_val2((end+1), :) = x;
    end
end

for i = 2:40
    for j = 1:50
        [cov, iter] = jso(i);
        x = [cov, iter];
        final_val3((end+1), :) = x;
    end
end

cov_arr1 = final_val1(:, 1);
iter_arr1 = final_val1(:, 2);
cov_arr1 = reshape(cov_arr1, 50, []);
mean_cov1 = mean(cov_arr1);
iter_arr1 = reshape(iter_arr1, 50, []);
mean_iter1 = int32(mean(iter_arr1));

cov_arr2 = final_val2(:, 1);
iter_arr2 = final_val2(:, 2);
cov_arr2 = reshape(cov_arr2, 50, []);
mean_cov2 = mean(cov_arr2);
iter_arr2 = reshape(iter_arr2, 50, []);
mean_iter2 = int32(mean(iter_arr2));

cov_arr3 = final_val3(:, 1);
iter_arr3 = final_val3(:, 2);
cov_arr3 = reshape(cov_arr3, 50, []);
mean_cov3 = mean(cov_arr3);
iter_arr3 = reshape(iter_arr3, 50, []);
mean_iter3 = int32(mean(iter_arr3));

% Figure to depict coverage
figure;
hold on;
plot(2:40, mean_cov1, 'b-', 'LineWidth', 2);
plot(2:40, mean_cov2, 'r-', 'LineWidth', 2);
plot(2:40, mean_cov3, 'g-', 'LineWidth', 2);
% Add labels and legend
xlabel('No. of Robots');
ylabel('Coverage');
legend('VFA', 'Q-Learning', 'JSO');
% Set title and grid
title('Coverage Graphs');
grid on;
hold off;

% Figure to depict convergence iteration
figure;
hold on;
plot(2:40, mean_iter1, 'b-', 'LineWidth', 2);
plot(2:40, mean_iter2, 'r-', 'LineWidth', 2);
plot(2:40, mean_iter3, 'g-', 'LineWidth', 2);
% Add labels and legend
xlabel('No. of Robots');
ylabel('Convergence Iteration');
legend('VFA', 'Q-Learning', 'JSO');
% Set title and grid
title('Convergence Iteration Graphs');
grid on;
