clc;clear;close all; format default;
w= 21;
% Initialize an array to store values of m that satisfy the inequality
satisfying_m = [];

% Define the upper limit for m (you can adjust this as needed)
upper_limit = 100; % Change this value if you need a larger range

for m = -upper_limit:upper_limit
    % Calculate the value of f(m)
    f_m_pos = w + m * 21;
    f_m_neg = -w + m * 21;
    
    % Check if f(m) is less than 50
    if (f_m_pos< 50 && f_m_pos >0) || (f_m_neg< 50 && f_m_neg >0)
        % If it is, add m to the list of satisfying values
        satisfying_m = [satisfying_m, m];
        
        % Output the current value of m
        fprintf('m = %d, f(m) = %d\n', m, f_m_pos);
        fprintf('m = %d, f(m) = %d\n', m, f_m_neg);
    end
end

% Display the values of m that satisfy the inequality
disp('Values of m that satisfy the inequality:');
disp(satisfying_m);
