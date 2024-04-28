clc;clear;close all; format default;

K = 4;
Kr = 5;

% K= b;


sim("ex3_6_hw11_me561_simu_model.slx")

A = 0.9213;
B = 0.0787;
C = 1;

sim("ex3_6_2_state_space_hw11_me561_simu_model.slx")

Kr = 1.25;


sim("ex3_8_hw11_me561_simu_model.slx")
