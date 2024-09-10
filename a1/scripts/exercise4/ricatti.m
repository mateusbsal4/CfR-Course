clear all
close all
clc


A = 1;
B = 1;
Q = 0.5;
R = 1;
E = 1;
S = 0;


[X, K, ~] = idare(A, B, Q, R, [], []);
alpha = -inv(B'*X*B + R)*(B'*X*A)