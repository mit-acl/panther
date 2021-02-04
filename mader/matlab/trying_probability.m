clc; clear;
mu=[0.2, 0.3, 0.4]'; std_deviation=[5.0, 3.0, 1.0];
Sigma=diag(std_deviation.^2);
a=[0.1, 0.6, 0.3]'; b=[0.3, 0.9, 2.3]';
mvncdf(b, mu, Sigma) - mvncdf(a, mu, Sigma)

prob_less_a=1.0;
for i=1:numel(a)
    prob_less_a=prob_less_a*normcdf(a(i), mu(i), std_deviation(i));
end

prob_less_b=1.0;
for i=1:numel(b)
    prob_less_b=prob_less_b*normcdf(b(i), mu(i), std_deviation(i));
end

prob_less_b-prob_less_a
% normcdf(b(1), mu(1), Sigma(1,1))*normcdf(b(2), mu(2), Sigma(2,2))*normcdf(b(3), mu(3), Sigma(3,3)) -...
%      normcdf(a(1), mu(1), Sigma(1,1))*normcdf(a(2), mu(2), Sigma(2,2))*normcdf(a(3), mu(3), Sigma(3,3))