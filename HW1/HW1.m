%   ENAE4880        %
% Romeo Perlstein   %
%      HW1          %
% I thought the only lonely place, was on the moon!

%% Q1
%%% 1 
% Given the graph in fig 1., we can find the degree matrix as the
% following:
degree_mat = [
    2 0 0 0 0;
    0 2 0 0 0;
    0 0 2 0 0;
    0 0 0 2 0;
    0 0 0 0 2]
adjacency_mat = [
    0 1 0 0 1;
    1 0 1 0 0;
    0 1 0 1 0;
    0 0 1 0 1;
    1 0 0 1 0]
lapacian_mat = degree_mat - adjacency_mat

[V, D, W] = eig(lapacian_mat)

x0 = transpose([-2 4 -1 7 5])

xdot0 = -lapacian_mat*x0

c = 0;
for i=1:length(x0)
    c = c + (V(1,i)*x0(i));
end
c

tau = 1/D(2,2)