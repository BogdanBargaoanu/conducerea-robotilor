A = [zeros(2) eye(2); zeros(2) zeros(2)];
B = [zeros(2); eye(2)];
K = place(A, B, [-5 -5 -4 -4]);
Ki = 0.5;