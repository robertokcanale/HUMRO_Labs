function f = periodic_function(X_star)
% minimization function
f = Periodic(X_star) - X_star;
end