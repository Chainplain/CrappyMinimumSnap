syms x a b c
f(x,a) = a * x^2;
diff(f(x,a),x,1)

coefficients = ones(1,7);
g(x) = poly2sym(coefficients, x);

int(g(x),x)
% How to use
% subs(g(1), [a, b, c], [2, 3, 4])