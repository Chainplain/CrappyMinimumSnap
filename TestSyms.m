syms c_1 [1,7];
syms c_2 [1,7];
syms c_3 [1,7];
syms c_4 [1,7];
syms t;

% global sigma_1(t);
% global sigma_2(t);
% global sigma_3(t);
% global sigma_4(t);

sigma1(t) = poly2sym(c_1, t);
sigma2(t) = poly2sym(c_2, t);
sigma3(t) = poly2sym(c_3, t);
sigma4(t) = poly2sym(c_4, t);

mu_p = 1;
mu_psi = 1;
T = 2;




d4sigma1(t) = diff(sigma1(t),t,4);
d4sigma2(t) = diff(sigma2(t),t,4);
d4sigma3(t) = diff(sigma3(t),t,4);
d4sigma4(t) = diff(sigma4(t),t,4);

sumSquare(t) = mu_p * d4sigma1(t) ^ 2 + ...
                mu_p * d4sigma2(t) ^ 2 + ...
                mu_p * d4sigma3(t) ^ 2 + ...
                mu_psi * d4sigma4(t) ^ 2; 
intSumSquare = int (sumSquare(t), t, 0, T);
symbols = num2cell([c_1,c_2,c_3,c_4]);
coef = num2cell(ones(1, 7*4));

subs(intSumSquare, symbols, coef)
subs(sigma1, symbols, coef)