f = @(x,y) x.*exp(-x.^2-y.^2)+(x.^2+y.^2)/20;
g = @(x,y) x.*y/2+(x+2).^2+(y-2).^2/2-3;

options = optimoptions('fmincon','Algorithm','sqp','Display','iter');
gfun = @(x) deal(g(x(1),x(2)),[]);
fun = @(x) f(x(1),x(2));

x0 = [-2 1];

[x,fval,exitflag,output] = fmincon(fun,x0,[],[],[],[],[],[],gfun,options);


fimplicit(g)
axis([-6 0 -1 7])
hold on
fcontour(f)
plot(x(1), x(2),'ro');
legend('constraint','f contours','minimum');
hold off