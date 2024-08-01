close all
div=1;
ampl = 5.0;
multip= 4;
rf3 = @(x, y) RastriginsFunction(x/div, y/div, ampl);
fsurf(rf3,[-2 4],"ShowContours","on")
title("rastriginsfcn([x/10,y/10])")
xlabel("x")
ylabel("y")

figure Name 'rosen'
rb = @(x, y) Rosenb(x/div,y/div, 8, 1);
fsurf(rb,[-20 40],"ShowContours","on")
title("rosenb([x/10,y/10])")
xlabel("x")
ylabel("y")

figure Name 'comb'
comb =  @(x, y) (Rosenb(x/div,y/div, 8, 1)+multip*RastriginsFunction(x/div, y/div, ampl));
fsurf(comb,[-2 4],"ShowContours","on")
title("rosenb+rastriginsfcn([x/10,y/10])")
xlabel("x")
ylabel("y")

min_opt_bound = -1e6;
max_opt_bound = 1e6;
x = optimvar("x","LowerBound",min_opt_bound,"UpperBound",max_opt_bound);
y = optimvar("y","LowerBound",min_opt_bound,"UpperBound",max_opt_bound);

prob = optimproblem("Objective",(Rosenb(x/div,y/div, 8, 1)+multip*RastriginsFunction(x/div, y/div, ampl)));
options = optimoptions("ga","PlotFcn","gaplotbestf");
rng default % For reproducibility
[sol,fval] = solve(prob,"Solver","ga","Options",options)