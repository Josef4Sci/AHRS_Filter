function f=RastriginsFunction(x, y, ampl)
 f =20 + x.^2 + y.^2 - ampl*(cos(2*pi*x) + cos(2*pi*y));
end
