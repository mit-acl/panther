function untitled3() %#codegen

x=0.1:0.1:0.9; %0.1:0.1:0.9;tl
y=x.^7+sin(x*100);%zeros(size(x))

knots=aptknt(x,4); %Acceptable knot sequence

sp = spap2(knots,4,x,y); %sp.coefs are the control points;

sp=fn2fm(sp,'pp'); 

return;