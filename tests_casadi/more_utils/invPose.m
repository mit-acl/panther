function result=invPose(T)

assert(size(T,1)==4 && size(T,2)==4)

R=T(1:3,1:3);
t=T(1:3,4);

result=[R' -R'*t; zeros(1,3) 1];

end