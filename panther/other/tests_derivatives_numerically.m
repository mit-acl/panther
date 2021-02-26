%Import data from rosbag first

close all; clc;
time=secs+nsecs*1e-9;
dc=0.01;

p=pos_z;
v=v_z;
a=a_z;
j=j_z;

plot(time,p); hold on;

plot(time,v)
plot(time(2:end), diff(p)/dc, '--')

plot(time,a)
plot(time(3:end), diff(diff(p))/(dc^2), '--')


plot(time,j)
plot(time(4:end), diff(diff(diff(p)))/(dc^3), '--')