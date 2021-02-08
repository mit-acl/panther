clc; clear;close all;
formatSpec = '%f\n ---';
mumps = fscanf(fopen('mumps.txt','r'),formatSpec);
ma27 = fscanf(fopen('ma27.txt','r'),formatSpec);
ma57 = fscanf(fopen('ma57.txt','r'),formatSpec);
ma77 = fscanf(fopen('ma77.txt','r'),formatSpec);
ma86 = fscanf(fopen('ma86.txt','r'),formatSpec);
ma97 = fscanf(fopen('ma97.txt','r'),formatSpec);

x_min=0.0; x_max=350;

n=6;

subplot(n,1,1)
histogram(mumps); title('mumps'); xlim([x_min,x_max])

subplot(n,1,2)
histogram(ma27); title('ma27');xlim([x_min,x_max])

subplot(n,1,3)
histogram(ma57); title('ma57');xlim([x_min,x_max])

subplot(n,1,4)
histogram(ma77); title('ma77');xlim([x_min,x_max])

subplot(n,1,5)
histogram(ma86); title('ma86');xlim([x_min,x_max])

subplot(n,1,6)
histogram(ma97); title('ma97');xlim([x_min,x_max])