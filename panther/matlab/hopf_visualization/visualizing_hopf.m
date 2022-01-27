clc; clear;close all;
addpath(genpath('./../../../submodules/minvo/src/utils'));
set(0,'DefaultFigureWindowStyle','normal') %'normal' 'docked'

hopf([0 0 0 1])

all_psi=0:0.001:4*pi;%Note that we need 4pi here because stereogProj(q) != stereogProj(-q). and q_abc * q_psi = - q_abc * q_{psi+2pi}. Using 4pi covers all the cases
xi=rand(3,1);
abc=xi/norm(xi);

for psi=all_psi
    abc_prime=hopf(invhopf(abc(1), abc(2), abc(3), psi))
    assert(isAlmost(abc_prime(1),0.0,1e-6));
    assert(norm(abc_prime(2:end)-abc)<1e-6);
end

% clear psi
% syms psi real;
% q=invhopf(abc(1), abc(2), abc(3), psi);
% proj=stereogProj(q);
% figure;
% fplot3(proj(1),proj(2),proj(3),[0 2*pi]); axis equal

figure;  hold on
figure;  hold on
figure(2); hold on
plotSphere([0,0,0], 1.0, [17 17 17]/255)
alpha 0.2
xi1=1.4*[0.5 1 0.5]'; abc1=xi1/norm(xi1);
xi2=1.2*[1 -0.5 1]'; abc2=xi2/norm(xi2);
q_all=[];
my_colormap1=autumn;
my_colormap2=winter;

%%

for i=1:1:2
    if(i==1)
      abc=abc1;
      xi=xi1;
      my_colormap=my_colormap1;
    else
      xi=xi2;
      abc=abc2;
      my_colormap=my_colormap2;
    end
    all_projs=[];
    for psi=all_psi
        q=invhopf(abc(1), abc(2), abc(3), psi);
        all_projs=[all_projs stereogProj(q)];
    end
    figure(1); hold on

    color_tmp=my_colormap(ceil(end/2), :);
    my_colormap_plain=color_tmp.*ones(size(my_colormap));
    h=colormapline(all_projs(1,:),all_projs(2,:),all_projs(3,:),my_colormap_plain); axis equal;
    
    set(h,'linewidth',5)
    %colormap(my_colormap)
    %cbh=colorbar;
    %caxis([min(all_psi) max(all_psi)+0.001]);
    %set(cbh,'XTick',[0, pi, 2*pi, 3*pi, 4*pi])
    %set(cbh,'XTickLabel',{'0', '\pi', '2\pi', '3\pi', '4\pi'})

    figure(2); hold on
    
    plotSphere(abc, 0.05, color_tmp)
    arrow3dWithColor([0 0 0],xi',20,'cylinder',[0.15,0.1],color_tmp);

end
 figure(1);  axis off;
view(104,15)
lightangle(gca,180,60); axis equal;
plotAxesArrows(2.5)

 figure(2); axis off;
plotAxesArrows(1.5)
camlight
camlight
axis equal;
lighting phong; 
my_view=[115,18]
view(my_view(1),my_view(2));

%%
figure(1);
print('-dpng','-r500',"hopf_stereographic")
figure(2);
print('-dpng','-r500',"hopf_sphere")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:1:2
    figure; hold on; axis off;
    all_psi=[0.0:2*pi/4.0:(2*pi-0.01)];
    % subplot(1,numel(all_psi),i); hold on; axis off;
    if(i==1)
      abc=abc1;
      my_colormap=my_colormap1;
    else
      abc=abc2;
      my_colormap=my_colormap2;
    end
    for j=1:numel(all_psi)
        psi=all_psi(j);
    %     subplot(1,numel(all_psi),i);
        q=invhopf(abc(1), abc(2), abc(3), psi);

        trans=[-j/1.0;0;0];
        w_T_b=[toRotMat(q) trans; 0 0 0 1];
        tmp=[eye(3) trans; 0 0 0 1];
    %     plotAxesArrowsTnoColor(0.2,double(tmp));
        plotAxesArrowsT(0.2,double(w_T_b)); axis equal; lighting phong;
        text(trans(1),trans(2),trans(3)+0.3,['$\psi$=' num2str(psi)])
        view(my_view(1),my_view(2));
        percent=ceil(j/numel(all_psi)*size(my_colormap,1));

        color_tmp=my_colormap(ceil(end/2), :); %my_colormap(percent,:)
        plot3dcircleXY(w_T_b,0.2,color_tmp,0.5)
    end
    print('-dpng','-r500',['diff_yaws_',num2str(i)])
end

%%

%q must be in the form [w x y z]';
function result=hopf(q)
     q=checkQuatUnitNorm(q);
     result=quatmultiply(quatmultiply(q,[0 0 0 1]),quatinv(q));
     result=result';
end

function result=invhopf(a,b,c,psi)
  %See table 3 of https://arxiv.org/abs/2103.06372
  qabc=(1/sqrt(2*(1+c)))*[1+c -b a 0.0];
  result=quatmultiply(qabc, [cos(psi/2.0) 0.0 0.0 sin(psi/2.0)]);    
end

%See eq. 4 of https://nilesjohnson.net/hopf-articles/Lyons_Elem-intro-Hopf-fibration.pdf
function result=stereogProj(q)
     q=checkQuatUnitNorm(q);
     w=q(1); x=q(2); y=q(3); z=q(4);
     result=[x/(1-w), y/(1-w), z/(1-w)]';
end

function result=isAlmost(a,b,tol)
   result=(abs(a-b)<tol);
end

function result=checkQuatUnitNorm(q)
     q=q(:)';
     assert(numel(q)==4)
     if(isnumeric(q))
         assert(isAlmost(norm(q),1,1e-6)); %Make sure it has unit norm
         result=q/norm(q); %normalize in case numerical issues
     else
         result=q;
     end
end

