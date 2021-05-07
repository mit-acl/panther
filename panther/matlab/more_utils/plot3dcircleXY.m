% Taken partly from https://www.mathworks.com/matlabcentral/answers/355077-plot-a-circle-onto-a-3d-graph
function plot3dcircleXY(w_T_b,radius,color,alpha)
    teta=0:0.01:2*pi ;
    x_all=radius*cos(teta);
    y_all=radius*sin(teta) ;
    z_all=zeros(size(x_all)) ;
    
    x_all_tr=[];
    y_all_tr=[];
    z_all_tr=[];
    for (i=1:numel(x_all))
        transformed=w_T_b*[x_all(i);y_all(i);z_all(i);1.0];
        x_all_tr=[x_all_tr transformed(1)];
        y_all_tr=[y_all_tr transformed(2)];
        z_all_tr=[z_all_tr transformed(3)];
    end
    patch(x_all_tr,y_all_tr,z_all_tr,color,'FaceAlpha',alpha)
    hold on
%     plot3(C(1),C(2),C(3),'*r')
end