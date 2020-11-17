function H=demoOctave_shadedErrorBar
% demo function for shadedErrorBar
%
% function demo_shadedErrorBar
%
% No input or output arguments.
%
%
% 




rows=2;
cols=2;

clf



subplot(rows,cols,1)
%Plot the mean and standard deviation then overlay the raw data
y=randn(30,80)*5;
x=(1:size(y,2))-40;
yP = sin( linspace(-2*pi,2*pi,length(x)) )*20;
y = bsxfun(@plus,y,yP)+60;
shadedErrorBar(x,y,{@mean,@std}); 

hold on
plot(x,y,'.','color',[0.5,0.5,0.95])

hold off

grid on




subplot(rows,cols,2)
%Overlay different lines (transparent) lines and change their properties
hold on

plot2styles = {'-b';'-g'; '-r'};
for i = 1:3
  plt2invis = plot(0,0,plot2styles{i});
  set( plt2invis, 'visible', 'off'); 
end
  


x=(1:size(y,2))-40;
y=ones(30,1)*x; 
y=y+0.06*y.^2+randn(size(y))*10;
shadedErrorBar(x,y,{@mean,@std},'lineprops','-b','transparent',true,'patchSaturation',0.9)
shadedErrorBar(x,2*y+20,{@mean,@std},'lineprops',{'-go','MarkerFaceColor','g'},'transparent',true,'patchSaturation',0.9);

y=randn(30,80)*5; 
x=(1:size(y,2))-40;
yP = sin( linspace(-2*pi,2*pi,length(x)) )*20;
y = bsxfun(@plus,y,yP)+60;

%Make red line non-transparent
shadedErrorBar(x, y, {@mean,@std}, 'lineprops', '-r','transparent',false,'patchSaturation',0.075)
hold off

legend(['#1';'#2';'#3'],'location','northwest')
legend
grid on




subplot(rows,cols,3)
% Post-hoc modifications of line properties


y=randn(30,80)*5; 
x=(1:size(y,2));
yP = sin( linspace(-2*pi,2*pi,length(x)) )*20;
y = bsxfun(@plus,y,yP);


%Set face and edge properties
if (sum( size(ver('MATLAB'))) > 0  )
 s = shadedErrorBar(x, y, {@mean,@std}, 'lineprops', '-r')
 set(s.edge,'LineWidth',2,'LineStyle',':')
 s.mainLine.LineWidth = 5;
 s.patch.FaceColor = [0.5,0.25,0.25];
elseif (sum(size(ver('Octave'))) > 0)
 s = shadedErrorBar(x, y, {@mean,@std}, 'lineprops', {'-r','LineWidth',5,'LineStyle',':'});
end
hold on

if (sum( size(ver('MATLAB'))) > 0  )
 plot(s.mainLine.XData, s.mainLine.YData,'or','MarkerFaceColor','w')
end

hold off
grid on

set(gca,'XTickLabel',[],'YTickLabel',[])





subplot(rows,cols,4)
% Post-hoc modifications of line properties


y=randn(256,80)*5; 
x=(1:size(y,2));
yP = cos( linspace(-2*pi,2*pi,length(x)) )*10;
y = bsxfun(@plus,y,yP);


shadedErrorBar(x, y, {@mean,@std}, 'lineprops', '-r')

hold on

y=mean(y)+16;
errbar = [2*ones(1,length(x)) ; 4*ones(1,length(x))];

shadedErrorBar(x, y, errbar, 'lineprops', '-g')

