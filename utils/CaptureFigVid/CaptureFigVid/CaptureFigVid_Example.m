% Example of using CaptureFigVid
% Cheers, Dr. Alan Jennings, Research assistant professor, 
% Department of Aeronautics and Astronautics, Air Force Institute of Technology

%% Set up 3D plot to record
figure(171);clf;
surf(peaks,'EdgeColor','none','FaceColor','interp','FaceLighting','phong')
daspect([1,1,.3]);axis tight;

%% Set up recording parameters (optional), and record
OptionZ.FrameRate=15;OptionZ.Duration=5.5;OptionZ.Periodic=true;
CaptureFigVid([-20,10;-110,10;-190,80;-290,10;-380,10], 'WellMadeVid2',OptionZ)
