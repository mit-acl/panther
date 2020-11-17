function RGBvalue = ColorSpec_LongName_to_RGBvalue(LongName)

% RGBvalue = ColorSpec_LongName_to_RGBvalue(LongName) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     
%     Given the color 'LongName' ('red','green',...) convert to the matrix equivalent
%         
% Example:
%     val = ColorSpec_LongName_to_RGBvalue('magenta');
%     ------> returns the three-element equivalent to magenta = [1 0 1]
% 
% 
%     Author: Shawn Arseneau
%     Created: September 14, 2006
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    switch(lower(LongName))
        case 'yellow'
            RGBvalue = [1 1 0];
        case 'magenta'
            RGBvalue = [1 0 1];            
        case 'cyan'
            RGBvalue = [0 1 1];            
        case 'red'
            RGBvalue = [1 0 0];            
        case 'green'
            RGBvalue = [0 1 0];            
        case 'blue'
            RGBvalue = [0 0 1];            
        case 'white'
            RGBvalue = [1 1 1];            
        case 'black'
            RGBvalue = [0 0 0];            
        otherwise
            RGBvalue = [];  
            msg = sprintf('Unrecognized LongName: %s - See valid list (yellow, magenta, cyan, red, green, blue, white, black)', lower(LongName));
            error(msg); 
            return;
    end







