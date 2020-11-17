function RGBvalue = ColorSpec_ShortName_to_RGBvalue(shortName)

% RGBvalue = ColorSpec_ShortName_to_RGBvalue(shortName) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     
%     Given the single character shortName ('r','g',...) convert to the matrix equivalent
%         
% Example:
%     val = ColorSpec_ShortName_to_RGBvalue('m');
%     -------> returns the three-element equivalent to 'm' (magenta) = [1 0 1]
% 
%     Author: Shawn Arseneau
%     Created: September 14, 2006
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    switch(lower(shortName))
        case 'y'
            RGBvalue = [1 1 0];
        case 'm'
            RGBvalue = [1 0 1];            
        case 'c'
            RGBvalue = [0 1 1];            
        case 'r'
            RGBvalue = [1 0 0];            
        case 'g'
            RGBvalue = [0 1 0];            
        case 'b'
            RGBvalue = [0 0 1];            
        case 'w'
            RGBvalue = [1 1 1];            
        case 'k'
            RGBvalue = [0 0 0];            
        otherwise
            RGBvalue = [];
            msg = sprintf('Unrecognized ShortName: %s - See valid list (y, m, c, r, g, b, w, k)', lower(ShortName));
            error(msg); 
            return;
    end







