%Note that MX can only have numbers
%See also https://groups.google.com/g/casadi-users/c/8df2GfqrAS4/m/BTNsnkhiAwAJ
function result=convertMX2Matlab(A)
    f = casadi.Function('f', {}, {A},...
                      {},{'A'});
%     f=f.expand(); %not strictly necessary
    s=f(); %now the field s.A has type DM
    result=full(s.A); %Convert to matlab
end