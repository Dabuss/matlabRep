function rmat = eulerZYXToRotationMatrix(a,b,c)
    validateattributes(a,{'numeric'},{'size',[1,1]})
    validateattributes(b,{'numeric'},{'size',[1,1]})
    validateattributes(c,{'numeric'},{'size',[1,1]})
    
    rmat = [[cos(a)*cos(b)  ,  cos(a)*sin(b)*sin(c) - sin(a)*cos(c)  ,  cos(a)*sin(b)*cos(c) + sin(a)*sin(c)];...
            [sin(a)*cos(b)  ,  sin(a)*sin(b)*sin(c) + cos(a)*cos(c)  ,  sin(a)*sin(b)*cos(c) - cos(a)*sin(c)];...
            [     -sin(b)   ,                cos(b)*sin(c)           ,                cos(b)*cos(c)         ]];
    
end