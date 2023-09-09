classdef PolyTraj
    properties
        polyOrder, 
        cx, cy, cz,
        sigma_x, sigma_y, sigma_z,
        dsigma_x, dsigma_y, dsigma_z,
        d4sigma_x, d4sigma_y, d4sigma_z
    end
    
    methods
        function obj = PolyTraj(trajname, polyOrderInput)
            obj. polyOrder = polyOrderInput;
            
            obj. cx = syms (trajname + "cx",  [1,obj.polyOrder]);
            obj. cy = syms (trajname + "cy",  [1,obj.polyOrder]);
            obj. cz = syms (trajname + "cz",  [1,obj.polyOrder]);
            syms t;
            
  
            
            obj. sigma_x = poly2sym(cx, t);
            obj. sigma_y = poly2sym(cy, t);
            obj. sigma_z = poly2sym(cz, t);
            
            obj. dsigma_x = diff(obj.sigma_x,t,1);
            obj. dsigma_y = diff(obj.sigma_y,t,1);
            obj. dsigma_z = diff(obj.sigma_z,t,1);
            
            obj. d4sigma_x = diff(obj.sigma_x,t,4);
            obj. d4sigma_y = diff(obj.sigma_y,t,4);
            obj. d4sigma_z = diff(obj.sigma_z,t,4);
        end
    end
end