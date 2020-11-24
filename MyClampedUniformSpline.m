classdef MyClampedUniformSpline

    properties
        t0
        tf
        delta_t
        M
        N
        p   
        knots
    end
    
    methods
        function obj = MyClampedUniformSpline(varargin) %(t0, tf, degree, num_pol)
%             if nargin == 0, obj.q = Q.Identity; end
%             if nargin == 1, obj.q = varargin{1}; end
            obj.t0 = varargin{1};
            obj.tf = varargin{2};
            obj.p = varargin{3};
            num_pol_=varargin{4};
            obj.M = num_pol_ + 2 * obj.p;
            obj.delta_t = (obj.tf -  obj.t0) / (1.0 * (obj.M - 2 * obj.p - 1 + 1));
            obj.N = obj.M - obj.p - 1;
            
            
            obj.knots=[];
            for i=0:obj.p
                obj.knots=[obj.knots obj.t0];
            end

            for i=(obj.p + 1):(obj.M - obj.p - 1)
                obj.knots=[obj.knots obj.knots(tm(i - 1)) + obj.delta_t];
            end

            for i=(obj.M - obj.p):obj.M
                obj.knots=[obj.knots obj.tf];
            end

        end
        
        %WORK IN PROGRESS
        function R = eval(u,j) %j is the index of the interval
            
                init_int=obj.knots(tm(obj.p+j));
                end_int=obj.knots(tm(obj.p+j+1));

        end
 
    end
end
