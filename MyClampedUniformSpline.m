%Jesus Tordesillas Torres, jtorde@mit.edu, November 2020

%See https://www.mathworks.com/help/matlab/matlab_oop/comparing-handle-and-value-classes.html
classdef MyClampedUniformSpline < handle

    properties
        t0
        tf
        delta_t
        M
        N
        p   
        knots
        num_seg
        num_cpoints
        CPoints %Cell array of size 3\times(N+1)
    end
    
    methods
        function obj = MyClampedUniformSpline(t0, tf, deg, dim, num_seg, casadi_opti)
%             if nargin == 0, obj.q = Q.Identity; end
%             if nargin == 1, obj.q = varargin{1}; end
            obj.t0 = t0;
            obj.tf = tf;
            obj.p = deg;
            obj.num_seg = num_seg;
            obj.M =  obj.num_seg + 2 * obj.p;
            obj.delta_t = (obj.tf -  obj.t0) / (1.0 * (obj.M - 2 * obj.p - 1 + 1));
            obj.N = obj.M - obj.p - 1;
            obj.num_cpoints=obj.N+1;
            
            
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
            
            %Create the control points
            obj.CPoints={};
            for i=1:obj.num_cpoints
                obj.CPoints{end+1}=casadi_opti.variable(dim,1); %Control points
            end

        end

        function setCPoints(obj,Q) %Q should be a cell array
%             Q_matrix=cell2mat(Q);
%             assert(size(Q{1},1)==3,'size(Q_matrix,1)==3 not satisfied')
            assert(size(Q,2)==(obj.N+1),'size(Q_matrix,2)==(obj.N+1) not satisfied')
            obj.CPoints=Q;            
        end
        
        function result=getIndexIntervalforT(obj,t)
            assert(isa(t,'double'),'t should be double, not other type (like sym)')
            
            assert(t<=max(obj.knots),'t<=max(obj.knots) not satisfied')
            assert(t>=min(obj.knots),'t>=min(obj.knots) not satisfied')

            
            result=max(0,min(find(obj.knots>=t))-obj.p-2);
            
        end
        
        function result=timeSpanOfInterval(obj,j)
            init_int=obj.knots(tm(obj.p+j));
            end_int=obj.knots(tm(obj.p+j+1));
            result=[init_int, end_int];
        end
        
        function result=getCPsofInterval(obj,j)
            result={};
            for index=j:(j+obj.p)
                result{end+1}=obj.CPoints{tm(index)};
            end
        end
        
        function result=getMINVOCPsofInterval(obj,j)
            Q_Bs_matrix=obj.convertCellArrayCPsToMatrix(obj.getCPsofInterval(j));
            
            A_BS=obj.getA_BS_Interval(j);
            A_MV=getA_MV(obj.p,[0,1]);
            
            Q_Mv_matrix=Q_Bs_matrix*A_BS*inv(A_MV);
            
            result=obj.convertMatrixCPsToCellArray(Q_Mv_matrix);
            
        end

        %j>=0 is the index of the interval
        % u \in [0,1] is the time along that interval
        
        function result = getPosU(obj,u,j)
                result = obj.evalDerivativeU(0,u,j);               
        end

        function result = getVelU(obj,u,j)
                result = obj.evalDerivativeU(1,u,j);               
        end

        function result = getAccelU(obj,u,j)
                result = obj.evalDerivativeU(2,u,j);               
        end

        function result = getJerkU(obj,u,j)
                result = obj.evalDerivativeU(3,u,j);               
        end

        %%%
        function result = getPosT(obj,t)
                result = obj.evalDerivativeT(0,t);               
        end

        function result = getVelT(obj,t)
                result = obj.evalDerivativeT(1,t);               
        end

        function result = getAccelT(obj,t)
                result = obj.evalDerivativeT(2,t);               
        end

        function result = getJerkT(obj,t)
                result = obj.evalDerivativeT(3,t);               
        end
        
        %For spline, control_cost=
        function result=getControlCost(obj)
            
            result=0;
            for j=0:(obj.num_seg-1)   
%                 Q=[q_exp_{tm(i)} q_exp_{tm(i+1)} q_exp_{tm(i+2)} q_exp_{tm(i+3)}];
%                 jerk=Q*A_pos_bs_{tm(i)}*[6 0 0 0]';
                jerk=obj.evalDerivativeU(obj.p,0.5,j);
                result=result + jerk'*jerk*obj.delta_t;
            end
            
        end

        function result=convertMatrixCPsToCellArray(obj, cps)
            result={};
            for kk=1:size(cps,2)
               result{end+1}=cps(:,kk);
            end
        end
        
        function result=convertCellArrayCPsToMatrix(obj, cps)
            result=[];
            for kk=1:size(cps,2)
               result=[result cps{kk}]; 
            end
        end

        function result = evalDerivativeU(obj,order,u,j) %j>=0 is the index of the interval
                               % u \in [0,1] is the time along that interval
            
            syms tmp real;
           
            Tmp=(tmp.^[obj.p:-1:0])';
            
            A=obj.getA_BS_Interval(j);
            
            Q_j_cell=obj.getCPsofInterval(j);
            
%             Qj=cell2mat(obj.getCPsofInterval(j)); %doesn't work with sym objects
            Qj=obj.convertCellArrayCPsToMatrix(Q_j_cell);
            
            ADiffT=(A* subs(diff(Tmp,tmp,order),tmp,u));
                       
            if(numel(symvar(ADiffT))==0)
                ADiffT=double(ADiffT);
            end
           
            result=(1/(obj.delta_t^order)) *Qj*ADiffT;
            
        end
        
        function result=evalDerivativeT(obj,order,t)
            assert(isa(t,'double'),'t should be double, not other type (like sym)')
            
            j=obj.getIndexIntervalforT(t);
            
            interv=obj.timeSpanOfInterval(j);           
            
            u=(t-min(interv))/(max(interv)-min(interv));
            result=obj.evalDerivativeU(order,u,j);
        end

        function plotPosVelAccelJerk(obj)
            figure; hold on;
            subplot(4,1,1); hold on; title('pos')
            obj.plotPos();
            subplot(4,1,2); hold on; title('vel')
            obj.plotVel();
            subplot(4,1,3); hold on; title('accel')
            obj.plotAccel();
            subplot(4,1,4); hold on; title('jerk')
            obj.plotJerk();
        end
        
        function plotDerivative(obj,order)
            syms t real
            for j=0:(obj.num_seg-1)
                interv=obj.timeSpanOfInterval(j);           
                u=(t-min(interv))/(max(interv)-min(interv));
                fplot(obj.evalDerivativeU(order,u,j),interv); hold on;
            end        
        end

        function plotPos3D(obj)
            figure; hold on;
            syms t real
            for j=0:(obj.num_seg-1)
                interv=obj.timeSpanOfInterval(j);           
                u=(t-min(interv))/(max(interv)-min(interv));
                pos=obj.evalDerivativeU(0,u,j);
                fplot3(pos(1),pos(2),pos(3),interv);
            end           
            axis equal; view([45,45])
        end
        
        function plotPos(obj)
            obj.plotDerivative(0)      
        end

        function plotVel(obj)
            obj.plotDerivative(1)      
        end
        
        function plotAccel(obj)
            obj.plotDerivative(2)      
        end
        
        function plotJerk(obj)
            obj.plotDerivative(3)      
        end

        function A=getA_BS_Interval(obj,j)
            %the intervals are [0,1,2,...,num_seg-2,num_seg-1]
            if(j<(obj.num_seg-2))
                A=computeMatrixForClampedUniformBSpline(obj.p,j,[0,1]);
            elseif(j==(obj.num_seg-2))
                A=computeMatrixForClampedUniformBSpline(obj.p,-2,[0,1]);
            elseif(j==(obj.num_seg-1))
                A=computeMatrixForClampedUniformBSpline(obj.p,-1,[0,1]);
            else
                error("This interval does not exist");
            end
            
            A=double(A);
        end
        
        %Simply to check the derivatives by doing finite differences
        function plotPosVelAccelJerkFiniteDifferences(obj)
            figure; hold on;
            pos=[];
            delta_tmp=(obj.tf-obj.t0)/100;
            
            times=obj.t0:delta_tmp:obj.tf;
            for t=times
                fprintf('Progress= %f\n',100*t/obj.tf)
                pos=[pos double(obj.getPosT(t))];
            end


            subplot(4,1,1); hold on; title('pos')
            plot(times(1:end),pos,'--')
            subplot(4,1,2); hold on; title('vel')
            plot(times(1:end-1),diff(pos,1,2)/delta_tmp,'--')
            subplot(4,1,3); hold on; title('accel')
            plot(times(1:end-2),diff(pos,2,2)/delta_tmp^2,'--')
            subplot(4,1,4); hold on; title('jerk')
            plot(times(1:end-3),diff(pos,3,2)/delta_tmp^3,'--')     
        end
        
        function updateCPsWithSolution(obj, sol_casadi)
            Q={};
            for i=0:(obj.num_cpoints-1)
                Q{tm(i)}=sol_casadi.value(obj.CPoints{tm(i)}); %Control points
            end
            obj.CPoints=Q; 
        end
 
    end
end
