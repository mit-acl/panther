%Jesus Tordesillas Torres, jtorde@mit.edu, November 2020

%Everything here is 1-based indexing (first element is one)

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

            obj.t0 = t0;
            obj.tf = tf;
            obj.p = deg;
            obj.num_seg = num_seg;
            obj.M =  obj.num_seg + 2 * obj.p;
            obj.delta_t = (obj.tf -  obj.t0) / (1.0 * (obj.M - 2 * obj.p - 1 + 1));
            obj.N = obj.M - obj.p - 1;
            obj.num_cpoints=obj.N+1;

            obj.knots=[obj.t0*ones(1,obj.p+1)       obj.t0+obj.delta_t*(1:obj.M - 2*obj.p-1)          obj.tf*ones(1,obj.p+1)];
            
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

            
            result=max(1,min(find(obj.knots>=t))-obj.p-2+1);
            
        end
        
        function result=timeSpanOfInterval(obj,j)
            init_int=obj.knots(obj.p+j);
            end_int=obj.knots(obj.p+j+1);
            result=[init_int, end_int];
        end
        
        function result=getCPsofInterval(obj,j)
            result={};
            for index=j:(j+obj.p)
                result{end+1}=obj.CPoints{index};
            end
        end
        
        %returns the l-th BS velocity control point
        function result=getCP_BS_Vel(obj,l)
            assert(l<=(obj.N), 'l<=N not satisfied');
            result=(obj.p*(obj.CPoints{l+1}-obj.CPoints{l}))/(obj.knots(l+obj.p+1)-obj.knots(l+1));
        end
        
        %returns the l-th BS acceleration control point
        function result=getCP_BS_Accel(obj,l)
            assert(l<=(obj.N-1), 'l<=(N-1) not satisfied');
            result=(obj.p-1)*(obj.getCP_BS_Vel(l+1)-obj.getCP_BS_Vel(l))/(obj.knots((l+obj.p+1))-obj.knots((l+2)));
        end
        
        %%%%%%%%%%%%%% BSPLINE (BS)
        %%%%%%%%%%%%%%%%%%%%%%
        %returns the BS position control points of the interval j
        function result=getCPs_BS_Pos_ofInterval(obj,j)
            result=obj.CPoints(j:j+obj.p);
        end
        %returns the BS velocity control points of the interval j
        function result=getCPs_BS_Vel_ofInterval(obj,j)
            deg_vel=obj.p-1;
            Q_Bs_matrix=[];
            
            for tmp=j:(j+deg_vel)
                Q_Bs_matrix=[Q_Bs_matrix obj.getCP_BS_Vel(tmp)];
            end

            result=obj.convertMatrixCPsToCellArray(Q_Bs_matrix);
        end
        
        %returns the BS acceleration control points of the interval j
        function result=getCPs_BS_Accel_ofInterval(obj,j)
            deg_accel=obj.p-2;
            Q_Bs_matrix=[];
            
            for tmp=j:(j+deg_accel)
                Q_Bs_matrix=[Q_Bs_matrix obj.getCP_BS_Accel(tmp)];
            end

            result=obj.convertMatrixCPsToCellArray(Q_Bs_matrix);
        end
        %%%%%%%%%%%%%%%%%%%%%%

        %%%%%%%%%%%%%% MINVO (MV)
        %%%%%%%%%%%%%%%%%%%%%%
        %returns the MV position control points of the interval j
        function result=getCPs_MV_Pos_ofInterval(obj,j)
            Q_Bs_matrix=obj.convertCellArrayCPsToMatrix(obj.getCPs_BS_Pos_ofInterval(j));
                                                   %Q_Bs_matrix*   A_BS*                         inv(A_MV)
            result=obj.convertMatrixCPsToCellArray(Q_Bs_matrix*    obj.getA_BS_Pos_Interval(j)*  inv(getA_MV(obj.p, [0,1])));
        end
        
        %returns the MV velocity control points of the interval j
        function result=getCPs_MV_Vel_ofInterval(obj,j)
            Q_Bs_matrix=obj.convertCellArrayCPsToMatrix(obj.getCPs_BS_Vel_ofInterval(j));
                                                   %Q_Bs_matrix*   A_BS*                         inv(A_MV)
            result=obj.convertMatrixCPsToCellArray(Q_Bs_matrix*    obj.getA_BS_Vel_Interval(j)*  inv(getA_MV(obj.p-1, [0,1])));
        end
        
        %returns the MV acceleration control points of the interval j
        function result=getCPs_MV_Accel_ofInterval(obj,j)
            Q_Bs_matrix=obj.convertCellArrayCPsToMatrix(obj.getCPs_BS_Accel_ofInterval(j));
                                                   %Q_Bs_matrix*   A_BS*                         inv(A_MV)
            result=obj.convertMatrixCPsToCellArray(Q_Bs_matrix*    obj.getA_BS_Accel_Interval(j)*  inv(getA_MV(obj.p-2, [0,1])));
        end
        %%%%%%%%%%%%%%%%%%%%%%


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
            for j=1:obj.num_seg   
%                 Q=[q_exp_{(i)} q_exp_{(i+1)} q_exp_{(i+2)} q_exp_{(i+3)}];
%                 jerk=Q*A_pos_bs_{(i)}*[6 0 0 0]';
                control=obj.evalDerivativeU(obj.p,0.5,j);
                result=result + control'*control*obj.delta_t;
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

        function result=getCPsAsMatrix(obj)
            result=convertCellArrayCPsToMatrix(obj, obj.CPoints);
        end
        
        %Converts local time (u) to global time (t) 
        function result=u2t(obj,u,j)
            interv=obj.timeSpanOfInterval(j);
            result=min(interv)+u*(max(interv)-min(interv));
        end

        function result = evalDerivativeU(obj,order,u,j) %j>=1 is the index of the interval
                               % u \in [0,1] is the time along that interval    
            
            A=obj.getA_BS_Pos_Interval(j);
            
            Q_j_cell=obj.getCPsofInterval(j);
            
%             Qj=cell2mat(obj.getCPsofInterval(j)); %doesn't work with sym objects
            Qj=obj.convertCellArrayCPsToMatrix(Q_j_cell);
            
            %%%%%%%%%%%%%%%%%%%%%%
            %Option 1 (works always, but there seems to be a bug in casadi here: https://groups.google.com/g/casadi-users/c/CBimXBsQ2MA)
%             pto0=obj.p:-1:0;
%             if(order>obj.p)
%                 diffT=zeros(obj.p+1,1);
%             else
%                 diffT=((factorial(pto0)./factorial(max(pto0-order,0))).*[(u.^[obj.p-order:-1:0]) zeros(1,order)])';
%             end
%             ADiffT=A*diffT;
%             
%             if(numel(symvar(ADiffT'))==0)
%                 ADiffT=double(ADiffT);
%             end
%            
%             result=(1/(obj.delta_t^order)) *Qj*ADiffT;
            %%%%%%%%%%%%%%%%%%%%%%
            
            %Option 2 (works always)
            syms u_sym real;
            Tmp=(u_sym.^[obj.p:-1:0])';
            diffT= diff(Tmp,u_sym,order); 
            ADiffTdelta=simplify(A*diffT*(1/(obj.delta_t^order)));
            
            u_sym=u;
            ADiffTdelta=eval(char(ADiffTdelta)); %See https://www.mathworks.com/matlabcentral/answers/310042-how-to-convert-symbolic-expressions-to-transfer-functions

            result=Qj*ADiffTdelta;
            
%             if(numel(symvar(ADiffT'))==0)
%                 ADiffT=double(ADiffT);
%             end
%            
%             result=(1/(obj.delta_t^order)) *Qj*ADiffT;
%             
            
            
        end
        
        function result=evalDerivativeT(obj,order,t)
            assert(isa(t,'double'),'t should be double, not other type (like sym)')
            
            j=obj.getIndexIntervalforT(t);
            
            interv=obj.timeSpanOfInterval(j);           
            
            u=(t-min(interv))/(max(interv)-min(interv));
            
            result=obj.evalDerivativeU(order,u,j);
        end

        function plotPosVelAccelJerk(obj, v_max, a_max, j_max)
            figure; hold on;
            subplot(4,1,1); hold on; title('pos')
            obj.plotPos();
            subplot(4,1,2); hold on; title('vel')
            obj.plotVel();
            subplot(4,1,3); hold on; title('accel')
            obj.plotAccel();
            subplot(4,1,4); hold on; title('jerk')
            obj.plotJerk();
            style={'-.r','-.g','-.b'}; 
            if(nargin>=2)
                subplot(4,1,2);
                for coordinate=1:numel(v_max)
                    yline(v_max(coordinate),style{coordinate}); %TODO: this will crash if there are moree than three coordinates (because of the def of style above)
                    yline(-v_max(coordinate),style{coordinate});
                end
            end
            if(nargin>=3)
                subplot(4,1,3);
                for coordinate=1:numel(a_max)
                    yline(a_max(coordinate),style{coordinate}); %TODO: this will crash if there are moree than three coordinates (because of the def of style above)
                    yline(-a_max(coordinate),style{coordinate});
                end
            end     
            if(nargin>=4)
                subplot(4,1,4);
                for coordinate=1:numel(j_max)
                    yline(j_max(coordinate),style{coordinate}); %TODO: this will crash if there are moree than three coordinates (because of the def of style above)
                    yline(-j_max(coordinate),style{coordinate});
                end
            end     
        end
        
        function plotDerivative(obj,order)
            syms t real
            for j=1:obj.num_seg
                interv=obj.timeSpanOfInterval(j);           
                u=(t-min(interv))/(max(interv)-min(interv));
                fplot(obj.evalDerivativeU(order,u,j),interv); hold on;
            end        
        end

        function plotPos3D(obj)
            figure; hold on;
            syms t real
            for j=1:obj.num_seg
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

        function A=getA_BS_Derivative_Interval(obj,which_derivative,j) %which_derivative=0 for pos, 1 for vel, 2 for accel
            %the intervals are [0,1,2,...,num_seg-2,num_seg-1]
            
            assert(which_derivative<=(obj.p-1))
            assert(j>=1 )
            assert(j<=obj.num_seg)
       
            degree_derivative=obj.p-which_derivative;
            
            % ultimo segmento es num_seg
            % antepenultimo es num_seg-1
            
            %Note that this 
            if(j<(obj.num_seg-1))
                A=computeMatrixForClampedUniformBSpline(degree_derivative,j-1,[0,1]); 
            elseif(j==(obj.num_seg-1))
                A=computeMatrixForClampedUniformBSpline(degree_derivative,-2,[0,1]);
            elseif(j==obj.num_seg)
                A=computeMatrixForClampedUniformBSpline(degree_derivative,-1,[0,1]); %last segment
            end
            
            A=double(A);
        end
        
        function result=getA_BS_Pos_Interval(obj,j)
            result=obj.getA_BS_Derivative_Interval(0,j);
        end
        
        function result=getA_BS_Vel_Interval(obj,j)
            result=obj.getA_BS_Derivative_Interval(1,j);
        end
        
        function result=getA_BS_Accel_Interval(obj,j)
            result=obj.getA_BS_Derivative_Interval(2,j);
        end
        
        %Simply to check the derivatives by doing finite differences
        function plotPosVelAccelJerkFiniteDifferences(obj)
            hold on;
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
        
        function updateCPsWithSolution(obj, Qsolution_matrix)
            Q={};
            for i=1:obj.num_cpoints
                Q{i}=Qsolution_matrix(:,i);%sol_casadi.value(obj.CPoints{i}); %Control points
            end
            obj.CPoints=Q; 
        end

        function printCPs(obj)
            for i=1:obj.num_cpoints
                obj.CPoints{i}'
            end
        end
        
%         function initialGuessForCPs(obj, Q_guess, opti_casadi)
% %             for i=1:obj.num_cpoints
% %                 opti_casadi.set_initial(Q{(i)},Q_guess{(i)}); %Control points
% %             end
%         end
 
    end
end
