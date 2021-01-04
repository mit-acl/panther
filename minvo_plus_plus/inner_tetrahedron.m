%In the first part of this file we test that C=V*Q, where C is the matrix
%that contains the centroids as columns

%In the second part of this file we obtain the largest tetrahedron inside a
%given polynomial curve (inner tetrahedron inside a given polynomial curve)
%We use re results of Lemma 3.148 of Pablo
%Parillo's book (probability measure in [-1,1])

interv=[-1,1]
A=getA_MV(3,interv);  
P=[rot90(eye(3)), zeros(3,1)];
V=P*inv(A);
C=[mean(V(:,1:3),2) mean(V(:,[1,3,4]),2)  mean(V(:,[2,3,4]),2) mean(V(:,[1,2,4]),2)]

Q=(1/3)*[0 1 1 1;
         1 0 1 1;
         1 1 0 1;
         1 1 1 0];

C_mia=V*Q;

%%
addpath(genpath('./utils'));
addpath(genpath('./solutions'));

c0=sdpvar(3,1);
c1=sdpvar(3,1);
c2=sdpvar(3,1);
c3=sdpvar(3,1);

const=[];
const=[const getPsdConstraints(H1(c0)+H2(c0))];
const=[const getPsdConstraints(H1(c1)+H2(c1))];
const=[const getPsdConstraints(H1(c2)+H2(c2))];
const=[const getPsdConstraints(H1(c3)+H2(c3))];

const=[const getPsdConstraints(H1(c0)-H2(c0))];
const=[const getPsdConstraints(H1(c1)-H2(c1))];
const=[const getPsdConstraints(H1(c2)-H2(c2))];
const=[const getPsdConstraints(H1(c3)-H2(c3))];

C=[c0 c1 c2 c3];

obj=computeDet([C' ones(4,1)]);

assign(C,rand(size(C)))
assign(C,C_mia)

general_settings=sdpsettings('savesolveroutput',0,'savesolverinput',1,'showprogress',2,'verbose',2,'debug',1);
settings=sdpsettings(general_settings,'usex0',1,'solver','fmincon','bmibnb.maxiter',5e20000,'bmibnb.maxtime',5e20000);

result=optimize(const,obj,settings)

C=value(C)

function H=H1(c)
H=[1 c(1);c(1) c(2)];
end

function H=H2(c)
H=[c(1) c(2);c(2) c(3)];
end

function constraints=getPsdConstraints(A)
constraints=[];

%See Theorem 17 Of Lecture 5 of https://homepages.cwi.nl/~monique/eidma-seminar-parrilo/Parrilo-LectureNotes-EIDMA.pdf

sdpvar lambda;
n=size(A,1);
charac_poly=computeDet(lambda*eye(size(A,1))-A);

coeffs_charac_poly=coefficients(charac_poly,lambda); %[p0 p1 ... p_n-1 1]

size(coeffs_charac_poly)
for(i=0:(n-1))
    j=i+1;
    constraints=[constraints coeffs_charac_poly(j)*((-1)^(n-i))>=0 ];
end
% 
% if(size(A,1)==1)
%     constraints=[A(1,1)>=0];
%     return;
% elseif(size(A,1)==2)
%     constraints=[A(1,1)>=0, A(2,2)>=0, (A(1,1)*A(2,2)-A(1,2)*A(2,1))>=0]  
%     return;
% elseif(size(A,1)==3)
% 
%     tmp=A(2:3, 2:3); %delete 1
%     constraints=[constraints getPsdConstraints(tmp)] 
%     tmp=A([1, 3:end], [1, 3:end]); %delete 2
%     constraints=[constraints getPsdConstraints(tmp)]
%     tmp=A(1:2, 1:2); %delete 3
%     constraints=[constraints getPsdConstraints(tmp)] 
%     
%     constraints=[constraints computeDet(A)>=0]
%     return;
% else
%     error('NOT YET IMPLEMENTED')
% end
    
   
end

