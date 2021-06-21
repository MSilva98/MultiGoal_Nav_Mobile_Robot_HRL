function [I_comb,flag,IPoint] = LineLineIntersection(A,B)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
NUM_lines = size(A,1);
I_comb = combnk(1:NUM_lines,2);
NUM_comb = size(I_comb,1);
flag = zeros(1,NUM_comb);
IPoint = zeros(NUM_comb,2);
for kk = 1:NUM_comb,
    % Line AB represented as a1x + b1y = c1 
    a1 = B(I_comb(kk,1),2)-A(I_comb(kk,1),2); 
    b1 = A(I_comb(kk,1),1)-B(I_comb(kk,1),1); 
    c1 = A(I_comb(kk,1),1)*B(I_comb(kk,1),2)-A(I_comb(kk,1),2)*B(I_comb(kk,1),1);
    % Line CD represented as a2x + b2y = c2 
    a2 = B(I_comb(kk,2),2)-A(I_comb(kk,2),2); 
    b2 = A(I_comb(kk,2),1)-B(I_comb(kk,2),1); 
    c2 = A(I_comb(kk,2),1)*B(I_comb(kk,2),2)-A(I_comb(kk,2),2)*B(I_comb(kk,2),1);
  
    determinant = a1*b2 - a2*b1; 
    if (determinant == 0) ,
        % The lines are parallel
        flag(kk) = 0;
        IPoint(kk,:) = [NaN NaN];
    else
        flag(kk) = 1;
        x = (b2*c1 - b1*c2)/determinant; 
        y = (a1*c2 - a2*c1)/determinant; 
        IPoint(kk,:) = [x,y]; 
    end
end
end %###################################################################END
