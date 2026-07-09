function CCI = CCI(a,b)
%CCI 此处显示有关此函数的摘要
%   此处显示详细说明
% 0.9
% k=0.9;
% y = k+(1-k)*(tanh(a'*b*1e7)+1)/2;

% if(a'*b > 0)
%     y = 1;
% else
%     y = 0.9;
% end
CCI=[0;0;0];
k = 1;
y = a.* b>=0;
for i=1:3
    if(y(i)>0)
        CCI(i)=1;
    else
        CCI(i)=k;
    end
end

% for i=1:3
%     CCI(i)=k+(1-k)*(tanh(y(i)*1e10)+1)/2;
% end


