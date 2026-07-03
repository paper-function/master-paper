function r2d=Coefficient_r2d(alpha)
%=====根据给定马赫数和攻角，求取相关阻力系数=====
%=====创建时间2018.08.20======
%=====修改时间2018.08.20=======

if alpha<12   % alpha>0 &&
    r2d=1*180/pi;
elseif alpha<24  % alpha>20 &&
    r2d=((alpha-12)/(24-12)*(0.5-1)+1)*180/pi;
elseif alpha<41  % alpha>20 &&
    r2d=((alpha-24)/(41-24)*(0.2-0.5)+0.5)*180/pi;
elseif alpha<46  % alpha>20 &&
    r2d=((alpha-41)/(46-41)*(0.1-0.2)+0.2)*180/pi;
elseif alpha<51  % alpha>20 &&
    r2d=0.1*180/pi;
else
    r2d=0.1*180/pi;
end