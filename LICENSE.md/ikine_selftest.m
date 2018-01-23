clear;clc;
format short;
close all;


position=[10,80,70];  %end tip coordinates

TT0=eye(4);
TT0(1:3,end)=position';

Q=ikine_self(TT0)*180/pi;
disp(['~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~']);
disp(['D-H parameters Q=£º']);
disp(Q);

Q_studio=rem(Q,360);

disp(['~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~']);
disp(['Joint angle£º']);
Q_studio


if(size(Q_studio,2)>0)
    TT0=fkine_self(Q_studio(:,2));
end

