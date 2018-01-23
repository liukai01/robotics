function Q=ikine_self(TT)
%% Known robot D-H parameters and tool coordinate system pose TT, get each joint variable q
%Read robot D-H parameters and assign values
format short;
[D_Hdata,string]=xlsread('D-H parameters',1);
N=4;
% global D_Hdata;
for i=1:N
    eval(['a',num2str(i-1),'=D_Hdata(i,2);']);
    eval(['alpha',num2str(i-1),'=D_Hdata(i,3)*pi/180;']);
    eval(['d',num2str(i),'=D_Hdata(i,4);']);
end

%The pose of the tool coordinate system in the coordinate system {4}
TTe=[1 0 0  0;
     0 1 0  0;
     0 0 1  0;
     0 0 0  1];
TT0=TT;
eT0=TT0*TTe^(-1);                          
eR0=eT0(1:3,1:3);                           
x=eT0(1,4);   y=eT0(2,4);  z=eT0(3,4);      %coordinate{4}'s position coordinates in the base coordinate system

%% Calculate k3 
zz=z-d1;
r=norm([x,y,zz]);
n=r^4+4*a1^2*(zz^2+a1^2);
if (2*r^2+4*a1^2)^2-4*n<0
    disp(['error1:k3<0']);
    return;
end
k3=[((2*r^2+4*a1^2)+sqrt((2*r^2+4*a1^2)^2-4*n))/2 ((2*r^2+4*a1^2)-sqrt((2*r^2+4*a1^2)^2-4*n))/2];

%% Calculate theta3 
m=a1^2+a2^2+a3^2+d4^2;
% theta3_test=pi/6;
% k3test=m+2*a2*a3*cos(theta3_test)-2*a2*d4*sin(theta3_test)
% k3=k3test;

j=1;
for i=1:length(k3)
    A3=(2*a2*d4)^2-(k3(i)-m)^2;
    B3=-8*a2^2*a3*d4;
    C3=(2*a2*a3)^2-(k3(i)-m)^2;
    if B3^2-4*A3*C3<0
        disp(['error2:theta3<0']);
        disp(['NO',num2str(i),'solution is invalid']);
        continue;
    end
    t3(:,j)=[(-B3+sqrt(B3^2-4*A3*C3))/(2*A3);(-B3-sqrt(B3^2-4*A3*C3))/(2*A3)];
    j=j+1;
end
theta3=[atan(t3) atan(t3)+pi];

%% Calculate theta2
f1=a3*cos(theta3)-d4*sin(theta3)+a2;
f2=a3*sin(theta3)+d4*cos(theta3);
f3=d3;

k1=f1;
k2=-f2;
k3=2*a2*a3*cos(theta3)-2*a2*d4*sin(theta3)+m;
k4=0;

t2=((r^2-k3).*k2-2*a1.*k1.*(zz-k4))./((r^2-k3).*k1+2*a1*k2.*(zz-k4));
theta2=[atan(t2) atan(t2)+pi];
theta3=[theta3 theta3];
f1=[f1 f1];
f2=[f2 f2];

%% Calculate theta1
g1=cos(theta2).*f1-sin(theta2).*f2+a1;
g2=0;
for j=1:size(theta2,1)
    for k=1:size(theta2,2)
        %         if imag(g1(j,k))==0
        theta1(j,k)=atan2(y/g1(j,k),x/g1(j,k));
        %         end
    end
end

q=[];
for j=1:size(theta1,1)
    for k=1:size(theta1,2)
        in=(j-1)*size(theta1,2)+k;
%         if length(find(index==in))~=0
%             continue;
%         end
        qt=[theta1(j,k);theta2(j,k);theta3(j,k);0];
        q=[q,qt];
        
    end
end
disp(['~~~~~~~~~~~~~Given tool coordinate system£º~~~~~~~~~~~~~~~']);
num2str(TT)
disp(['~~~~~~~~~~~~~Calculate the workpiece coordinate system£º~~~~~~~~~~~~~~~']);
disp(['']);

Q=[];
for j=1:size(q,2)
    %%
    %Calculated for each link to the base coordinate system labeled homogeneous transformation matrix T0
    iT0=eye(4);
    T0=zeros(4,4,N);              
    for i=1:N
        a=eval(['a',num2str(i-1)]);
        alpha=eval(['alpha',num2str(i-1)]);
        d=eval(['d',num2str(i)]);
        iT0=iT0*T(a,alpha,d,q(i,j));
    end
    %%
    %Calculate the tool coordinate system under the base coordinate system pose
    eT0_cal=iT0;
    TT0_cal=eT0_cal*TTe;
    if(norm(TT(:,end)-TT0_cal(:,end))<1e-3)
       Q=[Q,q(:,j)];
       disp(['Joint angle£º']);
       num2str(q(:,j)'*180/pi)
       disp(['End coordinate£º']);
       num2str(TT0_cal) 
    end
    
end

Q(end,:)=[];



