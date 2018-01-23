function TT0=fkine_self(q)
%% Enter the joint angle, calculate the pose of the tool coordinate system
% q-joint angle£¬degree
theta1=q(1);
theta2=q(2);
theta3=q(3);
theta4=0;

syms a0 a1 a2;
syms alpha0 alpha1 alpha2;
syms d1 d2 d3;

%the number of robot joints
N=4;

%% 
%Read robot D-H parameters and assign values
[D_Hdata,string]=xlsread('D-H parameters',1);

% global D_Hdata;
for i=1:N
    eval(['a',num2str(i-1),'=D_Hdata(i,2);']);
    eval(['alpha',num2str(i-1),'=D_Hdata(i,3)*pi/180;']);
    eval(['d',num2str(i),'=D_Hdata(i,4);']);
end

%% 
%Calculated for each link to the base coordinate system labeled homogeneous transformation matrix T0
iT0=eye(4);
% T0=sym(zeros(4,4,N));
T0=zeros(4,4,N);              %Each connecting rod coordinate system to the base coordinate transformation matrix homogeneous
for i=1:N
    a=eval(['a',num2str(i-1)]);
    alpha=eval(['alpha',num2str(i-1)]);
    d=eval(['d',num2str(i)]);
    theta=eval(['theta',num2str(i)])*pi/180;
    
    iT0=iT0*T(a,alpha,d,theta);
    T0(:,:,i)=iT0;
end

%% Calculate the tool coordinate system under the base coordinate system pose
eT0=iT0;
%The vector of the tool coordinate system in the end coordinate system
% TTe=[1 0  0 0;
%      0 0  1 150;
%      0 -1 0 0;
%      0 0  0 1];
TT0=eT0;


%% Draw the coordinate system
% figure;
plot3([0 T0(1,end,1)],[0 T0(2,end,1)],[0 T0(3,end,1)],'-o','linewidth',5,'markersize',10);
hold on;
for i=1:N-1
    plot3([T0(1,end,i) T0(1,end,i+1)],[T0(2,end,i) T0(2,end,i+1)],[T0(3,end,i) T0(3,end,i+1)],'-o','linewidth',5,'markersize',10);
end
 plot3([T0(1,end,i+1) TT0(1,end)],[T0(2,end,i+1) TT0(2,end)],[T0(3,end,i+1) TT0(3,end)],'-o','linewidth',5,'markersize',10);
grid on;
axis equal;
xlabel('x');
ylabel('y');
zlabel('z');

