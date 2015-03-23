%% Quadrotor mathematical model
%
%  (c) Luca Cavanini, Gionata Cimini. Università Politecnica delle Marche 2015



function [sys,z0,str,ts] = quadrotor_linear(t,Z,U,flag,x,y,z,phi,theta,psy,u,v,w,p,q,r)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,z0,str,ts]=mdlInitializeSizes(x,y,z,phi,theta,psy,u,v,w,p,q,r);
    
  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,Z,U);
  
  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,Z,U);

  
  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case {2,4,9}
      sys=[];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end





%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,z0,str,ts]=mdlInitializeSizes(x,y,z,phi,theta,psy,u,v,w,p,q,r)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%

sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
z0  = [x,y,z,phi,theta,psy,u,v,w,p,q,r]';

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];





%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,X,U)

parameters;

x=X(1);
y=X(2);
z=X(3);

%%%%%%%%%%%%%%%%%%%

phi=X(4);
theta=X(5);
psy=X(6);

%%%%%%%%%%%%%%%%%%%
u=X(7);
v=X(8);
w=X(9);

%%%%%%%%%%%%%%%%%%%

p=X(10);
q=X(11);
r=X(12);


omega1=U(1);
omega2=U(2);
omega3=U(3);
omega4=U(4);

a1=(2*l*b/Ix)*omega0;

a2=(2*d/Iz)*omega0;

a3=(2*b/m)*omega0;

   B=[ 0  0   0   0   ;
       0  0   0   0   ;
       0  0   0   0   ;
       0  0   0   0   ;
       0  0   0   0   ;
       0  0   0   0   ;
       0  0   0   0   ;
       0  0   0   0   ;
       a3 a3 a3  a3   ;   
       0 a1   0   -a1 ;
       a1 0   -a1 0   ;
       a2 -a2 a2  -a2 ];

dX=A*X+B*U+N;

sys = dX;

% end mdlDerivatives

%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,Z,U)


sys = Z;

% end mdlOutputs

%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,Z,U)

sys = [];

% end mdlTerminate
