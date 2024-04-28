function ex4b_hw4_me599_MPC_sfunction(block)
%   MPC s-function for lecture 17
%   Written by Chris Vermillion - Built using the MATLAB template as a
%   starting point

%%
%% The setup method is used to set up the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.
%%
setup(block);

%endfunction

%% Function: setup ===================================================

function setup(block)

% Register number of ports
block.NumInputPorts  = 2;
block.NumOutputPorts = 1;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Register parameters
block.NumDialogPrms     = 0;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [-1, 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%   Register all relevant methods
block.RegBlockMethod('Outputs', @Outputs);     % Required
%block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C-MEX counterpart: mdlOutputs
%%
function Outputs(block)

%   Extract x and y postions from input data
x = block.InputPort(1).Data;
v = block.InputPort(2).Data;

%   Define the horizon length and time step
N = 40;
T_MPC = 0.5;

% %   Define the radius and center of the obstacle to be avoided
% R = 0.25;
% xc = 0.5;
% yc = 0;
% buffer = 0.1;

%   Specify an initial guess (control input = angular rate)
u0 = 0*ones(N,1);

% u_min = -5;
% u_max = 5;

% %   Use the top line when imposing a hard constraint on "mountain
% %   avoidance" - use the bottom line for a soft constraint (penalty)
% u_opt = fmincon(@(u)lec17_2023_objective(u,x_pos,y_pos,psi,N,T_MPC),u0,[],[],[],[],u_min,u_max,@(u)lec17_2023_constraint(u,x_pos,y_pos,psi,N,T_MPC,R,buffer,xc,yc),options);
% %Trajectory optimization (Direct shooting)
options = optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunctionEvaluations',120000); 
obj_fun = @(u)ex4b_hw4_me599_objective(u,x,v,N,T_MPC);    %   Name of objective function
nl_constraint = @(u)ex4b_hw4_me599_constraint(u,x,v,N,T_MPC);     %   Name of constraint function
u_opt = fmincon(obj_fun,u0,[],[],[],[],[],[],nl_constraint,options);

%   MPC output is the first step of the optimized control trajectory
block.OutputPort(1).Data = u_opt(1);

%end Outputs


%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C-MEX counterpart: mdlTerminate
%%
function Terminate(block)

%end Terminate

