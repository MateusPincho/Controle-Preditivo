%% Lecture 2: Introduction to Model Predictive Control
% Tutorial 02: demonstrate how to obtain adiscrete-time state-space model 
% from a continuous-time state-spacemodel, and form the augmented 
% discrete-time state-space model

% Form a continuous-time state variable mode
Ac = [0 1 0; 3 0 1; 0 1 0];
Bc = [1;1;3];
Cc = [0 1 0];
Dc = zeros(1,1);

Gss_c = ss(Ac,Bc,Cc,Dc);

% Discretize this model using 'c2d' function
DeltaT = 1;
Gss_d = c2d(Gss_c,DeltaT);

% Produce the augmented state-space model 
Gss_e = ss_augmented_model(Gss_d);


