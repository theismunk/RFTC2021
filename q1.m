clc;clear; close all
%%

% Inputs
syms u_1(t) u_2(t) % Symbolic input declearation
ST_input(1,:) = {u_1, u_2}; % symbolic variable
ST_input(2,:) = {'u_1\\left(t\\right)', 'u_2\\left(t\\right)'}; % LaTeX expression


% Measurements
syms d(t) y_1(t) y_2(t) y_3(t) % Symbolic measurement declearation
ST_meas(1,:) = {d, y_1, y_2, y_3}; % symbolic variable
ST_meas(2,:) = {'d\\left(t\\right)', 'y_1\\left(t\\right)', 'y_2\\left(t\\right)', 'y_3\\left(t\\right)'}; % LaTeX expression


% Unknowns
syms theta_1(t) omega_1(t) theta_2(t) omega_2(t) theta_3(t) omega_3(t) ...
    dtheta_1(t) domega_1(t) dtheta_2(t) domega_2(t) dtheta_3(t) domega_3(t) % Symbolic unknowns declearation
ST_unknowns(1,:) = {theta_1, omega_1, theta_2, omega_2, theta_3, omega_3, ...
    dtheta_1, domega_1, dtheta_2, domega_2, dtheta_3, domega_3}; % symbolic variable
ST_unknowns(2,:) = {...
    '\\theta_1\\left(t\\right)', '\\omega_1\\left(t\\right)', ...
	'\\theta_2\\left(t\\right)','\\omega_2\\left(t\\right)',...
	'\\theta_3\\left(t\\right)','\\omega_3\\left(t\\right)',...
    '\\dot{\\theta_1}\\left(t\\right)','\\dot{\\omega_1}\\left(t\\right)',...
    '\\dot{\\theta_2}\\left(t\\right)','\\dot{\\omega_2}\\left(t\\right)',...
    '\\dot{\\theta_3}\\left(t\\right)','\\dot{\\omega_3}\\left(t\\right)'}; % LaTeX expression


% Parameters
syms J_1 b_1 k_1 J_2 b_2 k_2 J_3 b_3 % Symbolic parameters declaration
ST_parameters(1,:) = {J_1, J_2, J_3, b_1, b_2, b_3, k_1, k_2}; % symbolic variable
ST_parameters(2,:) = {'J_1', 'J_2', 'J_3', 'b_1', 'b_2', 'b_3', 'k_1', 'k_2'}; % LaTeX expression


% Enter the constraints of the system
% ST_cons(1.:) comprise the Matlab names of constraints
% ST_cons(2.:) comprise the Latex 
% ST_cons(3,:) comprise a cell array list with the expressions of the constraints

ST_cons(1,:) = {'c1','c2','c3','c4','c5', 'c6', ...
    'd7', 'd8', 'd9', 'd10', 'd11', 'd12',...
    'm13', 'm14', 'm15'}; % Constraint names
ST_cons(2,:) = {'c_1','c_2','c_3','c_4','c_5', 'c_6', ...
    'd_7', 'd_8', 'd_9', 'd_{10}', 'd_{11}', 'd_{12}',...
    'm_{13}', 'm_{14}', 'm_{15}'}; % Constraint latex names
ST_cons(3,:) = {...
    0 == dtheta_1 - omega_1,...
    0 == J_1*domega_1 - u_1 + b_1*omega_1 + k_1 * (theta_1 - theta_2) + d, ...
    0 == dtheta_2 - omega_2, ...
    0 == J_2 * domega_2 - u_2 + b_2 * omega_2 + k_1*(theta_2 - theta_1) + k_2*(theta_2-theta_3),...
    0 == dtheta_3 - omega_3,...
    0 == J_3 * domega_3 + b_3 * omega_3 + k_2*(theta_3 - theta_2),...
    0 == dtheta_1 - diff(theta_1,t),...
    0 == domega_1 - diff(omega_1,t),...
    0 == dtheta_2 - diff(theta_2,t),...
    0 == domega_2 - diff(omega_2,t),...
    0 == dtheta_3 - diff(theta_3,t),...
    0 == domega_3 - diff(omega_3,t),...
    0 == y_1 - theta_1,...
    0 == y_2 - theta_2,...
    0 == y_3 - theta_3};
% NOTE that "diff(.,t)" is used as the differential operator, D == d/dt.


ST_canfail  = [1:6, 13:15]; % diff constraint cannot fail
ST_domains  = ones(1,15);
% NOTE NOTE NOTE:
% Incidence matrix MUST have colums ordered as: 
% input, measurements, unknowns
% an x in the book is a "-1" in the software tool

% ST_IncMat = ...
%     [ 0 0 0 1 1 1;
%       1 0 0 0 1 0;
%       0 0 1 0 0 1;
%       0 1 1 0 0 0;
%       0 0 -1 1 0 0];
  
% Automatic incMatrix generation
cons_oneway = {[],[],[],[],[],[],{theta_1},{omega_1},{theta_2},{omega_2},...
    {theta_3},{omega_3},[],[],[]};
ST_IncMat = sa_incidencematrix(ST_cons,...
                                    ST_input,ST_meas,...
                                    ST_unknowns,cons_oneway);
						
								
ST_sys =...
    sa_create(ST_IncMat,ST_cons,...
    ST_input, ST_meas,ST_unknowns,...
    ST_domains, ST_canfail,...
	ST_parameters);



sa_disp(ST_sys);

ST_sys=sa_match(ST_sys,'rank');

% Autogenerate pdf report - requires LaTeX to be installed
sa_report(ST_sys,'q4_knowndist','pdf',true);
disp('A report has been generated with the following results:')

disp('Obtained matching:');
sa_disp(ST_sys, 't');

disp('Parity relation symbolic form:')
sa_disp(ST_sys, 's');

disp('Parity relation analytic form:')
sa_disp(ST_sys, 'a');