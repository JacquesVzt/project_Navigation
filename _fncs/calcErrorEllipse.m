function [major,minor,alpha] = calcErrorEllipse(C,confidence)
% [std_major,std_minor,alpha] = calcErrorEllipse(C,z)
% 
%   Calculate error ellipse from 2x2 covariance matrix
%
%   In:
%       C           2x2 covariance matrix or 2x2xn array of covariance
%                   matrices
%       confidence  error confidence level given as value between 0...1
% 
%   Out:
%       major       error ellipse semi-major axis
%       minor       error ellipse semi-minor axis
%       alpha       error ellipse orientation in degree
%
%   Other m-files required: none
%   Subfunctions: none
%   MAT-files required: none
%
%   See also: plotErrorEllipse

%   Author: Hanno Winter
%   email: hanno.winter@rmr.tu-darmstadt.de
%   Date: 13-May-2020; Last revision: 14-May-2020
%

%% Initialization and checks

if (size(C,1) ~= 2) || (size(C,2) ~= 2)
    error('calcErrorEllipse: Covariance matrix ''C'' is not valid!');
end

if (length(confidence) == 1) && (size(C,3) > 1)
    confidence = ones(1,size(C,3))*confidence;
elseif length(confidence) ~= size(C,3)
    error('calcErrorEllipse: Confidence array ''confidence'' has wrong dimension!');
end % if

%% Calculation

z = chi2inv(confidence,2);
major = zeros(1,size(C,3));
minor = zeros(1,size(C,3));
alpha = zeros(1,size(C,3));
for i = 1:size(C,3)
    [V,D] = eig(C(1:2,1:2,i));
    [lambda,lambda_index] = sort(diag(D),'descend');
    v_lambda = V(:,lambda_index);

    major(i) = sqrt(z(i)*abs(lambda(1)));
    minor(i) = sqrt(z(i)*abs(lambda(2)));
    alpha(i) = mod(atan2d(v_lambda(2,1),v_lambda(1,1)),360);    
end % if
    
end % function