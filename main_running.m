clear all
clc
params.K1 = [1.8]; %动臂长度l1与斗杆长度l2的比值 
params.Ks = [1.26]; %松散系数 1.24:1.3
params.b = [1.9]; %斗宽
params.phi_max2 = deg2rad([100]); %90~110
params.l_kq_r = [0.3]; %0.3~0.38
params.D_KQV = deg2rad([105]); %95~105
params.Y_A = [0.8];
params.Z_A = [1.425];
params.lambda_1 = [1.7]; % 1.6~1.7 特殊1.75
params.L_1min = [2];
params.l_5_r = [0.54]; % 0.5~0.6
params.alpha_11 = deg2rad([55]); % >50
params.alpha_1 = deg2rad([137]); % 110~170

params.lambda_2 =  [1.7]; % 1.6~1.7 
params.theta_2max = deg2rad([109]); %105~125
params.l_9 = 0.88;
params.D_EFQ = deg2rad([131]); %130~170
params.D_DFB = deg2rad(8);
params.K2 = [1.6]; % 1.1~1.3
params.L_3min = 2.45;
params.lambda_3 = [1.7]; %1.5~1.7
params.l_NQ_r = [0.8]; %0.7~0.8

params.l_GF = [0.7];
params.D_GFQ = deg2rad([50]);
params.zeta = deg2rad([53]);

params.draw = false; %是否画包络图

f = fieldnames(params);
nf = numel(f);
sz = NaN(nf, 1);

% Loop over all parameters to get sizes
for jj = 1: nf
    sz(jj) = numel( params.(f{jj}) );
end

%Loop for every combination of parameters
idx = cell(1,nf);
check = prod(sz);
tic
for ii = 1:check
    % Use ind2sub to switch from a linear index to the combination set
    [idx{:}] = ind2sub( sz, ii );
    % Create currentParam from the combination indices
    currentParam = struct();
    for jj = 1:nf
        currentParam.(f{jj}) = params.(f{jj})(idx{jj});
    end
    over_all(ii) = Design(currentParam);
    if over_all(ii) == min(over_all)
        [best, num] = min(over_all);
        opt = currentParam;
    end
    if mod(ii,100000) == 0
%         toc
        best
        jindu = ii / check
    end
    
    
end
save best