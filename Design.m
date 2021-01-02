function over_all = Design(params)

    K1 = params.K1; %动臂长度l1与斗杆长度l2的比值 经验值
    Ks = params.Ks; %松散系数 取1.24~1.3 经验值
    q = 2.2;       %斗容量 设计参数
    b = params.b; %斗宽  经验值
    phi_max2 =params.phi_max2; % 转斗挖掘装满转角 一般取90~110 经验值

    l_3 = sqrt((2*q)/(b*(phi_max2- sin(phi_max2))*Ks)); %转斗挖掘半径

    l_kq = params.l_kq_r*l_3; %（0.3~0.38）*l3 经验值
    D_KQV = params.D_KQV;%一般在95°~105°  经验值
    alpha_10 = D_KQV;

    Y_A = params.Y_A; %A点坐标 经验值
    Z_A = params.Z_A;%A点坐标 经验值

    lambda_1 = params.lambda_1; %动臂油缸伸缩比 经验值
    L_1min=params.L_1min; %动臂油缸最短长度 经验值
    delta_L1= L_1min*(lambda_1-1); %动臂油缸行程
    L_1max =  L_1min * lambda_1; %动臂油缸最大长度
    l_5 = params.l_5_r * L_1min; %AC长度 经验值
    alpha_11=params.alpha_11; %AC与水平面倾角 经验值
    r_1=11.560; %最大挖掘半径 设计参数
    l_2 = (r_1 - l_3)/(1 + K1); %近似取r_1 = l_1+l_2+l_3
    l_1=K1 * l_2;

    alpha_1 = params.alpha_1; %动臂弯角 110°~170° 经验值
    K2 = params.K2; %上下动臂比值 l_UF / l_UC 经验值

    l_UC = l_1 / sqrt(1+ (K2)^2 - 2*K2* cos(alpha_1)); % 在三角形UCF中用余弦定理
    l_UF = K2 * l_UC;

    l_7 = l_UC; % 初选时就可将B点与U点重合
    D_BCF = acos((l_7^2 + l_1^2 - l_UF^2) / (2 * l_1 * l_7)); %∠BCF
    D_UCF = D_BCF;
    alpha_2 = D_BCF;

    %% 校核动臂三角形ABC
    if (l_5 + L_1min)>l_7 && abs(l_5 - l_7)<L_1min && abs(l_7 - L_1min) < l_5 ...
            && (l_5 + l_7)>L_1max && abs(l_5 - L_1max)<l_7 && abs(l_7 - L_1max)<l_5
    %     disp('动臂机构不存在干涉问题。')
    else
        over_all = 100;
        return
    %     error('动臂机构存在干涉！')
    end

    %% 动臂摆角
    phi_1max = acos((l_5^2 + l_7^2 - L_1max^2)/ (2 * l_5 * l_7)) - alpha_11 - alpha_2; % 动臂最大仰角
    phi_1min = acos((l_5^2 + l_7^2 - L_1min^2)/ (2 * l_5 * l_7)) - alpha_11 - alpha_2; % 动臂最大俯角
    delta_phi = phi_1max - phi_1min; % 动臂摆角的变化范围

    %% 反铲斗杆设计
    lambda_2 = params.lambda_2; % 斗杆油缸的伸缩比 经验值
    n_2 = 2; % 斗杆油缸数目
    D_2 = 0.195; % 缸径
    d_2 = 0.115; % 活塞杆直径
    theta_2max = params.theta_2max; % 斗杆的摆角范围 经验值

    l_9 = params.l_9; %斗杆油缸的最大力臂 经验值

    delta_L2max = 2 * l_9 * sin(theta_2max/2); 
    L_2min = delta_L2max / (lambda_2 - 1);
    L_2max = lambda_2 * L_2min;

    l_8 = sqrt(l_9^2 + L_2max^2 - 2 * l_9 * L_2max * sin(theta_2max/2)); % 斗杆油缸和动臂的铰接点D与斗杆和动臂铰接点F的距离

    D_EFQ = params.D_EFQ; %斗杆前后段夹角 经验值

    %% 校核斗杆三角形
    if (l_9 + L_2min)>l_8 && abs(l_9 - l_8)<L_2min && abs(l_8 - L_2min) < l_9 ...
            && (l_9 + l_8)>L_2max && abs(l_9 - L_2max)<l_8 && abs(l_8 - L_2max)<l_9
    %     disp('斗杆机构不存在干涉问题。')
    else
        over_all = 100;
        return
    %     disp('斗杆机构存在干涉！')
    end

    %% 斗杆摆角
    D_CFB = pi - alpha_1 - alpha_2; 
    D_DFB = params.D_DFB; % 初选值 经验值
    alpha_3 = D_DFB + D_CFB; % 动臂上的结构角CFD
    phi_2min = pi - alpha_3 - D_EFQ - acos((l_8^2 + l_9^2 - L_2max^2) / (2 * l_8 * l_9));
    phi_2max = pi - alpha_3 - D_EFQ - acos((l_8^2 + l_9^2 - L_2min^2) / (2 * l_8 * l_9));
    delta_phi_2 = phi_2max - phi_2min;

    %% 反铲铲斗连杆机构的设计
    D_3 = 0.200;% 铲斗油缸的缸径 
    d_3 = 0.085;%活塞杆缸径
    L_3min = params.L_3min;%铲斗油缸最小长度 经验值
    lambda_3=1.33; %铲斗油缸压缩比 经验值
    L_3max = L_3min * lambda_3;%铲斗油缸最大长度 经验值
    delta_L3 = L_3max - L_3min;
    l_MN = l_kq;
    l_13 = l_MN;
    l_MK = l_kq;
    l_NQ = params.l_NQ_r * l_MN; % 经验值
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%                                   反铲工作装置的运动分析                                     %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    cc = 0.45; % 动臂与机身铰接点C的横坐标 经验值
    bb = 0.9; % 动臂与机身铰接点C在转台坐标系上的纵坐标 经验值
    aa = 1.25; % 转台高度 经验值
    phi_0 = 0; % 转台绕z轴转动的角度
    % phi_3min = deg2rad(-135); % 铲斗转角最大值
    % phi_3max = deg2rad(45); % 铲斗转角最大值

    %% 回转平台的运动分析
    R_z0phi0 = [cos(phi_0), -sin(phi_0), 0;...
                        sin(phi_0),  cos(phi_0), 0;...
                             0      ,        0,          1]; % 转台相对于z0轴的旋转矩阵
    xyz_0A = [0; cc + l_5 * cos(alpha_11); bb - l_5 * sin(alpha_11)];
    xyz_0C = [0; cc; bb];
    XYZ_A = R_z0phi0 * xyz_0A + [0; 0; aa];
    XYZ_C = R_z0phi0 * xyz_0C + [0; 0; aa];

    %% 动臂的运动分析
    phi_1 = deg2rad(30); % 动臂瞬时摆角
    R_x1phi1 = [1, 0, 0;...
                        0,  cos(phi_1), -sin(phi_1);...
                        0,   sin(phi_1), cos(phi_1)]; % 动臂上各铰接点绕x1轴的旋转矩阵
    xyz_1B = [0; l_7*cos(alpha_2); l_7*sin(alpha_2)];                
    XYZ_B = R_z0phi0 * (xyz_0C + R_x1phi1 * xyz_1B) + [0; 0; aa];
    xyz_1D = [0; l_1 - l_8*cos(alpha_3); l_8 * sin(alpha_3)];                
    XYZ_D = R_z0phi0 * (xyz_0C + R_x1phi1 * xyz_1D) + [0; 0; aa];
    xyz_1F = [0; l_1; 0];      
    XYZ_F = R_z0phi0 * (xyz_0C + R_x1phi1 * xyz_1F) + [0; 0; aa];

    %% 斗杆的运动分析
    phi_2 = deg2rad(-60); % 斗杆瞬时摆角
    R_x2phi2 = [1, 0, 0;...
                        0,  cos(phi_2), -sin(phi_2);...
                        0,   sin(phi_2), cos(phi_2)]; % 斗杆上各铰接点绕x2轴的旋转矩阵
    xyz_2E = [0; l_9 * cos(D_EFQ); l_9 * sin(D_EFQ)];
    XYZ_E = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * xyz_2E)) + [0; 0; aa];

    l_GF = params.l_GF; %经验值
    l_10 = l_GF;
    D_GFQ = params.D_GFQ; %经验值
    xyz_2G = [0; l_GF * cos(D_GFQ); l_GF * sin(D_GFQ)];
    XYZ_G = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * xyz_2G)) + [0; 0; aa];

    z_2N = 0.05; %经验值
    xyz_2N = [0; l_2 - sqrt(l_NQ^2 - z_2N^2); z_2N];
    XYZ_N = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * xyz_2N)) + [0; 0; aa];

    xyz_2Q = [0; l_2; 0];
    XYZ_Q = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * xyz_2Q)) + [0; 0; aa];

    %% 铲斗及铲斗连杆机构的运动分析
    l_GN = sqrt(sum((xyz_2G-xyz_2N).^2));
    l_11 = l_GN;
    theta_3min = acos((l_11^2 + l_13^2 - L_3min^2) / (2 * l_11 * l_13));
    theta_3max = acos((l_11^2 + l_13^2 - L_3max^2) / (2 * l_11 * l_13));
    % syms theta_3 % 铲斗油缸瞬时长度
    alpha_5 =acos( sum( (-xyz_2N) .* (xyz_2G - xyz_2N) ) / (sqrt(sum((-xyz_2N).^2)) * sqrt(sum((xyz_2G - xyz_2N).^2))) );
    D_FNQ =acos( sum( (-xyz_2N) .* (xyz_2Q - xyz_2N) ) / (sqrt(sum((-xyz_2N).^2)) * sqrt(sum((xyz_2Q - xyz_2N).^2))) );
    alpha_7 =acos( sum( (-xyz_2Q) .* (xyz_2N - xyz_2Q) ) / (sqrt(sum((-xyz_2Q).^2)) * sqrt(sum((xyz_2N - xyz_2Q).^2))) );
    phi_4 = - theta_3min + 2*pi - alpha_5 - D_FNQ - alpha_7;

    xyz_2M = [0; xyz_2N(2) + l_13 * cos(phi_4); xyz_2N(3) + l_13 * sin(phi_4)];
    XYZ_M = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * xyz_2M)) + [0; 0; aa]; 

    l_NQ = sqrt(sum((xyz_2N - xyz_2Q).^2));
    l_MQ_max = sqrt(l_NQ^2 + l_MN^2 - 2 * l_NQ * l_MN * cos(2*pi - theta_3min - alpha_5 - D_FNQ));
    l_MQ_min = sqrt(l_NQ^2 + l_MN^2 - 2 * l_NQ * l_MN * cos(2*pi - theta_3max - alpha_5 - D_FNQ));
    D_NQM_min = acos((l_NQ^2 + l_MQ_max^2 - l_MN^2) / (2 * l_NQ * l_MQ_max));
    D_MQK_min = acos((l_kq^2 + l_MQ_max^2 - l_MK^2) / (2 * l_kq * l_MQ_max));
    phi_3max = -(alpha_7 + D_NQM_min + D_MQK_min + alpha_10 - pi);
    D_NQM_max = acos((l_NQ^2 + l_MQ_min^2 - l_MN^2) / (2 * l_NQ * l_MQ_min));
    D_MQK_max = acos((l_kq^2 + l_MQ_min^2 - l_MK^2) / (2 * l_kq * l_MQ_min));
    phi_3min = -(alpha_7 + D_NQM_max + D_MQK_max + alpha_10 - pi);

    phi_3 = 0; %铲斗瞬时摆角

    R_x3phi3 = [1, 0, 0;...
                        0,  cos(phi_3), -sin(phi_3);...
                        0,   sin(phi_3), cos(phi_3)]; % 铲斗上各铰接点绕x3轴的旋转矩阵
    xyz_3K = [0; l_kq * cos(alpha_10); l_kq * sin(alpha_10)];
    XYZ_K = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * (xyz_2Q + R_x3phi3 * xyz_3K))) + [0; 0; aa]; 

    xyz_3V = [0; l_3; 0];
    XYZ_V = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * (xyz_2Q + R_x3phi3 * xyz_3V))) + [0; 0; aa]; 

    %% 反铲工作装置作业参数
    h_1 = aa + bb + l_1 * sin(phi_1min) - l_2 - l_3; %最大挖掘深度
    h_2 = aa + bb + l_1 * sin(phi_1max) + l_2 * sin(phi_1max+phi_2max) + l_3 * sin(phi_1max+phi_2max+phi_3max); %最大挖掘高度
    l_CQmax = sqrt(l_1^2 + l_2^2 - 2 * l_1 * l_2 * cos(pi + phi_2max));
    r_1 = cc + l_CQmax + l_3; % 最大挖掘半径

    gamma = atan((aa + bb)/(sqrt((l_CQmax + l_3)^2 - (aa + bb)^2)));
    r_0 = cc + (l_CQmax + l_3) * cos(gamma); %停机面上最大挖掘半径

    h_3 = aa + bb + l_1 * sin(phi_1max) + l_2 * sin(phi_1max + phi_2max) - l_3; %最大卸载高度

    zeta = params.zeta; % 经验值
    phi_30 = -(pi/2 - zeta);

    if (phi_1min+phi_2max+phi_3max) < phi_30
        phi_1 = phi_30 - phi_2max - phi_3max;
        h_4 = aa + bb + l_1 * sin(phi_1) + l_2 * sin(phi_1 + phi_2max) - l_3 * cos(zeta);
        %最大垂直挖掘深度
    else
        % case 1 
        phi_3 = phi_30 - phi_1min - phi_2max;
        h_4_temp1 = aa + bb +l_1 * sin(phi_1min) + l_2 * sin(phi_1min + phi_2max) - l_3 * cos(zeta);
        % case 2
        phi_2 = phi_30 - phi_1min - phi_3max;
        h_4_temp2 = aa + bb +l_1 * sin(phi_1min) + l_2 * sin(phi_30 - phi_3max) - l_3 * cos(zeta);    
        %case 3
        phi = [0;0];
        A = [-1, -1; 1, 0; 0, 1];
        b = [-phi_30 + phi_1min; phi_2max; phi_3max];
        lb = [phi_2min; phi_3min];
        option=optimset; option.LargeScale='off';option.Display='off';
        
%         [phi_best, h_4_temp3] = fmincon('fop', phi, A, b, [ ], [ ], lb, [ ], [ ], option)
        
        h_4 = min(h_4_temp1, h_4_temp2);
        
    end

    h_5 = aa + bb + l_1 * sin(phi_1min) - sqrt((l_2 + l_3)^2 - 1.5625);
    D_V1FV2 = 2 * atan(1.25/(sqrt((l_2 + l_3)^2 - 1.5625)));

    h_1ref = 7.285;
    h_2ref = 10.705;
    r_1ref = 11.56;
    r_0ref = 11.47;
    h_3ref = 7.445;
    h_4ref = 5.495;
    h_5ref = 6.846;

    Delta_h_1 = (abs(h_1) - h_1ref)/h_1ref;
    Delta_h_2 = (abs(h_2) - h_2ref)/h_2ref;
    Delta_r_1 = (abs(r_1) - r_1ref)/r_1ref;
    Delta_r_0 = (abs(r_0) - r_0ref)/r_1ref;
    Delta_h_3 = (abs(h_3) - h_3ref)/h_3ref;
    Delta_h_4 = (abs(h_4) - h_4ref)/h_4ref;
    
    
    Delta_phi_3 = (rad2deg(phi_3max-phi_3min) - 180)/180;
    over_all = Delta_h_1^2 + Delta_h_2^2 + Delta_r_1^2 + Delta_r_0^2 + Delta_h_3^2 + Delta_h_4^2 + Delta_phi_3^2;
    % Delta_h_5 = (abs(h_5) - h_5ref)/h_5ref;
    if isreal(over_all)==0
        over_all = 100; %如果最终over_all 含虚数，则无效
    end
    
    savevideo = true;
    texttext = false;
    if params.draw %如果draw为true则画包络图
        results.cc = cc;
        results.bb = bb;
        results.aa = aa;
        results.l_7 = l_7;
        results.alpha_2 = alpha_2;
        results.l_1 = l_1;
        results.l_2 = l_2;
        results.l_3 = l_3; % 画包络图所需的参数

        %% 包络图绘制
        figure(1);
        m = moviein(10);
        frame = 0; %帧序号
        for ii = 1: 8 %包络图分8段
            switch(ii)
                case 1 %V1~V2段动臂油缸最长、斗杆油缸最短、铲斗油缸短到长
                    for j = phi_3max : -0.0175 : phi_3min %步长为0.0175个弧度，即1°

                        frame = frame + 1;
                        clf;
                        [C1, B1, F1, Q1, V1] = convert(phi_0, phi_1max, phi_2max, j, results);
                        YZ = [C1, B1, F1, Q1, V1];
                        if frame == 1
                            Limit_position{1} = YZ;
                        end
                        Y = YZ(1,:);
                        Z = YZ(2,:);
                        nib_Y(frame) = V1(1); %存下每一帧中铲斗尖V点的位置
                        nib_Z(frame) = V1(2); %存下每一帧中铲斗尖V点的位置
                        Curve(1)= 1;
                        Curve(2)= frame;
                        DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
                        m(frame) = getframe;
                    end
                    Limit_position{end+1} = YZ;
                    
                case 2 %V2~V3段动臂油缸最长、斗杆油缸由短到长、铲斗油缸最长
                    for j = phi_2max : -0.0175 : phi_2min %步长为0.0175个弧度，即1°
                        frame = frame + 1;
                        clf;
                        [C1, B1, F1, Q1, V1] = convert(phi_0, phi_1max, j, phi_3min, results);
                        YZ = [C1, B1, F1, Q1, V1];
                        Y = YZ(1,:);
                        Z = YZ(2,:);
                        nib_Y(frame) = V1(1); %存下每一帧中铲斗尖V点的位置
                        nib_Z(frame) = V1(2); %存下每一帧中铲斗尖V点的位置
                        Curve(3)= frame;
                        DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
                        m(frame) = getframe;
                    end
                    Limit_position{end+1} = YZ;
                    
                case 3 %V3~V4段动臂油缸最长、斗杆油缸最长、铲斗油缸由最长缩至斗齿尖位于C、Q2连线上
                    j = phi_3min;
                    while V1(2) > ((C1(2) - Q1(2)) / (C1(1) - Q1(1))) * V1(1) + (C1(2) - ((C1(2) - Q1(2)) / (C1(1) - Q1(1)))*C1(1)) && j < phi_3max
                        j = j + 0.0175; %上式成立即说明V点还在CQ连线之上，所以继续循环
                        frame = frame + 1;
                        clf;
                        [C1, B1, F1, Q1, V1] = convert(phi_0, phi_1max, phi_2min, j, results);
                        YZ = [C1, B1, F1, Q1, V1];
                        Y = YZ(1,:);
                        Z = YZ(2,:);
                        nib_Y(frame) = V1(1); %存下每一帧中铲斗尖V点的位置
                        nib_Z(frame) = V1(2); %存下每一帧中铲斗尖V点的位置
                        Curve(4)= frame;
                        
                        DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
                        m(frame) = getframe;
                    end
                    Limit_position{end+1} = YZ;
                    Current_phi_3 = j;
                    
                case 4 %V4~V5段动臂油缸由长到短、斗杆油缸最长、铲斗油缸固定
                    for j = phi_1max : -0.0175 : phi_1min %步长为0.0175个弧度，即1°
                        frame = frame + 1;
                        clf;
                        [C1, B1, F1, Q1, V1] = convert(phi_0, j, phi_2min, Current_phi_3, results);
                        YZ = [C1, B1, F1, Q1, V1];
                        Y = YZ(1,:);
                        Z = YZ(2,:);
                        nib_Y(frame) = V1(1); %存下每一帧中铲斗尖V点的位置
                        nib_Z(frame) = V1(2); %存下每一帧中铲斗尖V点的位置
                        Curve(5)= frame;
                        DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
                        m(frame) = getframe;
                    end
                    Limit_position{end+1} = YZ;
                    
                case 5 %V5~V6段动臂油缸最短、斗杆油缸最长、铲斗油缸由原固定值缩至FQV三点一线
                    j = Current_phi_3;
                    while V1(2) > ((F1(2) - Q1(2)) / (F1(1) - Q1(1))) * V1(1) + (F1(2) - ((F1(2) - Q1(2)) / (F1(1) - Q1(1)))*F1(1)) && j<phi_3max
                        j = j + 0.0175; %上式成立即说明V点还在CQ连线之上，所以继续循环
                        frame = frame + 1;
                        clf;
                        [C1, B1, F1, Q1, V1] = convert(phi_0, phi_1min, phi_2min, j, results);
                        YZ = [C1, B1, F1, Q1, V1];
                        Y = YZ(1,:);
                        Z = YZ(2,:);
                        nib_Y(frame) = V1(1); %存下每一帧中铲斗尖V点的位置
                        nib_Z(frame) = V1(2); %存下每一帧中铲斗尖V点的位置
                        Curve(6)= frame;
                        DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
                        m(frame) = getframe;
                    end
                    Current_phi_3 = j;
                    Limit_position{end+1} = YZ;
                    
                case 6 %V6~V7段动臂油缸最短、斗杆油缸由长到短、铲斗油缸固定
                    for j = phi_2min : 0.0175 : phi_2max %步长为0.0175个弧度，即1°
                        frame = frame + 1;
                        clf;
                        [C1, B1, F1, Q1, V1] = convert(phi_0, phi_1min, j, Current_phi_3, results);
                        YZ = [C1, B1, F1, Q1, V1];
                        Y = YZ(1,:);
                        Z = YZ(2,:);
                        nib_Y(frame) = V1(1); %存下每一帧中铲斗尖V点的位置
                        nib_Z(frame) = V1(2); %存下每一帧中铲斗尖V点的位置
                        Curve(7)= frame;
                        DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
                        m(frame) = getframe;
                    end
                    Limit_position{end+1} = YZ;
                    
                case 7 %V7~V8段动臂油缸最短、斗杆油缸最短、铲斗油缸由原固定值缩至CQV三点一线
                    j = Current_phi_3;
                    while V1(2) < ((C1(2) - Q1(2)) / (C1(1) - Q1(1))) * V1(1) + (C1(2) - ((C1(2) - Q1(2)) / (C1(1) - Q1(1)))*C1(1)) && j<phi_3max
                        j = j + 0.0175; %上式成立即说明V点还在CQ连线之上，所以继续循环
                        frame = frame + 1;
                        clf;
                        [C1, B1, F1, Q1, V1] = convert(phi_0, phi_1min, phi_2max, j, results);
                        YZ = [C1, B1, F1, Q1, V1];
                        Y = YZ(1,:);
                        Z = YZ(2,:);
                        nib_Y(frame) = V1(1); %存下每一帧中铲斗尖V点的位置
                        nib_Z(frame) = V1(2); %存下每一帧中铲斗尖V点的位置
                        Curve(8)= frame;
                        DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
                        m(frame) = getframe;
                        
                    end
                    Current_phi_3 = j;
                    Limit_position{end+1} = YZ;
                    
                case 8 %V8~V9段动臂油缸由短到长、斗杆油缸最短、铲斗油缸固定
                    for j = phi_1min : 0.0175 : phi_1max %步长为0.0175个弧度，即1°
                        frame = frame + 1;
                        clf;
                        [C1, B1, F1, Q1, V1] = convert(phi_0, j, phi_2max, Current_phi_3, results);
                        YZ = [C1, B1, F1, Q1, V1];
                        Y = YZ(1,:);
                        Z = YZ(2,:);
                        nib_Y(frame) = V1(1); %存下每一帧中铲斗尖V点的位置
                        nib_Z(frame) = V1(2); %存下每一帧中铲斗尖V点的位置
                        Curve(9)= frame;
                        DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
                        m(frame) = getframe;
                        
                    end
                    Limit_position{end+1} = YZ;
                    
            end
        end
        movie(m,1,100);
        save m
        
        % 保存最终的图
        figure(2)
%         DrawPoints(Y, Z, nib_Y, nib_Z, Curve);
        DrawV(nib_Y, nib_Z, Curve)
        for vvv = 1 : length(Limit_position)
            DrawPoints(Limit_position{vvv}(1,:), Limit_position{vvv}(2,:), nib_Y, nib_Z, Curve);
        end
        set(gcf,'color','none');                     %这三句设置没有背景颜色
        set(gca,'color','none');
        set(gcf,'InvertHardCopy','off');
        export_fig D:\Files\Master\课程\工程机械设计（二）\包络图.png -transparent %%改名
        
        if savevideo
            video = ['test.avi'];
            if exist(video, 'file') % 文件存在返回2
                delete(video); % 保证创建视频对象时开始时其为空
            end
            video_object = VideoWriter(video);
            video_object.FrameRate = 100;
            open(video_object);
            for frame = 1: size(m,2)
                frames = m(frame).cdata;
                writeVideo(video_object, frames); % 写入内容
            end
            close(video_object); %关闭文件
        end
    end
end
    



function [C, B, F, Q, V] = convert(phi_0, phi_1, phi_2, phi_3, results)
    % 参数赋值
    cc = results.cc;
    bb = results.bb;
    aa = results.aa;
    l_7 = results.l_7;
    alpha_2 = results.alpha_2;
    l_1 = results.l_1;
    l_2 = results.l_2;
    l_3 = results.l_3;

    %% 回转平台的运动分析
    R_z0phi0 = [cos(phi_0), -sin(phi_0), 0;...
                       sin(phi_0),  cos(phi_0), 0;...
                            0      ,        0,          1]; % 转台相对于z0轴的旋转矩阵
    xyz_0C = [0; cc; bb];
    XYZ_C = R_z0phi0 * xyz_0C + [0; 0; aa];
    C = XYZ_C(2:3);

    R_x1phi1 = [1, 0, 0;...
                    0,  cos(phi_1), -sin(phi_1);...
                    0,   sin(phi_1), cos(phi_1)]; % 动臂上各铰接点绕x1轴的旋转矩阵
    xyz_1B = [0; l_7*cos(alpha_2); l_7*sin(alpha_2)];                
    XYZ_B = R_z0phi0 * (xyz_0C + R_x1phi1 * xyz_1B) + [0; 0; aa];
    B = XYZ_B(2:3);
    
    xyz_1F = [0; l_1; 0];      
    XYZ_F = R_z0phi0 * (xyz_0C + R_x1phi1 * xyz_1F) + [0; 0; aa];
    F = XYZ_F(2:3);
    
    R_x2phi2 = [1, 0, 0;...
                    0,  cos(phi_2), -sin(phi_2);...
                    0,   sin(phi_2), cos(phi_2)]; % 斗杆上各铰接点绕x2轴的旋转矩阵
    xyz_2Q = [0; l_2; 0];
    XYZ_Q = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * xyz_2Q)) + [0; 0; aa];
    Q = XYZ_Q(2:3);
    
    R_x3phi3 = [1, 0, 0;...
                    0,  cos(phi_3), -sin(phi_3);...
                    0,   sin(phi_3), cos(phi_3)]; % 铲斗上各铰接点绕x3轴的旋转矩阵
    xyz_3V = [0; l_3; 0];
    XYZ_V = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * (xyz_2Q + R_x3phi3 * xyz_3V))) + [0; 0; aa]; 
    V = XYZ_V(2:3);
end

function DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
    plot(Y, Z);
    hold on
    plot(nib_Y, nib_Z); % 画包络图
    grid on;hold on;
    plot(Y(1),Z(1),'o');
    plot(Y(3),Z(3),'o');
    plot(Y(4),Z(4),'o');
    plot(Y(5),Z(5),'o');
    texttext = false;
    if texttext
        text(Y(5)+0.25,Z(5)+0.2,'V');
        text(Y(4)+0.25,Z(4)+0.2,'Q');
        text(Y(3)+0.25,Z(3)+0.2,'F');
        text(Y(2)+0.25,Z(2)+0.2, 'B');
        text(Y(1)+0.25,Z(1)+0.2, 'C');
    end
    axis equal
    axis([-2 14 -8 11]); 
    xlabel('Y (m)')
    ylabel('Z (m)')
    title('挖掘包络图');
    DrawV(nib_Y, nib_Z, Curve(1:end-1))
end

function DrawV(nib_Y, nib_Z, Curve)
    hold on
%     plot(nib_Y(Curve), nib_Z(Curve)); % 画包络图
    plot(nib_Y(Curve), nib_Z(Curve),'x');
    for vv = 1:length(Curve)
%         text(nib_Y(Curve(vv))+0.25,nib_Z(Curve(vv))+0.2, ['V_', num2str(vv)]);
    end
end