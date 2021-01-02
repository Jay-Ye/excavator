function over_all = Design(params)

    K1 = params.K1; %���۳���l1�붷�˳���l2�ı�ֵ ����ֵ
    Ks = params.Ks; %��ɢϵ�� ȡ1.24~1.3 ����ֵ
    q = 2.2;       %������ ��Ʋ���
    b = params.b; %����  ����ֵ
    phi_max2 =params.phi_max2; % ת���ھ�װ��ת�� һ��ȡ90~110 ����ֵ

    l_3 = sqrt((2*q)/(b*(phi_max2- sin(phi_max2))*Ks)); %ת���ھ�뾶

    l_kq = params.l_kq_r*l_3; %��0.3~0.38��*l3 ����ֵ
    D_KQV = params.D_KQV;%һ����95��~105��  ����ֵ
    alpha_10 = D_KQV;

    Y_A = params.Y_A; %A������ ����ֵ
    Z_A = params.Z_A;%A������ ����ֵ

    lambda_1 = params.lambda_1; %�����͸������� ����ֵ
    L_1min=params.L_1min; %�����͸���̳��� ����ֵ
    delta_L1= L_1min*(lambda_1-1); %�����͸��г�
    L_1max =  L_1min * lambda_1; %�����͸���󳤶�
    l_5 = params.l_5_r * L_1min; %AC���� ����ֵ
    alpha_11=params.alpha_11; %AC��ˮƽ����� ����ֵ
    r_1=11.560; %����ھ�뾶 ��Ʋ���
    l_2 = (r_1 - l_3)/(1 + K1); %����ȡr_1 = l_1+l_2+l_3
    l_1=K1 * l_2;

    alpha_1 = params.alpha_1; %������� 110��~170�� ����ֵ
    K2 = params.K2; %���¶��۱�ֵ l_UF / l_UC ����ֵ

    l_UC = l_1 / sqrt(1+ (K2)^2 - 2*K2* cos(alpha_1)); % ��������UCF�������Ҷ���
    l_UF = K2 * l_UC;

    l_7 = l_UC; % ��ѡʱ�Ϳɽ�B����U���غ�
    D_BCF = acos((l_7^2 + l_1^2 - l_UF^2) / (2 * l_1 * l_7)); %��BCF
    D_UCF = D_BCF;
    alpha_2 = D_BCF;

    %% У�˶���������ABC
    if (l_5 + L_1min)>l_7 && abs(l_5 - l_7)<L_1min && abs(l_7 - L_1min) < l_5 ...
            && (l_5 + l_7)>L_1max && abs(l_5 - L_1max)<l_7 && abs(l_7 - L_1max)<l_5
    %     disp('���ۻ��������ڸ������⡣')
    else
        over_all = 100;
        return
    %     error('���ۻ������ڸ��棡')
    end

    %% ���۰ڽ�
    phi_1max = acos((l_5^2 + l_7^2 - L_1max^2)/ (2 * l_5 * l_7)) - alpha_11 - alpha_2; % �����������
    phi_1min = acos((l_5^2 + l_7^2 - L_1min^2)/ (2 * l_5 * l_7)) - alpha_11 - alpha_2; % ������󸩽�
    delta_phi = phi_1max - phi_1min; % ���۰ڽǵı仯��Χ

    %% �����������
    lambda_2 = params.lambda_2; % �����͸׵������� ����ֵ
    n_2 = 2; % �����͸���Ŀ
    D_2 = 0.195; % �׾�
    d_2 = 0.115; % ������ֱ��
    theta_2max = params.theta_2max; % ���˵İڽǷ�Χ ����ֵ

    l_9 = params.l_9; %�����͸׵�������� ����ֵ

    delta_L2max = 2 * l_9 * sin(theta_2max/2); 
    L_2min = delta_L2max / (lambda_2 - 1);
    L_2max = lambda_2 * L_2min;

    l_8 = sqrt(l_9^2 + L_2max^2 - 2 * l_9 * L_2max * sin(theta_2max/2)); % �����͸׺Ͷ��۵Ľ½ӵ�D�붷�˺Ͷ��۽½ӵ�F�ľ���

    D_EFQ = params.D_EFQ; %����ǰ��μн� ����ֵ

    %% У�˶���������
    if (l_9 + L_2min)>l_8 && abs(l_9 - l_8)<L_2min && abs(l_8 - L_2min) < l_9 ...
            && (l_9 + l_8)>L_2max && abs(l_9 - L_2max)<l_8 && abs(l_8 - L_2max)<l_9
    %     disp('���˻��������ڸ������⡣')
    else
        over_all = 100;
        return
    %     disp('���˻������ڸ��棡')
    end

    %% ���˰ڽ�
    D_CFB = pi - alpha_1 - alpha_2; 
    D_DFB = params.D_DFB; % ��ѡֵ ����ֵ
    alpha_3 = D_DFB + D_CFB; % �����ϵĽṹ��CFD
    phi_2min = pi - alpha_3 - D_EFQ - acos((l_8^2 + l_9^2 - L_2max^2) / (2 * l_8 * l_9));
    phi_2max = pi - alpha_3 - D_EFQ - acos((l_8^2 + l_9^2 - L_2min^2) / (2 * l_8 * l_9));
    delta_phi_2 = phi_2max - phi_2min;

    %% �����������˻��������
    D_3 = 0.200;% �����͸׵ĸ׾� 
    d_3 = 0.085;%�����˸׾�
    L_3min = params.L_3min;%�����͸���С���� ����ֵ
    lambda_3=1.33; %�����͸�ѹ���� ����ֵ
    L_3max = L_3min * lambda_3;%�����͸���󳤶� ����ֵ
    delta_L3 = L_3max - L_3min;
    l_MN = l_kq;
    l_13 = l_MN;
    l_MK = l_kq;
    l_NQ = params.l_NQ_r * l_MN; % ����ֵ
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%                                   ��������װ�õ��˶�����                                     %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    cc = 0.45; % ���������½ӵ�C�ĺ����� ����ֵ
    bb = 0.9; % ���������½ӵ�C��ת̨����ϵ�ϵ������� ����ֵ
    aa = 1.25; % ת̨�߶� ����ֵ
    phi_0 = 0; % ת̨��z��ת���ĽǶ�
    % phi_3min = deg2rad(-135); % ����ת�����ֵ
    % phi_3max = deg2rad(45); % ����ת�����ֵ

    %% ��תƽ̨���˶�����
    R_z0phi0 = [cos(phi_0), -sin(phi_0), 0;...
                        sin(phi_0),  cos(phi_0), 0;...
                             0      ,        0,          1]; % ת̨�����z0�����ת����
    xyz_0A = [0; cc + l_5 * cos(alpha_11); bb - l_5 * sin(alpha_11)];
    xyz_0C = [0; cc; bb];
    XYZ_A = R_z0phi0 * xyz_0A + [0; 0; aa];
    XYZ_C = R_z0phi0 * xyz_0C + [0; 0; aa];

    %% ���۵��˶�����
    phi_1 = deg2rad(30); % ����˲ʱ�ڽ�
    R_x1phi1 = [1, 0, 0;...
                        0,  cos(phi_1), -sin(phi_1);...
                        0,   sin(phi_1), cos(phi_1)]; % �����ϸ��½ӵ���x1�����ת����
    xyz_1B = [0; l_7*cos(alpha_2); l_7*sin(alpha_2)];                
    XYZ_B = R_z0phi0 * (xyz_0C + R_x1phi1 * xyz_1B) + [0; 0; aa];
    xyz_1D = [0; l_1 - l_8*cos(alpha_3); l_8 * sin(alpha_3)];                
    XYZ_D = R_z0phi0 * (xyz_0C + R_x1phi1 * xyz_1D) + [0; 0; aa];
    xyz_1F = [0; l_1; 0];      
    XYZ_F = R_z0phi0 * (xyz_0C + R_x1phi1 * xyz_1F) + [0; 0; aa];

    %% ���˵��˶�����
    phi_2 = deg2rad(-60); % ����˲ʱ�ڽ�
    R_x2phi2 = [1, 0, 0;...
                        0,  cos(phi_2), -sin(phi_2);...
                        0,   sin(phi_2), cos(phi_2)]; % �����ϸ��½ӵ���x2�����ת����
    xyz_2E = [0; l_9 * cos(D_EFQ); l_9 * sin(D_EFQ)];
    XYZ_E = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * xyz_2E)) + [0; 0; aa];

    l_GF = params.l_GF; %����ֵ
    l_10 = l_GF;
    D_GFQ = params.D_GFQ; %����ֵ
    xyz_2G = [0; l_GF * cos(D_GFQ); l_GF * sin(D_GFQ)];
    XYZ_G = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * xyz_2G)) + [0; 0; aa];

    z_2N = 0.05; %����ֵ
    xyz_2N = [0; l_2 - sqrt(l_NQ^2 - z_2N^2); z_2N];
    XYZ_N = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * xyz_2N)) + [0; 0; aa];

    xyz_2Q = [0; l_2; 0];
    XYZ_Q = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * xyz_2Q)) + [0; 0; aa];

    %% �������������˻������˶�����
    l_GN = sqrt(sum((xyz_2G-xyz_2N).^2));
    l_11 = l_GN;
    theta_3min = acos((l_11^2 + l_13^2 - L_3min^2) / (2 * l_11 * l_13));
    theta_3max = acos((l_11^2 + l_13^2 - L_3max^2) / (2 * l_11 * l_13));
    % syms theta_3 % �����͸�˲ʱ����
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

    phi_3 = 0; %����˲ʱ�ڽ�

    R_x3phi3 = [1, 0, 0;...
                        0,  cos(phi_3), -sin(phi_3);...
                        0,   sin(phi_3), cos(phi_3)]; % �����ϸ��½ӵ���x3�����ת����
    xyz_3K = [0; l_kq * cos(alpha_10); l_kq * sin(alpha_10)];
    XYZ_K = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * (xyz_2Q + R_x3phi3 * xyz_3K))) + [0; 0; aa]; 

    xyz_3V = [0; l_3; 0];
    XYZ_V = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * (xyz_2Q + R_x3phi3 * xyz_3V))) + [0; 0; aa]; 

    %% ��������װ����ҵ����
    h_1 = aa + bb + l_1 * sin(phi_1min) - l_2 - l_3; %����ھ����
    h_2 = aa + bb + l_1 * sin(phi_1max) + l_2 * sin(phi_1max+phi_2max) + l_3 * sin(phi_1max+phi_2max+phi_3max); %����ھ�߶�
    l_CQmax = sqrt(l_1^2 + l_2^2 - 2 * l_1 * l_2 * cos(pi + phi_2max));
    r_1 = cc + l_CQmax + l_3; % ����ھ�뾶

    gamma = atan((aa + bb)/(sqrt((l_CQmax + l_3)^2 - (aa + bb)^2)));
    r_0 = cc + (l_CQmax + l_3) * cos(gamma); %ͣ����������ھ�뾶

    h_3 = aa + bb + l_1 * sin(phi_1max) + l_2 * sin(phi_1max + phi_2max) - l_3; %���ж�ظ߶�

    zeta = params.zeta; % ����ֵ
    phi_30 = -(pi/2 - zeta);

    if (phi_1min+phi_2max+phi_3max) < phi_30
        phi_1 = phi_30 - phi_2max - phi_3max;
        h_4 = aa + bb + l_1 * sin(phi_1) + l_2 * sin(phi_1 + phi_2max) - l_3 * cos(zeta);
        %���ֱ�ھ����
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
        over_all = 100; %�������over_all ������������Ч
    end
    
    savevideo = true;
    texttext = false;
    if params.draw %���drawΪtrue�򻭰���ͼ
        results.cc = cc;
        results.bb = bb;
        results.aa = aa;
        results.l_7 = l_7;
        results.alpha_2 = alpha_2;
        results.l_1 = l_1;
        results.l_2 = l_2;
        results.l_3 = l_3; % ������ͼ����Ĳ���

        %% ����ͼ����
        figure(1);
        m = moviein(10);
        frame = 0; %֡���
        for ii = 1: 8 %����ͼ��8��
            switch(ii)
                case 1 %V1~V2�ζ����͸���������͸���̡������͸׶̵���
                    for j = phi_3max : -0.0175 : phi_3min %����Ϊ0.0175�����ȣ���1��

                        frame = frame + 1;
                        clf;
                        [C1, B1, F1, Q1, V1] = convert(phi_0, phi_1max, phi_2max, j, results);
                        YZ = [C1, B1, F1, Q1, V1];
                        if frame == 1
                            Limit_position{1} = YZ;
                        end
                        Y = YZ(1,:);
                        Z = YZ(2,:);
                        nib_Y(frame) = V1(1); %����ÿһ֡�в�����V���λ��
                        nib_Z(frame) = V1(2); %����ÿһ֡�в�����V���λ��
                        Curve(1)= 1;
                        Curve(2)= frame;
                        DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
                        m(frame) = getframe;
                    end
                    Limit_position{end+1} = YZ;
                    
                case 2 %V2~V3�ζ����͸���������͸��ɶ̵����������͸��
                    for j = phi_2max : -0.0175 : phi_2min %����Ϊ0.0175�����ȣ���1��
                        frame = frame + 1;
                        clf;
                        [C1, B1, F1, Q1, V1] = convert(phi_0, phi_1max, j, phi_3min, results);
                        YZ = [C1, B1, F1, Q1, V1];
                        Y = YZ(1,:);
                        Z = YZ(2,:);
                        nib_Y(frame) = V1(1); %����ÿһ֡�в�����V���λ��
                        nib_Z(frame) = V1(2); %����ÿһ֡�в�����V���λ��
                        Curve(3)= frame;
                        DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
                        m(frame) = getframe;
                    end
                    Limit_position{end+1} = YZ;
                    
                case 3 %V3~V4�ζ����͸���������͸���������͸�����������ݼ�λ��C��Q2������
                    j = phi_3min;
                    while V1(2) > ((C1(2) - Q1(2)) / (C1(1) - Q1(1))) * V1(1) + (C1(2) - ((C1(2) - Q1(2)) / (C1(1) - Q1(1)))*C1(1)) && j < phi_3max
                        j = j + 0.0175; %��ʽ������˵��V�㻹��CQ����֮�ϣ����Լ���ѭ��
                        frame = frame + 1;
                        clf;
                        [C1, B1, F1, Q1, V1] = convert(phi_0, phi_1max, phi_2min, j, results);
                        YZ = [C1, B1, F1, Q1, V1];
                        Y = YZ(1,:);
                        Z = YZ(2,:);
                        nib_Y(frame) = V1(1); %����ÿһ֡�в�����V���λ��
                        nib_Z(frame) = V1(2); %����ÿһ֡�в�����V���λ��
                        Curve(4)= frame;
                        
                        DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
                        m(frame) = getframe;
                    end
                    Limit_position{end+1} = YZ;
                    Current_phi_3 = j;
                    
                case 4 %V4~V5�ζ����͸��ɳ����̡������͸���������͸׹̶�
                    for j = phi_1max : -0.0175 : phi_1min %����Ϊ0.0175�����ȣ���1��
                        frame = frame + 1;
                        clf;
                        [C1, B1, F1, Q1, V1] = convert(phi_0, j, phi_2min, Current_phi_3, results);
                        YZ = [C1, B1, F1, Q1, V1];
                        Y = YZ(1,:);
                        Z = YZ(2,:);
                        nib_Y(frame) = V1(1); %����ÿһ֡�в�����V���λ��
                        nib_Z(frame) = V1(2); %����ÿһ֡�в�����V���λ��
                        Curve(5)= frame;
                        DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
                        m(frame) = getframe;
                    end
                    Limit_position{end+1} = YZ;
                    
                case 5 %V5~V6�ζ����͸���̡������͸���������͸���ԭ�̶�ֵ����FQV����һ��
                    j = Current_phi_3;
                    while V1(2) > ((F1(2) - Q1(2)) / (F1(1) - Q1(1))) * V1(1) + (F1(2) - ((F1(2) - Q1(2)) / (F1(1) - Q1(1)))*F1(1)) && j<phi_3max
                        j = j + 0.0175; %��ʽ������˵��V�㻹��CQ����֮�ϣ����Լ���ѭ��
                        frame = frame + 1;
                        clf;
                        [C1, B1, F1, Q1, V1] = convert(phi_0, phi_1min, phi_2min, j, results);
                        YZ = [C1, B1, F1, Q1, V1];
                        Y = YZ(1,:);
                        Z = YZ(2,:);
                        nib_Y(frame) = V1(1); %����ÿһ֡�в�����V���λ��
                        nib_Z(frame) = V1(2); %����ÿһ֡�в�����V���λ��
                        Curve(6)= frame;
                        DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
                        m(frame) = getframe;
                    end
                    Current_phi_3 = j;
                    Limit_position{end+1} = YZ;
                    
                case 6 %V6~V7�ζ����͸���̡������͸��ɳ����̡������͸׹̶�
                    for j = phi_2min : 0.0175 : phi_2max %����Ϊ0.0175�����ȣ���1��
                        frame = frame + 1;
                        clf;
                        [C1, B1, F1, Q1, V1] = convert(phi_0, phi_1min, j, Current_phi_3, results);
                        YZ = [C1, B1, F1, Q1, V1];
                        Y = YZ(1,:);
                        Z = YZ(2,:);
                        nib_Y(frame) = V1(1); %����ÿһ֡�в�����V���λ��
                        nib_Z(frame) = V1(2); %����ÿһ֡�в�����V���λ��
                        Curve(7)= frame;
                        DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
                        m(frame) = getframe;
                    end
                    Limit_position{end+1} = YZ;
                    
                case 7 %V7~V8�ζ����͸���̡������͸���̡������͸���ԭ�̶�ֵ����CQV����һ��
                    j = Current_phi_3;
                    while V1(2) < ((C1(2) - Q1(2)) / (C1(1) - Q1(1))) * V1(1) + (C1(2) - ((C1(2) - Q1(2)) / (C1(1) - Q1(1)))*C1(1)) && j<phi_3max
                        j = j + 0.0175; %��ʽ������˵��V�㻹��CQ����֮�ϣ����Լ���ѭ��
                        frame = frame + 1;
                        clf;
                        [C1, B1, F1, Q1, V1] = convert(phi_0, phi_1min, phi_2max, j, results);
                        YZ = [C1, B1, F1, Q1, V1];
                        Y = YZ(1,:);
                        Z = YZ(2,:);
                        nib_Y(frame) = V1(1); %����ÿһ֡�в�����V���λ��
                        nib_Z(frame) = V1(2); %����ÿһ֡�в�����V���λ��
                        Curve(8)= frame;
                        DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
                        m(frame) = getframe;
                        
                    end
                    Current_phi_3 = j;
                    Limit_position{end+1} = YZ;
                    
                case 8 %V8~V9�ζ����͸��ɶ̵����������͸���̡������͸׹̶�
                    for j = phi_1min : 0.0175 : phi_1max %����Ϊ0.0175�����ȣ���1��
                        frame = frame + 1;
                        clf;
                        [C1, B1, F1, Q1, V1] = convert(phi_0, j, phi_2max, Current_phi_3, results);
                        YZ = [C1, B1, F1, Q1, V1];
                        Y = YZ(1,:);
                        Z = YZ(2,:);
                        nib_Y(frame) = V1(1); %����ÿһ֡�в�����V���λ��
                        nib_Z(frame) = V1(2); %����ÿһ֡�в�����V���λ��
                        Curve(9)= frame;
                        DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
                        m(frame) = getframe;
                        
                    end
                    Limit_position{end+1} = YZ;
                    
            end
        end
        movie(m,1,100);
        save m
        
        % �������յ�ͼ
        figure(2)
%         DrawPoints(Y, Z, nib_Y, nib_Z, Curve);
        DrawV(nib_Y, nib_Z, Curve)
        for vvv = 1 : length(Limit_position)
            DrawPoints(Limit_position{vvv}(1,:), Limit_position{vvv}(2,:), nib_Y, nib_Z, Curve);
        end
        set(gcf,'color','none');                     %����������û�б�����ɫ
        set(gca,'color','none');
        set(gcf,'InvertHardCopy','off');
        export_fig D:\Files\Master\�γ�\���̻�е��ƣ�����\����ͼ.png -transparent %%����
        
        if savevideo
            video = ['test.avi'];
            if exist(video, 'file') % �ļ����ڷ���2
                delete(video); % ��֤������Ƶ����ʱ��ʼʱ��Ϊ��
            end
            video_object = VideoWriter(video);
            video_object.FrameRate = 100;
            open(video_object);
            for frame = 1: size(m,2)
                frames = m(frame).cdata;
                writeVideo(video_object, frames); % д������
            end
            close(video_object); %�ر��ļ�
        end
    end
end
    



function [C, B, F, Q, V] = convert(phi_0, phi_1, phi_2, phi_3, results)
    % ������ֵ
    cc = results.cc;
    bb = results.bb;
    aa = results.aa;
    l_7 = results.l_7;
    alpha_2 = results.alpha_2;
    l_1 = results.l_1;
    l_2 = results.l_2;
    l_3 = results.l_3;

    %% ��תƽ̨���˶�����
    R_z0phi0 = [cos(phi_0), -sin(phi_0), 0;...
                       sin(phi_0),  cos(phi_0), 0;...
                            0      ,        0,          1]; % ת̨�����z0�����ת����
    xyz_0C = [0; cc; bb];
    XYZ_C = R_z0phi0 * xyz_0C + [0; 0; aa];
    C = XYZ_C(2:3);

    R_x1phi1 = [1, 0, 0;...
                    0,  cos(phi_1), -sin(phi_1);...
                    0,   sin(phi_1), cos(phi_1)]; % �����ϸ��½ӵ���x1�����ת����
    xyz_1B = [0; l_7*cos(alpha_2); l_7*sin(alpha_2)];                
    XYZ_B = R_z0phi0 * (xyz_0C + R_x1phi1 * xyz_1B) + [0; 0; aa];
    B = XYZ_B(2:3);
    
    xyz_1F = [0; l_1; 0];      
    XYZ_F = R_z0phi0 * (xyz_0C + R_x1phi1 * xyz_1F) + [0; 0; aa];
    F = XYZ_F(2:3);
    
    R_x2phi2 = [1, 0, 0;...
                    0,  cos(phi_2), -sin(phi_2);...
                    0,   sin(phi_2), cos(phi_2)]; % �����ϸ��½ӵ���x2�����ת����
    xyz_2Q = [0; l_2; 0];
    XYZ_Q = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * xyz_2Q)) + [0; 0; aa];
    Q = XYZ_Q(2:3);
    
    R_x3phi3 = [1, 0, 0;...
                    0,  cos(phi_3), -sin(phi_3);...
                    0,   sin(phi_3), cos(phi_3)]; % �����ϸ��½ӵ���x3�����ת����
    xyz_3V = [0; l_3; 0];
    XYZ_V = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * (xyz_2Q + R_x3phi3 * xyz_3V))) + [0; 0; aa]; 
    V = XYZ_V(2:3);
end

function DrawPoints(Y, Z, nib_Y, nib_Z, Curve)
    plot(Y, Z);
    hold on
    plot(nib_Y, nib_Z); % ������ͼ
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
    title('�ھ����ͼ');
    DrawV(nib_Y, nib_Z, Curve(1:end-1))
end

function DrawV(nib_Y, nib_Z, Curve)
    hold on
%     plot(nib_Y(Curve), nib_Z(Curve)); % ������ͼ
    plot(nib_Y(Curve), nib_Z(Curve),'x');
    for vv = 1:length(Curve)
%         text(nib_Y(Curve(vv))+0.25,nib_Z(Curve(vv))+0.2, ['V_', num2str(vv)]);
    end
end