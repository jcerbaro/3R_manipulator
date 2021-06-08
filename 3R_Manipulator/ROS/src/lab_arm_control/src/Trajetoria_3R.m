clear all;
close all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% CONFIGURAÇÃO - AQUI É ONDE PODE EDITAR VALORES %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %Parâmetros D-H do manipulador.
%     theta = [  pi; -pi/2; -pi/2];
    theta = [  deg2rad(-40); deg2rad(145); deg2rad(-133)];
    a     = [0.235;  0.245;  0.065];
    alpha = [   0;     0;     0];
    d     = [   0;     0;     0];

    %Centro da trajetória [x0 ;y0];
    centro = [0.36; 0.18];

    %Raio da trajetória [rx ;ry];
    raio = [0.10; 0.10];

    %Tempo total da trajetória, em segundos.
    t_max = 60;
    
    %Quantas voltas vai dar, dentro do tempo máximo.
    voltas = 1;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% GERAÇÃO DA TRAJETÓRIA - ESPAÇO OPERACIONAL %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    disp('Iniciou a geração de trajetória no espaço operacional...')
    
    syms t;
    loop_time = voltas/t_max;

    %Monta a trajetória de posição (modo simbólico) circular com base no que forneceu.
    %pd = [centro(1) + raio(1)*cos(2*pi*t*loop_time);
    %      centro(2) + raio(2)*sin(2*pi*t*loop_time)];
    
    %Trajetória circular conforme definido.
    %O sinal de negativo dentro do seno da paramétrica de y é para dar o sentido horário.
    %O +pi em x e y é para começar em 2pi rad.
    pd = [centro(1) + raio(1)*cos(    2*pi*t*loop_time   +pi);
          centro(2) + raio(2)*sin( -  2*pi*t*loop_time   +pi);
                                             sin(pi/10000*t)];
    
    interval = 0.001;

%     pd = [linspace( 0.45,0.45,t_max/interval+1);
%           linspace(-0.15,0.05,t_max/interval+1);
%           linspace( 0.00,0.00,t_max/interval+1)];

    %Monta a trajetória de velocidade (modo simbólico), derivando a trajetória de posição.
%     pd_dot = pd;
    pd_dot = diff(pd,t);

    %Monta o intervalo de valores do tempo.
    
    t = 0:interval:t_max;

    %Monta a trajetória de posição e velocidade (modo vetor numérico), com base nos valores de t gerados.
    desired = eval(pd);
    desired_dot = eval(pd_dot);
%     desired = pd;
%     desired_dot = pd_dot;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% %%%%%%%%% GERAÇÃO DA TRAJETÓRIA - ESPAÇO DAS JUNTAS %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    disp('Iniciou a geração de trajetória no espaço das juntas...')

    %Inicializa os valores de theta para o início da trajetória.
    erro = ones(3,1);
    while (norm(erro) > 0.001)
        
        [T1,T2,T3,FK] = FK3RDHMTH(alpha, a, theta, d);
        %Atribui as posições das juntas e do efetuador para plotar.
        J1  =  T1(1:2,4);
        J2  =  T2(1:2,4);
        J3  =  T3(1:2,4);
        eff =  FK(1:2,4);      
        
        %Calcula a orientação.
        yaw_ = atan2(FK(2,1), FK(1,1));
        
        %Calcula os erros de posição e orientação.
        erro_pos = norm(desired(1:2,1)-eff);
        erro_ori = desired(3,1) - yaw_;
        erro     = [desired(1:2,1)-eff;
                        erro_ori];

        %Jacobiano Geométrico.
        J = [(-a(1)*sin(theta(1))-a(2)*sin(theta(1)+theta(2))-a(3)*sin(theta(1)+theta(2)+theta(3))) ,  (-a(2)*sin(theta(1)+theta(2))-a(3)*sin(theta(1)+theta(2)+theta(3)))  ,  (-a(3)*sin(theta(1)+theta(2)+theta(3)));
             (a(1)*cos(theta(1))+a(2)*cos(theta(1)+theta(2))+a(3)*cos(theta(1)+theta(2)+theta(3)))  ,   (a(2)*cos(theta(1)+theta(2))+a(3)*cos(theta(1)+theta(2)+theta(3)))  ,   (a(3)*cos(theta(1)+theta(2)+theta(3)));
                                                                                                 1  ,                                                                    1  ,                                       1];

        K = diag([500;500;100]);
        q_dot = J'*K*erro;
        
        theta = [theta(1) + q_dot(1)*interval;
                 theta(2) + q_dot(2)*interval;
                 theta(3) + q_dot(3)*interval];
    end
    
    starter = rad2deg(theta);
    
    % Objetivo secundário... Maximizar a manipulabilidade.
    w = (1/2)*(sin(theta(2))^2 + (sin(theta(3))^2));
    w_dot = [0;
             cos(theta(2))*sin(theta(2));
             cos(theta(3))*sin(theta(3))];
    
    %Loop de geração da trajetória das juntas por cinemática inversa.
    erro = zeros(3,1);
    for k=1:length(t)
        
        [T1,T2,T3,FK] = FK3RDHMTH(alpha, a, theta, d);
        %Atribui as posições das juntas e do efetuador para plotar.
        J1(:,k)  =  T1(1:2,4);
        J2(:,k)  =  T2(1:2,4);
        J3(:,k)  =  T3(1:2,4);
        eff(:,k) =  FK(1:2,4);      
        
        %Calcula a orientação.
        yaw(k) = atan2(FK(2,1), FK(1,1));
        
        %Calcula os erros de posição e orientação.
        erro_pos(k) = norm(desired(1:2,k)-eff(:,k));
        erro_ori(k) = desired(3,k) - yaw(k);
        erro        = [desired(1:2,k)-eff(:,k);
                             erro_ori(k)];

        %Jacobiano Geométrico.
        J = [(-a(1)*sin(theta(1))-a(2)*sin(theta(1)+theta(2))-a(3)*sin(theta(1)+theta(2)+theta(3)))  ,  (-a(2)*sin(theta(1)+theta(2))-a(3)*sin(theta(1)+theta(2)+theta(3)))  ,  (-a(3)*sin(theta(1)+theta(2)+theta(3)));
              (a(1)*cos(theta(1))+a(2)*cos(theta(1)+theta(2))+a(3)*cos(theta(1)+theta(2)+theta(3)))  ,   (a(2)*cos(theta(1)+theta(2))+a(3)*cos(theta(1)+theta(2)+theta(3)))  ,   (a(3)*cos(theta(1)+theta(2)+theta(3)));
                                                                                                  1  ,                                                                    1  ,                                       1];

        K = diag([500;500;100]);

        k_zero = 100;

        [~,w_dot] = manipulability(theta);

        q_zero(:,k) = k_zero*w_dot;

        pseudo = J'*(inv(J*J'));

        q_dot = pseudo*(desired_dot(:,k) + K*erro) + (eye(3) - pseudo*J)*q_zero(:,k);
        
        q_plot(:,k) = theta;
        q_dot_plot(:,k) = double(q_dot);

        theta = [theta(1) + q_dot(1)*interval;
                 theta(2) + q_dot(2)*interval;
                 theta(3) + q_dot(3)*interval];
             
        percent = k/length(t);
%         clc;
%         disp('geração de trajetória no espaço das juntas...')
%         fprintf('%.2f completo.\n', percent)
    end
    
    %Converte todo o vetor de ângulos da trajetória para graus.
    joints = rad2deg(q_plot);
    
    disp('Trajetória gerada com sucesso!')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOT DA TRAJETÓRIA PLANEJADA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PLOT = 0;

if(PLOT==1)
    for k=1:length(q_plot)
        [manip(k),~] = manipulability(q_plot(:,k));
    end
    figure;
    hold on;
    
    %%%% Plot %%%%%

    subplot(2,2,1);
    plot(q_plot(1,:),'r');hold on
    plot(q_plot(2,:),'g');hold on
    plot(q_plot(3,:),'b');hold on
    axis ([0 length(t) -5 5]);
    title('Ângulo das juntas (rad)')

    subplot(2,2,2);
    plot(q_dot_plot(1,50:length(t)),'r');hold on
    plot(q_dot_plot(2,50:length(t)),'g');hold on
    plot(q_dot_plot(3,50:length(t)),'b');hold on
    axis ([0 length(t) -1 1]);
    title('Velocidade das juntas (rad/s)')

    subplot(2,2,3);
    plot(erro_pos(50:length(t)),'k')
    title('Erro de Posição')

    subplot(2,2,4);
    plot(manip,'k');
%     axis ([0 t_max*1000 0.6 1]);
    title('Manipulabilidade');

    hold off

    for k = 1:100:length(eff')
        tic

        x = [J1(1,k), J2(1,k), J3(1,k), eff(1,k)];
        y = [J1(2,k), J2(2,k), J3(2,k), eff(2,k)];

        start_quiver = [eff(1,k) eff(2,k)];

        end_quiver         = [eff(1,k)+ 0.10*cos(yaw(k)) eff(2,k)+ 0.10*sin(yaw(k))];
        end_quiver_desired = [eff(1,k)+ 0.15*cos(desired(3,k)) eff(2,k)+ 0.15*sin(desired(3,k))];

        len_quiver = end_quiver-start_quiver;
        len_quiver_desired = end_quiver_desired-start_quiver;

        figure(2)
        plot(x,y,'-o','Linewidth',3,'Color','k');hold on
        plot(desired(1,:),desired(2,:),'Color','r');hold on
        seta = quiver(start_quiver(1),start_quiver(2),len_quiver(1),len_quiver(2),'Color','k');
        seta.MaxHeadSize = 10;
        seta2 = quiver(start_quiver(1),start_quiver(2),len_quiver_desired(1),len_quiver_desired(2),'Color','r');
        seta2.MaxHeadSize = 10;
        hold off;

        axis ([-0.2 0.5 -0.2 0.5]);

        while(toc < interval)
            %do nothing.
        end
    end
end



    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONFIGURAÇÃO DO ROS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    
ROS = 1;

if(ROS == 1)
    %Cria a mensagem e inicia o publicador.
    rosshutdown;
    rosinit('http://jonathan-notebook:11311/');
    
    msg = rosmessage('custom_msg/set_angles');
    pub = rospublisher('/cmd_3R');
    
    IsDoneOMB = rosmessage('custom_msg/status_arm');
    IsDoneCOT = rosmessage('custom_msg/status_arm');
    IsDonePUN = rosmessage('custom_msg/status_arm');
    subOMB = rossubscriber('/status_OMB');
    subCOT = rossubscriber('/status_COT');
    subPUN = rossubscriber('/status_PUN');
    
    %Manda o primeiro ponto da trajetória, para iniciar com erro zero.
    msg.SetOMB = joints(1,1);
    msg.SetCOT = joints(2,1);
    msg.SetPUN = joints(3,1);
    send(pub,msg);
    disp('Mandou o starter!')
    
    pause(1);
    
    %Lê o status das juntas até todas estarem OK.
    ready = false;
    tic
    while(toc < 5)
%         
%         IsDoneOMB = receive(subOMB,10);
%         IsDoneCOT = receive(subCOT,10);
%         IsDonePUN = receive(subPUN,10);
%         
%         clc
%         disp('Os três precisam estar com valor 1:')
%         fprintf('Ombro: %i\n', IsDoneOMB.IsDone)
%         fprintf('Cotovelo: %i\n', IsDoneCOT.IsDone)
%         fprintf('Punho: %i\n', IsDonePUN.IsDone)
%         
%         if(IsDoneOMB.IsDone && IsDoneCOT.IsDone && IsDonePUN.IsDone)
%             ready = true;
%         end
    end
    
    disp('Começou a trajetória!')
    
    %Inicia a trajetória mandando a mensagem cada vez que o timeout dela chegar.
    spacing = 10;
    j=1;
    for k=spacing:spacing:length(joints)
        tic
        
        msg.SetOMB = joints(1,k);
        msg.SetCOT = joints(2,k);
        msg.SetPUN = joints(3,k);
        
        joint_sent(1,j) = msg.SetOMB;
        joint_sent(2,j) = msg.SetCOT;
        joint_sent(3,j) = msg.SetPUN;
        
        while(toc < interval*spacing)
            %do nothing.
        end
        send(pub,msg);
        
        j=j+1;
    end
    
    disp('Terminou a trajetória!')
end

% figure(4)
% plot(joint_sent(1,:)); hold on;
% plot(joint_sent(2,:)); hold on;
% plot(joint_sent(3,:)); hold on;
%     
    
    
    
    
    
    
    
    
    
    