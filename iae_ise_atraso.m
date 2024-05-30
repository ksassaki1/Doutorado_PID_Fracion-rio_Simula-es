clear
clc
% Defina o sistema a ser controlado
s = fotf('s');
% Definição da parte sem atraso da função de transferência
num = 1.378;  % Numerador
den = [77.93 9.48 1];  % Denominador (329s + 1)
Ga_s = tf(num, den);  % Função de Transferência sem atraso

% Definição do atraso usando aproximação de Padé
atraso_s = 3.84;  % Atraso de 20 segundos
[nump, denp] = pade(atraso_s, 1);  % Aproximação de Padé de 1ª ordem
atraso_tf = tf(nump, denp);  % Função de Transferência do atraso

% Combinação da parte sem atraso e do atraso
Ga_completa = Ga_s * atraso_tf;  % Função de Transferência completa com atraso

% Parâmetros do PSO
nParticulas = 50;
nIteracoes = 50;
dimensoes = 5; % kp, ki, kd, lambda, mu
limites = [0, 200 ; 0, 200 ; 0, 2000; 0, 2; 0, 2];
valoresIniciais = [2, 2, 2, 1, 1];
varianciaInicial = 0.1;

% Otimização usando ISE
[gBest_ISE, gBestScore_ISE] = otimizarPSO(@calcularISE, Ga_completa, nParticulas, nIteracoes, dimensoes, limites, valoresIniciais, varianciaInicial);

% Otimização usando IAE
[gBest_IAE, gBestScore_IAE] = otimizarPSO(@calcularIAE, Ga_completa, nParticulas, nIteracoes, dimensoes, limites, valoresIniciais, varianciaInicial);


% Definir o controlador com os melhores parâmetros para ISE
C_ISE = gBest_ISE(1) + gBest_ISE(2)*s^(-gBest_ISE(4)) + gBest_ISE(3)*s^(gBest_ISE(5));
% Definir o controlador com os melhores parâmetros para IAE
C_IAE = gBest_IAE(1) + gBest_IAE(2)*s^(-gBest_IAE(4)) + gBest_IAE(3)*s^(gBest_IAE(5));

% Sistema em malha fechada com o controlador otimizado por ISE
T_ISE = feedback(C_ISE*Ga_completa, 1);
% Sistema em malha fechada com o controlador otimizado por IAE
T_IAE = feedback(C_IAE*Ga_completa, 1);

% Definir o intervalo de tempo para a simulação da resposta ao degrau
tFinal = 50; % Tempo final para a simulação
t = linspace(0, tFinal, 2000); % Vetor de tempo

% Calcular a resposta ao degrau
respostaDegrau_ISE = step(T_ISE, t);
respostaDegrau_IAE = step(T_IAE, t);

figure; % Cria uma nova janela para o gráfico
hold on; % Mantém a figura ativa para múltiplos plots
plot(t, respostaDegrau_ISE, 'r-', 'LineWidth', 2); % Resposta ISE em vermelho
plot(t, respostaDegrau_IAE, 'b--', 'LineWidth', 2); % Resposta IAE em azul pontilhado

% Definindo os parâmetros do controlador PID baseado em Ziegler-Nichols
Kp_ZN = 107.3;
Ki_ZN = 2.7;
Kd_ZN = 1073;

% Criando o controlador PID como uma 'fotf'
C_ZN = Kp_ZN + Ki_ZN/s + Kd_ZN*s;

% Combinando o controlador com a planta
T_ZN = feedback(C_ZN*Ga_completa, 1);

% Calculando a resposta ao degrau do sistema em malha fechada com o controlador ZN
respostaDegrau_ZN = step(T_ZN, t);



% Adicionando a curva da resposta ao degrau com o controlador Ziegler-Nichols no gráfico
%plot(t, respostaDegrau_ZN, 'm-', 'LineWidth', 2); % Resposta ZN em magenta

% Atualizando a legenda para incluir todas as curvas
legend('Resposta com ISE', 'Resposta com IAE', 'Location', 'Best');

title('Comparação das Respostas ao Degrau'); % Título do gráfico
xlabel('Tempo (s)'); % Rótulo do eixo X
ylabel('Resposta'); % Rótulo do eixo Y
grid on; % Adiciona grade ao gráfico



% Atualizar a legenda para incluir a curva de Valerio
legend('Resposta com ISE', 'Resposta com IAE', 'Resposta com Ziegler-Nichols', 'Location', 'Best');

% Segurar o gráfico para adições futuras
hold off;

% Calcule IAE, ISE e ITAE para a sintonia baseada em ISE
IAE_ISE = calcularIAE(gBest_ISE, Ga_completa);
ISE_ISE = calcularISE(gBest_ISE, Ga_completa);
ITAE_ISE = calcularITAE(gBest_ISE, Ga_completa);

% Calcule IAE, ISE e ITAE para a sintonia baseada em IAE
IAE_IAE = calcularIAE(gBest_IAE, Ga_completa);
ISE_IAE = calcularISE(gBest_IAE, Ga_completa);
ITAE_IAE = calcularITAE(gBest_IAE, Ga_completa);

% Exiba os resultados para a sintonia baseada em ISE
disp('Resultados da sintonia baseada em ISE:');
disp(['Kp: ', num2str(gBest_ISE(1))]);
disp(['Ki: ', num2str(gBest_ISE(2))]);
disp(['Kd: ', num2str(gBest_ISE(3))]);
disp(['Lambda: ', num2str(gBest_ISE(4))]);
disp(['Mu: ', num2str(gBest_ISE(5))]);
disp(['IAE: ', num2str(IAE_ISE)]);
disp(['ISE: ', num2str(ISE_ISE)]);
disp(['ITAE: ', num2str(ITAE_ISE)]);

% Exiba os resultados para a sintonia baseada em IAE
disp('Resultados da sintonia baseada em IAE:');
disp(['Kp: ', num2str(gBest_IAE(1))]);
disp(['Ki: ', num2str(gBest_IAE(2))]);
disp(['Kd: ', num2str(gBest_IAE(3))]);
disp(['Lambda: ', num2str(gBest_IAE(4))]);
disp(['Mu: ', num2str(gBest_IAE(5))]);
disp(['IAE: ', num2str(IAE_IAE)]);
disp(['ISE: ', num2str(ISE_IAE)]);
disp(['ITAE: ', num2str(ITAE_IAE)]);

% Calcular IAE para Ziegler-Nichols
IAE_ZN = calcularIAE([Kp_ZN, Ki_ZN, Kd_ZN, 1, 1], Ga_completa);

% Calcular ISE para Ziegler-Nichols
ISE_ZN = calcularISE([Kp_ZN, Ki_ZN, Kd_ZN, 1, 1], Ga_completa);

% Calcular ITAE para Ziegler-Nichols
ITAE_ZN = calcularITAE([Kp_ZN, Ki_ZN, Kd_ZN, 1, 1], Ga_completa);

% Exibir os resultados para a sintonia baseada em Ziegler-Nichols
disp('Resultados da sintonia baseada em Ziegler-Nichols:');
disp(['Kp: ', num2str(Kp_ZN)]);
disp(['Ki: ', num2str(Ki_ZN)]);
disp(['Kd: ', num2str(Kd_ZN)]);
disp(['IAE: ', num2str(IAE_ZN)]);
disp(['ISE: ', num2str(ISE_ZN)]);
disp(['ITAE: ', num2str(ITAE_ZN)]);

% Função para calcular IAE
function IAE = calcularIAE(param, Ga_completa)
    kp = param(1);
    ki = param(2);
    kd = param(3);
    lambda = param(4);
    mu = param(5);

    % Defina s como um objeto fotf
    s = fotf('s');

    % Crie o controlador PI^lambda D^mu
    C = kp + ki*s^(-lambda) + kd*s^(mu);

    % Sistema em malha fechada
    T = feedback(C*Ga_completa, 1);

    % Defina um intervalo de tempo para a simulação
    tFinal = 50; % Tempo final para a simulação
    t = linspace(0, tFinal, 2000); % Gere um vetor de tempo

    % Calcule o IAE
    resposta = step(T, t);
    IAE = trapz(t, abs(resposta - 1)); % Supondo uma resposta desejada de 1 (degrau unitário)
end

function ISE = calcularISE(param, Ga_completa)
    kp = param(1);
    ki = param(2);
    kd = param(3);
    lambda = param(4);
    mu = param(5);
    s = fotf('s');
    C = kp + ki*s^(-lambda) + kd*s^(mu);
    T = feedback(C*Ga_completa, 1);
    tFinal = 50; % Definindo o tempo final
    nPoints = 2000; % Definindo o número de pontos
    t = linspace(0, tFinal, nPoints); % Gerando os pontos de tempo
    resposta = step(T, t);
    erro = 1 - resposta; % Calculando o erro

    % Implementação manual do método dos trapézios para ISE
    dt = t(2) - t(1); % Calculando o delta t
    integral = 0;
    for i = 1:(nPoints - 1)
        trapezio = ((erro(i))^2 + (erro(i+1))^2) / 2;
        integral = integral + trapezio * dt;
    end
    ISE = integral;
end

function ITAE = calcularITAE(param, Ga_completa)
    kp = param(1);
    ki = param(2);
    kd = param(3);
    lambda = param(4);
    mu = param(5);
    s = fotf('s');
    C = kp + ki*s^(-lambda) + kd*s^(mu);
    T = feedback(C*Ga_completa, 1);
    tFinal = 50; % Definindo o tempo final
    nPoints = 2000; % Definindo o número de pontos
    t = linspace(0, tFinal, nPoints); % Gerando os pontos de tempo
    resposta = step(T, t);
    erro = 1 - resposta; % Calculando o erro

    % Implementação manual do método dos trapézios
    dt = t(2) - t(1); % Calculando o delta t
    integral = 0;
    for i = 1:(nPoints - 1)
        trapezio = (t(i) * abs(erro(i)) + t(i+1) * abs(erro(i+1))) / 2;
        integral = integral + trapezio * dt;
    end
    ITAE = integral;
end

function [gBest, gBestScore] = otimizarPSO(funcaoObjetivo, G, nParticulas, nIteracoes, dimensoes, limites, valoresIniciais, varianciaInicial)
    % Inicialização do PSO com valores iniciais
    posicao = repmat(valoresIniciais, nParticulas, 1) + varianciaInicial * randn(nParticulas, dimensoes);
    posicao = max(posicao, limites(:,1)');
    posicao = min(posicao, limites(:,2)');

    % Inicialização da velocidade
    velocidade = zeros(size(posicao));

    % Inicialização do pBest
    pBest = posicao;
    pBestScore = arrayfun(@(i) funcaoObjetivo(posicao(i,:), G), 1:nParticulas);

    % Encontrar o melhor score global e a posição correspondente (gBest)
    [gBestScore, idx] = min(pBestScore);
    gBest = posicao(idx,:);

    % Parâmetros do PSO
    w = 0.5; % Inércia
    c1 = 2.5; % Coeficiente cognitivo
    c2 = 1.5; % Coeficiente social

    % Loop principal do PSO
    for iter = 1:nIteracoes
        for i = 1:nParticulas
            % Atualizações de velocidade e posição
            velocidade(i,:) = w * velocidade(i,:) ...
                            + c1 * rand * (pBest(i,:) - posicao(i,:)) ...
                            + c2 * rand * (gBest - posicao(i,:));
            posicao(i,:) = posicao(i,:) + velocidade(i,:);
            posicao(i,:) = max(posicao(i,:), limites(:,1)');
            posicao(i,:) = min(posicao(i,:), limites(:,2)');

            % Avaliação
            scoreAtual = funcaoObjetivo(posicao(i,:), G);
            if scoreAtual < pBestScore(i)
                pBestScore(i) = scoreAtual;
                pBest(i,:) = posicao(i,:);
            end
            if scoreAtual < gBestScore
                gBestScore = scoreAtual;
                gBest = posicao(i,:);
            end
        end
    end
end