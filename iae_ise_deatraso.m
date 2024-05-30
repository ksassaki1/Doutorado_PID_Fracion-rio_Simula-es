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

% Valores Ziegler-Nichols
zn_values = [2, 2, 2, 1, 1];

% Configurações da Evolução Diferencial
nIndividuos = 50;
nIteracoes = 50;
dimensoes = 5; % kp, ki, kd, lambda, mu
limites = [0, 200; 0, 200; 0, 2000; 0, 2; 0, 2];
variancia = [10, 0.1, 100, 0.1, 0.1]; % Variação relativa para inicialização


% Otimização usando IAE como função de fitness
[gBestIAE, ~] = optimizeFitness(@calcularIAE, zn_values, nIndividuos, nIteracoes, dimensoes, variancia, limites, Ga_completa);
[gBestISE, ~] = optimizeFitness(@calcularISE, zn_values, nIndividuos, nIteracoes, dimensoes, variancia, limites, Ga_completa);


% Calcular IAE e ISE para os melhores parâmetros encontrados após otimização para IAE
IAE_IAE = calcularIAE(gBestIAE, Ga_completa);
ISE_IAE = calcularISE(gBestIAE, Ga_completa);

% Calcular IAE e ISE para os melhores parâmetros encontrados após otimização para ISE
IAE_ISE = calcularIAE(gBestISE, Ga_completa);
ISE_ISE = calcularISE(gBestISE, Ga_completa);

% Resposta ao degrau para os melhores parâmetros com IAE
kp = gBestIAE(1);
ki = gBestIAE(2);
kd = gBestIAE(3);
lambda = gBestIAE(4);
mu = gBestIAE(5);
C_IAE = kp + ki*s^(-lambda) + kd*s^(mu);
T_IAE = feedback(C_IAE*Ga_completa, 1);
respostaIAE = step(T_IAE, linspace(0, 1000, 1000));

% Resposta ao degrau para os melhores parâmetros com ISE
kp = gBestISE(1);
ki = gBestISE(2);
kd = gBestISE(3);
lambda = gBestISE(4);
mu = gBestISE(5);
C_ISE = kp + ki*s^(-lambda) + kd*s^(mu);
T_ISE = feedback(C_ISE*Ga_completa, 1);
respostaISE = step(T_ISE, linspace(0, 1000, 1000));

% Exibir os resultados para a sintonia baseada em ISE
disp('Resultados da sintonia baseada em ISE:');
disp(['Kp: ', num2str(gBestISE(1))]);
disp(['Ki: ', num2str(gBestISE(2))]);
disp(['Kd: ', num2str(gBestISE(3))]);
disp(['Lambda: ', num2str(gBestISE(4))]);
disp(['Mu: ', num2str(gBestISE(5))]);
disp(['IAE: ', num2str(IAE_ISE)]);
disp(['ISE: ', num2str(ISE_ISE)]);

% Exibir os resultados para a sintonia baseada em IAE
disp('Resultados da sintonia baseada em IAE:');
disp(['Kp: ', num2str(gBestIAE(1))]);
disp(['Ki: ', num2str(gBestIAE(2))]);
disp(['Kd: ', num2str(gBestIAE(3))]);
disp(['Lambda: ', num2str(gBestIAE(4))]);
disp(['Mu: ', num2str(gBestIAE(5))]);
disp(['IAE: ', num2str(IAE_IAE)]);
disp(['ISE: ', num2str(ISE_IAE)]);


% Definindo os parâmetros do controlador PID baseado em Ziegler-Nichols
Kp_ZN = 107.3;
Ki_ZN = 2.7;
Kd_ZN = 1073;

% Criando o controlador PID como uma 'fotf'
C_ZN = Kp_ZN + Ki_ZN/s + Kd_ZN*s;

% Combinando o controlador com a planta
T_ZN = feedback(C_ZN*Ga_completa, 1);

% Defina o vetor de tempo para a simulação da resposta ao degrau
tFinal = 50; % Tempo final para a simulação
t = linspace(0, tFinal, 1000); % Vetor de tempo

% Calculando a resposta ao degrau do sistema em malha fechada com o controlador ZN
respostaDegrau_ZN = step(T_ZN, t);


% Calcular IAE para Ziegler-Nichols
IAE_ZN = calcularIAE([Kp_ZN, Ki_ZN, Kd_ZN, 1, 1], Ga_completa);

% Calcular ISE para Ziegler-Nichols
ISE_ZN = calcularISE([Kp_ZN, Ki_ZN, Kd_ZN, 1, 1], Ga_completa);


% Exibir os resultados para a sintonia baseada em Ziegler-Nichols
disp('Resultados da sintonia baseada em Ziegler-Nichols:');
disp(['Kp: ', num2str(Kp_ZN)]);
disp(['Ki: ', num2str(Ki_ZN)]);
disp(['Kd: ', num2str(Kd_ZN)]);
disp(['IAE: ', num2str(IAE_ZN)]);
disp(['ISE: ', num2str(ISE_ZN)]);

% Plota a resposta ao degrau
figure;
hold on;
%plot(t, respostaDegrau_ZN, 'm-', 'LineWidth', 2); % Resposta ZN em magenta

% Otimize e plote as respostas para ISE e IAE (aqui você incluiria o código de otimização ou as chamadas para essas funções)
% Supondo que 'respostaISE' e 'respostaIAE' sejam variáveis já definidas no seu código
plot(t, respostaIAE, 'b-', 'LineWidth', 2);
plot(t, respostaISE, 'r--', 'LineWidth', 2);

title('Comparação das Respostas ao Degrau');
xlabel('Tempo (s)');
ylabel('Resposta');
legend('Resposta com IAE', 'Resposta com ISE', 'Location', 'Best');
grid on;
hold off;

% Função de otimização com fitness customizado
function [gBest, gBestScore] = optimizeFitness(fitnessFunc, zn_values, nIndividuos, nIteracoes, dimensoes, variancia, limites, G)
    % Inicializar população perto dos valores de Ziegler-Nichols
    populacao = repmat(zn_values, nIndividuos, 1) + variancia .* randn(nIndividuos, dimensoes);
    populacao = max(populacao, limites(:,1)'); % Garantir que não ultrapasse os limites inferiores
    populacao = min(populacao, limites(:,2)'); % Garantir que não ultrapasse os limites superiores

    % Avaliação inicial
    fitness = arrayfun(@(i) fitnessFunc(populacao(i,:), G), 1:nIndividuos);

    % Encontrar o melhor inicial
    [gBestScore, idx] = min(fitness);
    gBest = populacao(idx,:);

    % Parâmetros da DE
    F = 0.8; % Fator de mutação
    CR = 0.9; % Taxa de crossover

    % Loop de otimização
    for iter = 1:nIteracoes
        for i = 1:nIndividuos
            % Mutação
            idxs = [1:i-1, i+1:nIndividuos];
            r = idxs(randperm(length(idxs), 3));
            x1 = populacao(r(1), :);
            x2 = populacao(r(2), :);
            x3 = populacao(r(3), :);
            mutante = x1 + F * (x2 - x3);
            mutante = min(max(mutante, limites(:,1)'), limites(:,2)'); % Garante limites

            % Crossover
            trial = populacao(i,:);
            crossPoints = rand(1, dimensoes) < CR;
            trial(crossPoints) = mutante(crossPoints);

            % Seleção
            trialFitness = fitnessFunc(trial, G);
            if trialFitness < fitness(i)
                populacao(i,:) = trial;
                fitness(i) = trialFitness;
                if trialFitness < gBestScore
                    gBestScore = trialFitness;
                    gBest = trial;
                end
            end
        end
    end
end

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
    t = linspace(0, tFinal, 1000); % Gere um vetor de tempo

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
    nPoints = 1000; % Definindo o número de pontos
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