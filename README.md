# Matlab Control Scripts

Este repositório contém scripts em MATLAB para o projeto e otimização de controladores PID fracionários, especificamente para sistemas com atrasos significativos. Os scripts incluem funcionalidades para calcular índices de desempenho como **IAE** (Integral of Absolute Error), **ISE** (Integral of Squared Error) e **ITAE** (Integral of Time-weighted Absolute Error), utilizando os algoritmos de **Particle Swarm Optimization (PSO)** e **Differential Evolution (DE)** para otimização.

---

## 📂 **Conteúdos**

### `iae_ise.m`
**Propósito:**  
Fornece funções para calcular **IAE** e **ISE** para controladores PID fracionários usando **PSO**.

#### **Funções Principais:**
```matlab
calcularIAE(param, Ga_completa) 
% Calcula o Integral of Absolute Error (IAE).

calcularISE(param, Ga_completa) 
% Calcula o Integral of Squared Error (ISE).

calcularITAE(param, Ga_completa) 
% Calcula o Integral of Time-weighted Absolute Error (ITAE).
```

---

### `iae_ise_atraso.m`
**Propósito:**  
Estende os cálculos básicos de **IAE** e **ISE** para sistemas com atrasos.

#### **Funções:**  
Similar às do script `iae_ise.m`, mas adaptadas para sistemas com atrasos.

---

### `iae_ise_de.m`
**Propósito:**  
Implementa a otimização por **Differential Evolution (DE)** para sintonizar parâmetros de controladores PID fracionários.

#### **Funções Principais:**
```matlab
differentialEvolution(fitnessFunc, numGenerations, populationSize, CR, F) 
% Realiza a otimização utilizando DE.

fitnessFunc(param, Ga_completa) 
% Define a função de custo para a otimização.
```

---

### `iae_ise_deatraso.m`
**Propósito:**  
Combina a otimização por **DE** com a compensação de atraso para a sintonia de controladores PID fracionários em sistemas com atrasos.

#### **Funções:**  
Integra as funcionalidades dos scripts `iae_ise_atraso.m` e `iae_ise_de.m`.

---

## 📖 **Instruções de Uso**
1. Certifique-se de ter o **MATLAB** instalado e configurado corretamente.
2. Clone este repositório para sua máquina local:
   ```bash
   git clone https://github.com/seu_usuario/matlab-control-scripts.git
   cd matlab-control-scripts
   ```
3. Execute os scripts no MATLAB para calcular os índices de desempenho ou otimizar os parâmetros do controlador.

---

## 🔧 **Dependências**
- **MATLAB** R2021a ou superior
- FOTF toolbox

---

## 👤 **Autor**
Guilherme Koiti Tanaka Sassaki  
[LinkedIn](https://www.linkedin.com/in/guilherme-sassaki-10b81ba7/)  
