#include "../include/algExato.hpp"
#include "../include/leitorArquivos.hpp"

#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>

// Construtor 
SolExata::SolExata(const Instancia& instancia, double fatorVelocidadeDrone, int qntDrones) : 
                    instancia_(instancia), fatorVelocidadeDrone_(fatorVelocidadeDrone), dronesPorEstacao_(qntDrones) {}

// Calcula a distancia entre dois nós
double SolExata::calcularDistancia(Coordenada c1, Coordenada c2) const {
    double dx = c1.x - c2.x;
    double dy = c1.y - c2.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Calcula a menor contribuição possível de cada cliente
void SolExata::precalcularCustosMinimos() {
    int totalClientes = (int)instancia_.clientes.size();
    int totalEstacoes = (int)instancia_.estacoes.size();
    custoMinimoCliente_.assign(totalClientes, 0.0);

    for (int i = 0; i < totalClientes; ++i) {
        Coordenada posCli = instancia_.clientes[i].posicao;
        double menorCusto = std::numeric_limits<double>::max();

        // Calcula a distância de um cliente até outro ponto (cliente, depósito ou estação)
        double distDeposito = calcularDistancia(posCli, instancia_.deposito.posicao);
        if (distDeposito < menorCusto) menorCusto = distDeposito;

        for (int j = 0; j < totalClientes; ++j) {
            if (i == j) continue;
            double d = calcularDistancia(posCli, instancia_.clientes[j].posicao);
            if (d < menorCusto) menorCusto = d;
        }

        for (int e = 0; e < totalEstacoes; ++e) {
            double d = calcularDistancia(posCli, instancia_.estacoes[e].posicao);
            if (d < menorCusto) menorCusto = d;
        }

        // Calcula o custo para atender cliente por drone (se estiver dentro do raio)
        for (int e = 0; e < totalEstacoes; ++e) {
            double distEstacao = calcularDistancia(posCli, instancia_.estacoes[e].posicao);
            if (distEstacao <= (double)instancia_.raio && dronesPorEstacao_ > 0) {
                // Tempo de ida e volta do drone
                double tempoDrone = (2.0 * distEstacao) / (1.0 * fatorVelocidadeDrone_);
                if (tempoDrone < menorCusto) {
                    menorCusto = tempoDrone;
                }
            }
        }

        custoMinimoCliente_[i] = menorCusto;
    }
}

// Calcula o Lower Bound
double SolExata::calcularLowerBound(const EstadoBusca& estado) const {
    int totalClientes = (int)instancia_.clientes.size();
    int qntCaminhoes = instancia_.deposito.qntCaminhoes;

    double somaTrabalhoRestante = 0.0;
    double maiorCustoIsolado = 0.0;

    for (int i = 0; i < totalClientes; ++i) {
        if (!estado.visitados[i]) {
            double c = custoMinimoCliente_[i];
            somaTrabalhoRestante += c;
            if (c > maiorCustoIsolado) {
                maiorCustoIsolado = c;
            }
        }
    }

    // Tempo já gasto somado ao custo restante dividido entre os caminhões disponíveis
    double somaTemposAtuais = 0.0;
    for (const auto& rota : estado.solucaoParcial.rotasCaminhoes) {
        somaTemposAtuais += rota.tempoViagem;
    }

    double lbMediaFrota = 0.0;
    if (qntCaminhoes > 0) {
        lbMediaFrota = (somaTemposAtuais + somaTrabalhoRestante) / (double)qntCaminhoes;
    }

    // Identifica o cliente mais caro
    double lbGargalo = std::max(estado.makespanAtual, maiorCustoIsolado);

    // Pegamos o máximo entre as duas estimativas 
    return std::max(lbGargalo, lbMediaFrota);
}

// Calcula o custo total de uma rota de caminhao
double SolExata::calcularCustoRotaCaminhao(const RotaCaminhao& rota) const {
    if(rota.sequenciaNos.empty() && rota.estacoesVisitadas.empty()) return 0.0;

    double custoTotal = 0.0;
    Coordenada posicaoAtual = instancia_.deposito.posicao;

    int totalEstacoesInstancia = (int)instancia_.estacoes.size();
    for (int idEstacao : rota.estacoesVisitadas) {
        if (idEstacao >= 0 && idEstacao < totalEstacoesInstancia) {
            Coordenada posEstacao = instancia_.estacoes[idEstacao].posicao;
            custoTotal += calcularDistancia(posicaoAtual, posEstacao);
            posicaoAtual = posEstacao;
        }
    }

    for (int idCliente : rota.sequenciaNos) {
        Coordenada posCliente = {0.0, 0.0};
        bool encontrado = false;
        for (const auto& cli : instancia_.clientes) {
            if (cli.idCliente == idCliente) {
                posCliente = cli.posicao;
                encontrado = true;
                break;
            }
        }
        if (encontrado) {
            custoTotal += calcularDistancia(posicaoAtual, posCliente);
            posicaoAtual = posCliente;
        }
    }

    custoTotal += calcularDistancia(posicaoAtual, instancia_.deposito.posicao);

    return custoTotal;
}

// Processamento de opções de Caminhão 
void SolExata::processarOpcoesCaminhao(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan) {
    int clienteAtual = estado.clienteAtual;
    int totalRotas = (int)estado.solucaoParcial.rotasCaminhoes.size();
    int totalEstacoes = (int)instancia_.estacoes.size();

    // Opção 1: O caminhão atende o cliente diretamente na rota existente
    for (int i = 0; i < totalRotas; ++i) {
        std::vector<bool> visitadosCaminhao = estado.visitados;
        visitadosCaminhao[clienteAtual] = true;

        estado.solucaoParcial.rotasCaminhoes[i].sequenciaNos.push_back(clienteAtual);
        double novoCustoRota = calcularCustoRotaCaminhao(estado.solucaoParcial.rotasCaminhoes[i]);
        double tempoAntigo = estado.solucaoParcial.rotasCaminhoes[i].tempoViagem;
        estado.solucaoParcial.rotasCaminhoes[i].tempoViagem = novoCustoRota;

        double makespanGlobalTemp = 0.0;
        for(const auto& rota : estado.solucaoParcial.rotasCaminhoes) {
            if (rota.tempoViagem > makespanGlobalTemp) makespanGlobalTemp = rota.tempoViagem;
        }
        makespanGlobalTemp = std::max(estado.makespanAtual, makespanGlobalTemp);

        EstadoBusca proximoEstado = {
            clienteAtual + 1,
            visitadosCaminhao,
            estado.caminhoesEmUso,
            estado.estacoesAtivadas,
            estado.dronesEmUsoPorEstacao,
            makespanGlobalTemp,
            estado.solucaoParcial
        };

        branchAndBound(proximoEstado, melhorSolucao, melhorMakespan);

        estado.solucaoParcial.rotasCaminhoes[i].sequenciaNos.pop_back();
        estado.solucaoParcial.rotasCaminhoes[i].tempoViagem = tempoAntigo;
    }

    // Opção 2: O caminhão ativa uma estação e atende um cliente
    for (int i = 0; i < totalEstacoes; ++i) {
        for (int j = 0; j < totalRotas; ++j) {
            std::vector<bool> visitadosCaminhao = estado.visitados;
            visitadosCaminhao[clienteAtual] = true;

            estado.solucaoParcial.rotasCaminhoes[j].estacoesVisitadas.push_back(i);
            estado.solucaoParcial.rotasCaminhoes[j].sequenciaNos.push_back(clienteAtual);

            double novoCustoRota = calcularCustoRotaCaminhao(estado.solucaoParcial.rotasCaminhoes[j]);
            double tempoAntigo = estado.solucaoParcial.rotasCaminhoes[j].tempoViagem;
            estado.solucaoParcial.rotasCaminhoes[j].tempoViagem = novoCustoRota;

            std::vector<bool> novasEstacoesAtivadas = estado.estacoesAtivadas;
            novasEstacoesAtivadas[i] = true; 

            double makespanGlobalTemp = 0.0;
            for(const auto& rota : estado.solucaoParcial.rotasCaminhoes) {
                if (rota.tempoViagem > makespanGlobalTemp) makespanGlobalTemp = rota.tempoViagem;
            }
            makespanGlobalTemp = std::max(estado.makespanAtual, makespanGlobalTemp);

            EstadoBusca proximoEstado = {
                clienteAtual + 1,
                visitadosCaminhao,
                estado.caminhoesEmUso,
                novasEstacoesAtivadas,
                estado.dronesEmUsoPorEstacao,
                makespanGlobalTemp,
                estado.solucaoParcial
            };

            branchAndBound(proximoEstado, melhorSolucao, melhorMakespan);

            estado.solucaoParcial.rotasCaminhoes[j].estacoesVisitadas.pop_back();
            estado.solucaoParcial.rotasCaminhoes[j].sequenciaNos.pop_back();
            estado.solucaoParcial.rotasCaminhoes[j].tempoViagem = tempoAntigo;
        }
    }

    // Opção 3: Abre uma nova rota para o caminhão
    if (estado.caminhoesEmUso < instancia_.deposito.qntCaminhoes) {
        std::vector<bool> visitadosCaminhao = estado.visitados;
        visitadosCaminhao[clienteAtual] = true;

        RotaCaminhao novaRota;
        novaRota.id = estado.caminhoesEmUso + 1;
        novaRota.sequenciaNos = {clienteAtual};
        novaRota.tempoViagem = calcularCustoRotaCaminhao(novaRota);

        estado.solucaoParcial.rotasCaminhoes.push_back(novaRota);

        double novoMakespan = std::max(estado.makespanAtual, novaRota.tempoViagem);

        EstadoBusca proximoEstado = {
            clienteAtual + 1,
            visitadosCaminhao,
            estado.caminhoesEmUso + 1,
            estado.estacoesAtivadas,
            estado.dronesEmUsoPorEstacao,
            novoMakespan,
            estado.solucaoParcial
        };

        branchAndBound(proximoEstado, melhorSolucao, melhorMakespan);

        estado.solucaoParcial.rotasCaminhoes.pop_back();
    }
}

// Processamento de opções de Drone
void SolExata::processarOpcoesDrone(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan) {
    int clienteAtual = estado.clienteAtual;
    double velocidadeCaminhao = 1.0; 
    int totalEstacoes = (int)instancia_.estacoes.size();

    for (int i = 0; i < totalEstacoes; ++i) {
        double distEstacaoCliente = calcularDistancia(instancia_.estacoes[i].posicao, instancia_.clientes[clienteAtual].posicao);
        
        if (estado.estacoesAtivadas[i] && distEstacaoCliente <= (double)instancia_.raio && estado.dronesEmUsoPorEstacao[i] < dronesPorEstacao_) {
            
            std::vector<bool> visitadosDrone = estado.visitados;
            visitadosDrone[clienteAtual] = true;
            
            std::vector<int> novoDronesEmUso = estado.dronesEmUsoPorEstacao;
            novoDronesEmUso[i]++;
            
            double custoTempoDrone = (2.0 * distEstacaoCliente) / (velocidadeCaminhao * fatorVelocidadeDrone_);
            double novoMakespanDrone = std::max(estado.makespanAtual, custoTempoDrone);
            
            estado.solucaoParcial.atendimentoDrones.push_back({i, clienteAtual});
            
            EstadoBusca proximoEstado = {
                clienteAtual + 1,
                visitadosDrone,
                estado.caminhoesEmUso,
                estado.estacoesAtivadas,
                novoDronesEmUso,
                novoMakespanDrone,
                estado.solucaoParcial
            };

            branchAndBound(proximoEstado, melhorSolucao, melhorMakespan);
            
            estado.solucaoParcial.atendimentoDrones.pop_back();
        }
    }
}

// Função principal de Branch and Bound
void SolExata::branchAndBound(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan) {
    if (timeoutAtingido_) return;

    // Verifica o cronômetro a cada 10.000 nós
    contadorNos_++;
    if (contadorNos_ >= 10000) {
        contadorNos_ = 0;
        auto agora = std::chrono::steady_clock::now();
        std::chrono::duration<double> duracao = agora - tempoInicio_;
        if (duracao.count() > limiteTempoSegundos_) {
            timeoutAtingido_ = true;
            return;
        }
    }

    // Poda
    double lowerBound = calcularLowerBound(estado);
    if (lowerBound >= melhorMakespan) {
        return;
    }

    int totalClientes = (int)instancia_.clientes.size();

    if (estado.clienteAtual >= totalClientes) {
        if (estado.makespanAtual < melhorMakespan) {
            melhorMakespan = estado.makespanAtual;
            melhorSolucao = estado.solucaoParcial;
        }
        return;
    }

    // Pula clientes que já foram visitados
    if (estado.visitados[estado.clienteAtual]) {
        estado.clienteAtual++;
        branchAndBound(estado, melhorSolucao, melhorMakespan);
        estado.clienteAtual--;
        return;
    }

    // Explora os ramos de caminhões e drones
    processarOpcoesCaminhao(estado, melhorSolucao, melhorMakespan);
    processarOpcoesDrone(estado, melhorSolucao, melhorMakespan);    
}

// Método de inicialização do algoritmo
bool SolExata::resolver(Solucao& melhorSolucao, double upperBoundInicial, double limiteTempoSegundos) {
    int totalClientes = (int)instancia_.clientes.size();
    int totalEstacoes = (int)instancia_.estacoes.size();
    
    // Calcula a tabela de estimativa usada no lower bound antes de iniciar a árvore
    precalcularCustosMinimos();

    limiteTempoSegundos_ = limiteTempoSegundos;
    timeoutAtingido_ = false;
    contadorNos_ = 0;
    tempoInicio_ = std::chrono::steady_clock::now();

    EstadoBusca estadoInicial;
    estadoInicial.visitados.assign(totalClientes, false);
    estadoInicial.estacoesAtivadas.assign(totalEstacoes, false);
    estadoInicial.dronesEmUsoPorEstacao.assign(totalEstacoes, 0);
    estadoInicial.clienteAtual = 0;
    estadoInicial.makespanAtual = 0.0;
    
    estadoInicial.solucaoParcial = melhorSolucao;
    double melhorMakespan = upperBoundInicial;
    
    std::cout << "Iniciando Branch and Bound (Limite: " << limiteTempoSegundos_ << "s) e Upper Bound = " << melhorMakespan << "...\n";
    
    branchAndBound(estadoInicial, melhorSolucao, melhorMakespan);

    if (timeoutAtingido_) {
        std::cout << "AVISO: Limite de 10 minutos atingido! Retornando a melhor solucao obtida.\n";
    }

    if (melhorMakespan >= upperBoundInicial && !timeoutAtingido_) {
        std::cout << "O exato nao encontrou melhora estrita. Mantendo o resultado da heuristica.\n";
        return false;
    }

    melhorSolucao.makespan = melhorMakespan;
    return true;
}