#include "../include/algExato.hpp"
#include "../include/leitorArquivos.hpp"

#include <cmath>
#include <iostream>

// OBS: Checar se o drone só pode visitar um cliente e tem que voltar pra estação ou se pode 
// entregar mais de um pacote na definicao do problema

// Construtor 
SolExata::SolExata(const Instancia& instancia, double fatorVelocidadeDrone, int qntDrones) : 
                    instancia_(instancia), fatorVelocidadeDrone_(fatorVelocidadeDrone), dronesPorEstacao_(qntDrones) {}


// Calcula a distancia entre dois nós
double SolExata::calcularDistancia(Coordenada c1, Coordenada c2) const {
    return std::sqrt(std::pow(c1.x - c2.x, 2) + std::pow(c1.y - c2.y, 2));
}

// Calcula o custo total de uma rota de caminhao
double SolExata::calcularCustoRotaCaminhao(const RotaCaminhao& rota) const {
    if(rota.sequenciaNos.empty() && rota.estacoesVisitadas.empty()) return 0;

    double custoTotal = 0;
    Coordenada posicaoAtual = instancia_.deposito.posicao;

    // O caminhao passa pelas estacoes de drone que ele vai ativar
    for (int idEstacao : rota.estacoesVisitadas) {
        Coordenada posEstacao = instancia_.estacoes[idEstacao].posicao;
        custoTotal += calcularDistancia(posicaoAtual, posEstacao);
        posicaoAtual = posEstacao;
    }

    // Atende os clientes na sequencia da rota
    for (int idCliente : rota.sequenciaNos) {
        Coordenada posCliente = instancia_.clientes[idCliente].posicao;
        custoTotal += calcularDistancia(posicaoAtual, posCliente);
        posicaoAtual = posCliente;
    }

    // Retorna ao deposito
    custoTotal += calcularDistancia(posicaoAtual, instancia_.deposito.posicao);

    return custoTotal;
}

// Entrega por Caminhao (modularizar!!!)
void SolExata::processarOpcoesCaminhao(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan) {
    int clienteAtual = estado.clienteAtual;

    // Testa atender o cliente atual por meio de um caminhão que já possui rota ativa
    for (int i = 0; i < estado.solucaoParcial.rotasCaminhoes.size(); ++i) {
        std::vector<bool> visitadosCaminhao = estado.visitados;
        visitadosCaminhao[clienteAtual] = true;

        estado.solucaoParcial.rotasCaminhoes[i].sequenciaNos.push_back(clienteAtual);
        double novoCustoRota = calcularCustoRotaCaminhao(estado.solucaoParcial.rotasCaminhoes[i]);
        double tempoAntigo = estado.solucaoParcial.rotasCaminhoes[i].tempoViagem;
        estado.solucaoParcial.rotasCaminhoes[i].tempoViagem = novoCustoRota;

        double makespanGlobalTemp = 0.0;
        // Pegamos a rota mais longa para ser o makespan, pois os automoveis trabalham em paralelo
        for(const auto& rota : estado.solucaoParcial.rotasCaminhoes) {
            if (rota.tempoViagem > makespanGlobalTemp) makespanGlobalTemp = rota.tempoViagem;
        }
        makespanGlobalTemp = std::max(estado.makespanAtual, makespanGlobalTemp);

        // Avanca para o proximo cliente
        EstadoBusca proximoEstado = {
            clienteAtual + 1, visitadosCaminhao, estado.caminhoesEmUso, estado.estacoesAtivadas, 
            estado.dronesEmUsoPorEstacao, makespanGlobalTemp, estado.solucaoParcial
        };

        branchAndBound(proximoEstado, melhorSolucao, melhorMakespan);

        // Backtracking
        estado.solucaoParcial.rotasCaminhoes[i].sequenciaNos.pop_back();
        estado.solucaoParcial.rotasCaminhoes[i].tempoViagem = tempoAntigo;
    }

    // Testa passar por uma estacao de drone durante as entregas
    for (int i = 0; i < instancia_.estacoes.size(); ++i) {
        for (int j = 0; j < estado.solucaoParcial.rotasCaminhoes.size(); ++j) {
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
                clienteAtual + 1, visitadosCaminhao, estado.caminhoesEmUso, novasEstacoesAtivadas, 
                estado.dronesEmUsoPorEstacao, makespanGlobalTemp, estado.solucaoParcial
            };

            branchAndBound(proximoEstado, melhorSolucao, melhorMakespan);

            // Backtracking
            estado.solucaoParcial.rotasCaminhoes[j].estacoesVisitadas.pop_back();
            estado.solucaoParcial.rotasCaminhoes[j].sequenciaNos.pop_back();
            estado.solucaoParcial.rotasCaminhoes[j].tempoViagem = tempoAntigo;
        }
    }

    // Testa abrir uma nova rota
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
            clienteAtual + 1, visitadosCaminhao, estado.caminhoesEmUso + 1, estado.estacoesAtivadas, 
            estado.dronesEmUsoPorEstacao, novoMakespan, estado.solucaoParcial
        };

        branchAndBound(proximoEstado, melhorSolucao, melhorMakespan);

        estado.solucaoParcial.rotasCaminhoes.pop_back();
    }
}

// Entrega por Drone
void SolExata::processarOpcoesDrone(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan) {
    int clienteAtual = estado.clienteAtual;
    double velocidadeCaminhao = 1.0; 

    for(int i = 0; i < instancia_.estacoes.size(); ++i) {
        double distEstacaoCliente = calcularDistancia(instancia_.estacoes[i].posicao, instancia_.clientes[clienteAtual].posicao);
        
        // Verifica se a estacao esta ativada, se esta dentro do raio de alcance e se há drone disponivel para realizar a entrega
        if(estado.estacoesAtivadas[i] && distEstacaoCliente <= instancia_.raio && estado.dronesEmUsoPorEstacao[i] < dronesPorEstacao_) {
            std::vector<bool> visitadosDrone = estado.visitados;
            visitadosDrone[clienteAtual] = true;
            
            std::vector<int> novoDronesEmUso = estado.dronesEmUsoPorEstacao;
            novoDronesEmUso[i]++;
            
            double custoTempoDrone = (2.0 * distEstacaoCliente) / (velocidadeCaminhao * fatorVelocidadeDrone_);
            double novoMakespanDrone = std::max(estado.makespanAtual, custoTempoDrone);
            
            estado.solucaoParcial.atendimentoDrones.push_back({i, clienteAtual});
            
            EstadoBusca proximoEstado = {
                clienteAtual + 1, visitadosDrone, estado.caminhoesEmUso, estado.estacoesAtivadas, 
                novoDronesEmUso, novoMakespanDrone, estado.solucaoParcial
            };

            branchAndBound(proximoEstado, melhorSolucao, melhorMakespan);
            
            estado.solucaoParcial.atendimentoDrones.pop_back();
        }
    }
}

// Implementacao do Branch and Bound
void SolExata::branchAndBound(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan) {
    
    // Poda
    if(estado.makespanAtual >= melhorMakespan) {
        return;
    }

    // Atualiza melhor makespan
    if(estado.clienteAtual == instancia_.clientes.size()) {
        if(estado.makespanAtual < melhorMakespan) {
            melhorMakespan = estado.makespanAtual;
            melhorSolucao = estado.solucaoParcial;
        }
        return;
    }

    // Pula clientes que já foram visitados
    if(estado.visitados[estado.clienteAtual]) {
        estado.clienteAtual++;
        branchAndBound(estado, melhorSolucao, melhorMakespan);
        return;
    }

    processarOpcoesCaminhao(estado, melhorSolucao, melhorMakespan);
    processarOpcoesDrone(estado, melhorSolucao, melhorMakespan);    
}

// Chama o branch and bound e retorna a solução
bool SolExata::resolver(Solucao& melhorSolucao, double upperBoundInicial = INFINITY) {
    int totalClientes = instancia_.clientes.size();
    int totalEstacoes = instancia_.estacoes.size();
    
    EstadoBusca estadoInicial;
    
    double melhorMakespan = upperBoundInicial;
    
    std::cout << "Iniciando Branch and Bound...\n";
    
    branchAndBound(estadoInicial, melhorSolucao, melhorMakespan);

    if (melhorMakespan >= upperBoundInicial) {
        std::cerr << "Nenhuma solução viável encontrada.\n";
        return false;
    }

    melhorSolucao.makespan = melhorMakespan;
    return true;
}