#include "algExato.hpp"
#include "leitorArquivos.hpp"

#include <iostream>

// Construtor 
SolExata::SolExata(const Instancia& instancia, double fatorVelocidadeDrone, int qntDrones) : 
                    instancia_(instancia), fatorVelocidadeDrone_(fatorVelocidadeDrone), dronesPorEstacao_(qntDrones) {}

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
        std::cout << "AVISO: Limite de tempo atingido! Retornando a melhor solucao obtida.\n";
    }

    if (melhorMakespan >= upperBoundInicial && !timeoutAtingido_) {
        std::cout << "O exato nao encontrou melhora estrita. Mantendo o resultado da heuristica.\n";
        return false;
    }

    melhorSolucao.makespan = melhorMakespan;
    return true;
}