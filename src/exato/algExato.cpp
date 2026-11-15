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

    // Poda por Lower Bound
    double lowerBound = calcularLowerBound(estado);
    if (lowerBound >= melhorMakespan) {
        return;
    }

    int totalClientes = (int)instancia_.clientes.size();

    // Procura o próximo cliente que ainda não foi visitado
    int proximoClienteNaoVisitado = -1;
    for (int i = 0; i < totalClientes; ++i) {
        if (!estado.visitados[i]) {
            proximoClienteNaoVisitado = i;
            break;
        }
    }

    // Verifca se não há mais clientes pendentes
    if (proximoClienteNaoVisitado == -1) {
        if (estado.makespanAtual < melhorMakespan) {
            melhorMakespan = estado.makespanAtual;
            melhorSolucao = estado.solucaoParcial;
        }
        return;
    }

    estado.clienteAtual = proximoClienteNaoVisitado;

    // Explora as opções de atendimento para o cliente
    processarOpcoesCaminhao(estado, melhorSolucao, melhorMakespan);
    processarOpcoesDrone(estado, melhorSolucao, melhorMakespan);    
}

// Método de inicialização do algoritmo
bool SolExata::resolver(Solucao& melhorSolucao, double upperBoundInicial, double limiteTempoSegundos) {
    int totalClientes = (int)instancia_.clientes.size();
    int totalEstacoes = (int)instancia_.estacoes.size();

    precalcularCustosMinimos();

    limiteTempoSegundos_ = limiteTempoSegundos;
    timeoutAtingido_ = false;
    contadorNos_ = 0;
    tempoInicio_ = std::chrono::steady_clock::now();

    EstadoBusca estadoInicial;
    estadoInicial.visitados.assign(totalClientes, false);
    estadoInicial.estacoesAtivadas.assign(totalEstacoes, false);
    estadoInicial.cargaDrones.assign(totalEstacoes, std::vector<double>(dronesPorEstacao_, 0.0));
    estadoInicial.caminhoesEmUso = 0;
    estadoInicial.clienteAtual = 0;
    estadoInicial.makespanAtual = 0.0;
    estadoInicial.solucaoParcial = Solucao{};

    double melhorMakespan = upperBoundInicial;

    std::cout << "Iniciando Branch and Bound (Limite: " << limiteTempoSegundos_
              << "s) e Upper Bound = " << melhorMakespan << "...\n";

    branchAndBound(estadoInicial, melhorSolucao, melhorMakespan);

    if (timeoutAtingido_) {
        std::cout << "AVISO: Limite de tempo atingido! Retornando a melhor solucao obtida.\n";
    }

    if (melhorMakespan >= upperBoundInicial) {
        std::cout << "O exato nao encontrou melhora, mantendo o resultado da heuristica.\n";
        return false;
    }

    melhorSolucao.makespan = melhorMakespan;
    return true;
}