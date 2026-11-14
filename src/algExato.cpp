#include "../include/algExato.hpp"
#include "../include/leitorArquivos.hpp"

#include <cmath>
#include <iostream>
#include <algorithm>

// Construtor 
SolExata::SolExata(const Instancia& instancia, double fatorVelocidadeDrone, int qntDrones) : 
                    instancia_(instancia), fatorVelocidadeDrone_(fatorVelocidadeDrone), dronesPorEstacao_(qntDrones) {}

// Calcula a distancia entre dois nós
double SolExata::calcularDistancia(Coordenada c1, Coordenada c2) const {
    double dx = c1.x - c2.x;
    double dy = c1.y - c2.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Calcula o custo total de uma rota de caminhao
double SolExata::calcularCustoRotaCaminhao(const RotaCaminhao& rota) const {
    if(rota.sequenciaNos.empty() && rota.estacoesVisitadas.empty()) return 0.0;

    double custoTotal = 0.0;
    Coordenada posicaoAtual = instancia_.deposito.posicao;

    // O caminhão passa pelas estações de drone que ele vai ativar
    int totalEstacoesInstancia = (int)instancia_.estacoes.size();
    for (int idEstacao : rota.estacoesVisitadas) {
        if (idEstacao >= 0 && idEstacao < totalEstacoesInstancia) {
            Coordenada posEstacao = instancia_.estacoes[idEstacao].posicao;
            custoTotal += calcularDistancia(posicaoAtual, posEstacao);
            posicaoAtual = posEstacao;
        }
    }

    // Atende os clientes na sequencia da rota
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

    // Retorna ao deposito
    custoTotal += calcularDistancia(posicaoAtual, instancia_.deposito.posicao);

    return custoTotal;
}

// Processamento de opções de Caminhão
void SolExata::processarOpcoesCaminhao(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan) {
    int clienteAtual = estado.clienteAtual;
    int totalRotas = (int)estado.solucaoParcial.rotasCaminhoes.size();
    int totalEstacoes = (int)instancia_.estacoes.size();

    // 1a opção: Atender clientes na rota já existente
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

        // Backtracking
        estado.solucaoParcial.rotasCaminhoes[i].sequenciaNos.pop_back();
        estado.solucaoParcial.rotasCaminhoes[i].tempoViagem = tempoAntigo;
    }

    // 2a opção: Ativar estação e atender clientes
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

            // Backtracking
            estado.solucaoParcial.rotasCaminhoes[j].estacoesVisitadas.pop_back();
            estado.solucaoParcial.rotasCaminhoes[j].sequenciaNos.pop_back();
            estado.solucaoParcial.rotasCaminhoes[j].tempoViagem = tempoAntigo;
        }
    }

    // 3a opção: Abrir nova rota para o caminhão explorar
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
        
        // Verifica ativação, raio de alcance e capacidade de drones na estação
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
    // Poda por bound
    if (estado.makespanAtual >= melhorMakespan) {
        return;
    }

    int totalClientes = (int)instancia_.clientes.size();

    // Condição de parada: todos os clientes foram avaliados
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

    // Explora as ramificações de caminhões e drones
    processarOpcoesCaminhao(estado, melhorSolucao, melhorMakespan);
    processarOpcoesDrone(estado, melhorSolucao, melhorMakespan);    
}

// Método inicializador do solver
bool SolExata::resolver(Solucao& melhorSolucao, double upperBoundInicial) {
    int totalClientes = (int)instancia_.clientes.size();
    int totalEstacoes = (int)instancia_.estacoes.size();
    
    EstadoBusca estadoInicial;
    estadoInicial.visitados.assign(totalClientes, false);
    estadoInicial.estacoesAtivadas.assign(totalEstacoes, false);
    estadoInicial.dronesEmUsoPorEstacao.assign(totalEstacoes, 0);
    estadoInicial.clienteAtual = 0;
    estadoInicial.makespanAtual = 0.0;
    
    // Inicializa a solução parcial com a da heurística passada como Upper Bound
    estadoInicial.solucaoParcial = melhorSolucao;
    
    double melhorMakespan = upperBoundInicial;
    
    std::cout << "[SolExata] Iniciando Branch and Bound com UB = " << melhorMakespan << "...\n";
    
    branchAndBound(estadoInicial, melhorSolucao, melhorMakespan);

    if (melhorMakespan >= upperBoundInicial) {
        std::cout << "[SolExata] O exato não encontrou melhora estrita. Mantendo o resultado da heurística.\n";
        return false;
    }

    melhorSolucao.makespan = melhorMakespan;
    return true;
}