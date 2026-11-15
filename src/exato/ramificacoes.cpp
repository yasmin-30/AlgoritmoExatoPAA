#include "algExato.hpp"
#include "leitorArquivos.hpp"
#include <algorithm>

// Recalcula o makespan do estado a partir das rotas e da carga dos drones
// Convenção dos nós: cliente = indice + 1, estação = -(indice + 1)
double SolExata::calcularMakespanParcial(const Solucao& sol, const std::vector<std::vector<double>>& carga) const {
    double makespan = 0.0;

    for (const auto& rota : sol.rotasCaminhoes) {
        makespan = std::max(makespan, rota.tempoViagem);

        Coordenada atual = instancia_.deposito.posicao;
        double tempoChegada = 0.0;

        for (int no : rota.sequenciaNos) {
            Coordenada prox = (no > 0) ? instancia_.clientes[no - 1].posicao
                                       : instancia_.estacoes[-no - 1].posicao;
            tempoChegada += calcularDistancia(atual, prox);
            atual = prox;

            if (no < 0) {
                int e = -no - 1;
                if (!carga[e].empty()) {
                    double maiorCarga = *std::max_element(carga[e].begin(), carga[e].end());
                    makespan = std::max(makespan, tempoChegada + maiorCarga);
                }
            }
        }
    }
    return makespan;
}

// Processamento de opções de Caminhão
void SolExata::processarOpcoesCaminhao(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan) {
    int clienteAtual = estado.clienteAtual;
    int totalRotas = (int)estado.solucaoParcial.rotasCaminhoes.size();
    int totalEstacoes = (int)instancia_.estacoes.size();

    // Opção 1: o caminhão atende o cliente diretamente
    for (int i = 0; i < totalRotas; ++i) {
        processarOpcaoCaminhaoExistente(estado, melhorSolucao, melhorMakespan, clienteAtual, i);
    }

    // Opção 2: o caminhão ativa uma estação que alcança o cliente
    if (dronesPorEstacao_ > 0) {
        for (int e = 0; e < totalEstacoes; ++e) {
            if (estado.estacoesAtivadas[e]) continue;

            double d = calcularDistancia(instancia_.estacoes[e].posicao,
                                         instancia_.clientes[clienteAtual].posicao);
            if (d > (double)instancia_.raio) continue;

            for (int j = 0; j < totalRotas; ++j) {
                processarOpcaoAtivarEstacao(estado, melhorSolucao, melhorMakespan, clienteAtual, e, j);
            }
            // Estação em uma rota nova
            if (estado.caminhoesEmUso < instancia_.deposito.qntCaminhoes) {
                processarOpcaoAtivarEstacao(estado, melhorSolucao, melhorMakespan, clienteAtual, e, -1);
            }
        }
    }

    // Opção 3: Abre uma nova rota para o caminhão
    if (estado.caminhoesEmUso < instancia_.deposito.qntCaminhoes) {
        processarOpcaoNovaRota(estado, melhorSolucao, melhorMakespan, clienteAtual);
    }
}

// O caminhão atende o cliente em qualquer posição da rota existente
void SolExata::processarOpcaoCaminhaoExistente(EstadoBusca& estado, Solucao& melhorSolucao,
                                               double& melhorMakespan, int clienteAtual, int i) {
    int tam = (int)estado.solucaoParcial.rotasCaminhoes[i].sequenciaNos.size();

    for (int p = 0; p <= tam; ++p) {
        EstadoBusca prox = estado;
        prox.visitados[clienteAtual] = true;

        RotaCaminhao& rota = prox.solucaoParcial.rotasCaminhoes[i];
        rota.sequenciaNos.insert(rota.sequenciaNos.begin() + p, clienteAtual + 1);
        rota.tempoViagem = calcularCustoRotaCaminhao(rota);

        prox.makespanAtual = calcularMakespanParcial(prox.solucaoParcial, prox.cargaDrones);
        if (prox.makespanAtual >= melhorMakespan) continue;

        branchAndBound(prox, melhorSolucao, melhorMakespan);
    }
}

// O caminhão passa por uma estação e a ativa
void SolExata::processarOpcaoAtivarEstacao(EstadoBusca& estado, Solucao& melhorSolucao,
                                           double& melhorMakespan, int clienteAtual,
                                           int estacao, int rotaIdx) {
    int tam = 0;
    if (rotaIdx >= 0) {
        tam = (int)estado.solucaoParcial.rotasCaminhoes[rotaIdx].sequenciaNos.size();
    }

    for (int p = 0; p <= tam; ++p) {
        EstadoBusca prox = estado;
        prox.estacoesAtivadas[estacao] = true;

        int idx = rotaIdx;
        if (idx < 0) {
            RotaCaminhao nova;
            nova.id = prox.caminhoesEmUso + 1;
            nova.tempoViagem = 0.0;
            prox.solucaoParcial.rotasCaminhoes.push_back(nova);
            prox.caminhoesEmUso++;
            idx = (int)prox.solucaoParcial.rotasCaminhoes.size() - 1;
        }

        RotaCaminhao& rota = prox.solucaoParcial.rotasCaminhoes[idx];
        rota.estacoesVisitadas.push_back(estacao);
        rota.sequenciaNos.insert(rota.sequenciaNos.begin() + p, -(estacao + 1));
        rota.tempoViagem = calcularCustoRotaCaminhao(rota);

        prox.makespanAtual = calcularMakespanParcial(prox.solucaoParcial, prox.cargaDrones);
        if (prox.makespanAtual >= melhorMakespan) continue;

        branchAndBound(prox, melhorSolucao, melhorMakespan);
    }
}

// Abre uma nova rota para o caminhão
void SolExata::processarOpcaoNovaRota(EstadoBusca& estado, Solucao& melhorSolucao,
                                      double& melhorMakespan, int clienteAtual) {
    EstadoBusca prox = estado;
    prox.visitados[clienteAtual] = true;

    RotaCaminhao novaRota;
    novaRota.id = estado.caminhoesEmUso + 1;
    novaRota.sequenciaNos = {clienteAtual + 1};
    novaRota.tempoViagem = calcularCustoRotaCaminhao(novaRota);

    prox.solucaoParcial.rotasCaminhoes.push_back(novaRota);
    prox.caminhoesEmUso = estado.caminhoesEmUso + 1;

    prox.makespanAtual = calcularMakespanParcial(prox.solucaoParcial, prox.cargaDrones);
    if (prox.makespanAtual >= melhorMakespan) return;

    branchAndBound(prox, melhorSolucao, melhorMakespan);
}

// Processamento de opções de Drone
void SolExata::processarOpcoesDrone(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan) {
    int clienteAtual = estado.clienteAtual;
    int totalEstacoes = (int)instancia_.estacoes.size();
    const double velocidadeCaminhao = 1.0;

    for (int i = 0; i < totalEstacoes; ++i) {
        if (!estado.estacoesAtivadas[i]) continue;

        double dist = calcularDistancia(instancia_.estacoes[i].posicao, instancia_.clientes[clienteAtual].posicao);
        if (dist > (double)instancia_.raio) continue;

        double tempoVoo = (2.0 * dist) / (velocidadeCaminhao * fatorVelocidadeDrone_);

        for (int d = 0; d < (int)estado.cargaDrones[i].size(); ++d) {
            // Drones com o mesmo tempo de voo são equivalentes
            bool repetido = false;
            for (int k = 0; k < d; ++k) {
                if (estado.cargaDrones[i][k] == estado.cargaDrones[i][d]) { repetido = true; break; }
            }
            if (repetido) continue;

            EstadoBusca prox = estado;
            prox.visitados[clienteAtual] = true;
            prox.cargaDrones[i][d] += tempoVoo;
            prox.solucaoParcial.atendimentoDrones.push_back({i, clienteAtual});

            prox.makespanAtual = calcularMakespanParcial(prox.solucaoParcial, prox.cargaDrones);
            if (prox.makespanAtual >= melhorMakespan) continue;

            branchAndBound(prox, melhorSolucao, melhorMakespan);
        }
    }
}