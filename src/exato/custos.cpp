#include "algExato.hpp"
#include "leitorArquivos.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

// Calcula a distancia entre dois nós
double SolExata::calcularDistancia(Coordenada c1, Coordenada c2) const {
    double dx = c1.x - c2.x;
    double dy = c1.y - c2.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Calcula o menor tempo possível para atender cada cliente (limite inferior individual)
void SolExata::precalcularCustosMinimos() {
    int totalClientes = (int)instancia_.clientes.size();
    int totalEstacoes = (int)instancia_.estacoes.size();
    custoMinimoCliente_.assign(totalClientes, 0.0);

    for (int i = 0; i < totalClientes; ++i) {
        Coordenada posCli = instancia_.clientes[i].posicao;

        double menorCusto = 2.0 * calcularDistancia(posCli, instancia_.deposito.posicao);

        // Atendimento por drone
        if (dronesPorEstacao_ > 0) {
            for (int e = 0; e < totalEstacoes; ++e) {
                double distEstacao = calcularDistancia(posCli, instancia_.estacoes[e].posicao);
                if (distEstacao <= (double)instancia_.raio) {
                    double tempoDrone = (2.0 * distEstacao) / fatorVelocidadeDrone_;
                    if (tempoDrone < menorCusto) menorCusto = tempoDrone;
                }
            }
        }

        custoMinimoCliente_[i] = menorCusto;
    }
}

// Calcula o Lower Bound
double SolExata::calcularLowerBound(const EstadoBusca& estado) const {
    int totalClientes = (int)instancia_.clientes.size();

    double maiorCustoIsolado = 0.0;
    for (int i = 0; i < totalClientes; ++i) {
        if (!estado.visitados[i]) {
            maiorCustoIsolado = std::max(maiorCustoIsolado, custoMinimoCliente_[i]);
        }
    }

    return std::max(estado.makespanAtual, maiorCustoIsolado);
}

// Calcula o custo total da rota de caminhao
// Notação dos nós: cliente = indice + 1, estação = -(indice + 1)
double SolExata::calcularCustoRotaCaminhao(const RotaCaminhao& rota) const {
    if (rota.sequenciaNos.empty()) return 0.0;

    double custoTotal = 0.0;
    Coordenada posicaoAtual = instancia_.deposito.posicao;

    for (int no : rota.sequenciaNos) {
        Coordenada posProximo;
        
        if (no > 0) {
            posProximo = instancia_.clientes[no - 1].posicao;
        } else {
            posProximo = instancia_.estacoes[-no - 1].posicao;
        }

        custoTotal += calcularDistancia(posicaoAtual, posProximo);
        posicaoAtual = posProximo;
    }

    // Retorna ao depósito ao final da rota
    custoTotal += calcularDistancia(posicaoAtual, instancia_.deposito.posicao);

    return custoTotal;
}