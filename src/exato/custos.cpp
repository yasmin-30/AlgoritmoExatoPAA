#include "algExato.hpp"
#include "leitorArquivos.hpp"

#include <cmath>
#include <limits>

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
    if (rota.sequenciaNos.empty()) return 0.0;

    double custoTotal = 0.0;
    Coordenada posicaoAtual = instancia_.deposito.posicao;

    for (int no : rota.sequenciaNos) {
        Coordenada posProximo;

        if (no > 0) {
            // Verifica se o nó é um cliente
            int idCliente = no;
            bool encontrado = false;
            for (const auto& cli : instancia_.clientes) {
                if (cli.idCliente == idCliente) {
                    posProximo = cli.posicao;
                    encontrado = true;
                    break;
                }
            }
            if (!encontrado) continue; // Segurança
        } 
        else {
            // Verifica se o nó é uma estação
            int idxEstacao = (-no) - 1; 
            if (idxEstacao >= 0 && idxEstacao < (int)instancia_.estacoes.size()) {
                posProximo = instancia_.estacoes[idxEstacao].posicao;
            } else {
                continue;
            }
        }

        // Soma a distância do ponto atual até o próximo nó
        custoTotal += calcularDistancia(posicaoAtual, posProximo);
        posicaoAtual = posProximo;
    }

    // Retorna ao depósito ao final da rota
    custoTotal += calcularDistancia(posicaoAtual, instancia_.deposito.posicao);

    return custoTotal;
}