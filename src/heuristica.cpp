#include "../include/heuristica.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <unordered_set>
#include <vector>
#include <unordered_map>

double calcularDistancia(const Coordenada& a, const Coordenada& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<int> clientesNoAlcance(const EstacaoDrone& estacao, const std::vector<Cliente>& clientes) {
    std::vector<int> clientesAlcancaveis;
    for (const Cliente& cliente : clientes) {
        double distancia = calcularDistancia(estacao.posicao, cliente.posicao);
        if (distancia <= estacao.alcance) {
            clientesAlcancaveis.push_back(cliente.idCliente);
        }
    }
    return clientesAlcancaveis;
}

std::vector<int> selecionarEstacoes(const Instancia& instancia) {
    std::vector<int> estacoesSelecionadas;

    for (const EstacaoDrone& estacao : instancia.estacoes) {
        estacoesSelecionadas.push_back(estacao.idEstacao);
    }

    return estacoesSelecionadas;
}

std::vector<std::pair<int, int>> atribuirClientes(const Instancia& instancia,
    const std::vector<int>& estacoesSelecionadas) {
    std::vector<std::pair<int, int>> atribuicoes;

    for (const Cliente& cliente : instancia.clientes) {
        double menorDistancia = std::numeric_limits<double>::max();
        int melhorEstacao = -1;
        for (int idEstacao : estacoesSelecionadas) {
            for (const EstacaoDrone& estacao : instancia.estacoes) {
                if (estacao.idEstacao != idEstacao) {
                    continue;
                }
                double distancia = calcularDistancia(
                    cliente.posicao,
                    estacao.posicao
                );

                if (distancia <= estacao.alcance &&
                    distancia < menorDistancia) {
                    menorDistancia = distancia;
                    melhorEstacao = estacao.idEstacao;
                }
                break;
            }
        }

        if (melhorEstacao != -1) {
            atribuicoes.push_back({melhorEstacao, cliente.idCliente});
        }
    }
    return atribuicoes;
}

std::vector<RotaCaminhao> construirRotas(const Instancia& instancia, const std::vector<int>& estacoesSelecionadas,
    const std::vector<std::pair<int, int>>& atribuicoes) {
    const int numCaminhoes = instancia.deposito.qntCaminhoes;

    if (numCaminhoes <= 0) {
        throw std::invalid_argument("A instancia deve ter ao menos um caminhao.");
    }
    const Coordenada& deposito = instancia.deposito.posicao;

    // Guarda os clientes atendidos por drones e as estacoes utilizadas.
    std::unordered_set<int> clientesPorDrone;
    std::unordered_set<int> estacoesUtilizadas;

    for (const auto& atribuicao : atribuicoes) {
        int idEstacao = atribuicao.first;
        int idCliente = atribuicao.second;
        if (std::find(
                estacoesSelecionadas.begin(),
                estacoesSelecionadas.end(),
                idEstacao
            ) != estacoesSelecionadas.end()) {
            clientesPorDrone.insert(idCliente);
            estacoesUtilizadas.insert(idEstacao);
        }
    }

    // Converte o ID do no em sua coordenada.
    // 0 = deposito, positivo = cliente, negativo = estacao.
    auto obterPosicao = [&](int idNo) -> Coordenada {
        if (idNo == 0) {
            return deposito;
        }

        if (idNo > 0) {
            for (const Cliente& cliente : instancia.clientes) {
                if (cliente.idCliente == idNo) {
                    return cliente.posicao;
                }
            }
        } else {
            int idEstacao = -idNo;

            for (const EstacaoDrone& estacao : instancia.estacoes) {
                if (estacao.idEstacao == idEstacao) {
                    return estacao.posicao;
                }
            }
        }
        throw std::invalid_argument("ID de no inexistente.");
    };

    // Monta a lista de nos que precisam ser visitados por caminhoes.
    std::vector<int> nos;

    for (const Cliente& cliente : instancia.clientes) {
        if (clientesPorDrone.find(cliente.idCliente) ==
            clientesPorDrone.end()) {
            nos.push_back(cliente.idCliente);
        }
    }

    for (int idEstacao : estacoesUtilizadas) {
        nos.push_back(-idEstacao);
    }

    // sweep: ordena os nos pelo angulo em relacao ao deposito.
    std::sort(nos.begin(), nos.end(), [&](int a, int b) {
        Coordenada posA = obterPosicao(a);
        Coordenada posB = obterPosicao(b);

        double anguloA = std::atan2(
            posA.y - deposito.y,
            posA.x - deposito.x
        );

        double anguloB = std::atan2(
            posB.y - deposito.y,
            posB.x - deposito.x
        );
        return anguloA < anguloB;
    });

    // Divide os nos em grupos contiguos segundo a ordem Sweep.
    std::vector<std::vector<int>> grupos(numCaminhoes);
    const int totalNos = static_cast<int>(nos.size());
    const int base = totalNos / numCaminhoes;
    const int resto = totalNos % numCaminhoes;
    int indice = 0;

    for (int i = 0; i < numCaminhoes; ++i) {
        int quantidade = base + (i < resto ? 1 : 0);
        for (int j = 0; j < quantidade; ++j) {
            grupos[i].push_back(nos[indice++]);
        }
    }

    // Ordena cada grupo pelo vizinho mais proximo.
    std::vector<RotaCaminhao> rotas;
    rotas.reserve(numCaminhoes);

    for (int i = 0; i < numCaminhoes; ++i) {
        RotaCaminhao rota;
        rota.id = i + 1;
        rota.tempoViagem = 0.0;

        rota.sequenciaNos.push_back(0);
        Coordenada posAtual = deposito;
        std::vector<int> pendentes = grupos[i];

        while (!pendentes.empty()) {
            auto maisProximo = pendentes.begin();
            double menorDistancia = calcularDistancia(
                posAtual,
                obterPosicao(*maisProximo)
            );

            for (auto it = pendentes.begin() + 1;
                 it != pendentes.end(); ++it) {
                double distancia = calcularDistancia(
                    posAtual,
                    obterPosicao(*it)
                );
                if (distancia < menorDistancia) {
                    menorDistancia = distancia;
                    maisProximo = it;
                }
            }

            int proximoNo = *maisProximo;
            rota.tempoViagem += menorDistancia;
            rota.sequenciaNos.push_back(proximoNo);
            posAtual = obterPosicao(proximoNo);

            pendentes.erase(maisProximo);
        }

        // Retorna ao deposito.
        rota.tempoViagem += calcularDistancia(posAtual, deposito);
        rota.sequenciaNos.push_back(0);

        rotas.push_back(rota);
    }

    return rotas;
}

std::unordered_map<int, double> calcularTempoDrones(const Instancia& instancia,
    const std::vector<std::pair<int, int>>& atribuicoes, int dronesPorEstacao, double alpha) {
    if (dronesPorEstacao <= 0 || alpha <= 0.0) {
        throw std::invalid_argument(
            "A quantidade de drones e alpha devem ser positivos."
        );
    }

    // Guarda os tempos de entrega de cada estacaoo.
    std::unordered_map<int, std::vector<double>> temposPorEstacao;

    for (const auto& atribuicao : atribuicoes) {
        int idEstacao = atribuicao.first;
        int idCliente = atribuicao.second;
        const Cliente* clienteEncontrado = nullptr;
        const EstacaoDrone* estacaoEncontrada = nullptr;

        for (const Cliente& cliente : instancia.clientes) {
            if (cliente.idCliente == idCliente) {
                clienteEncontrado = &cliente;
                break;
            }
        }
        for (const EstacaoDrone& estacao : instancia.estacoes) {
            if (estacao.idEstacao == idEstacao) {
                estacaoEncontrada = &estacao;
                break;
            }
        }

        if (clienteEncontrado == nullptr || estacaoEncontrada == nullptr) {
            throw std::invalid_argument(
                "Cliente ou estacao inexistente em uma atribuicao."
            );
        }

        double distancia = calcularDistancia(
            estacaoEncontrada->posicao,
            clienteEncontrado->posicao
        );

        // Ida e volta: a velocidade do caminhão é considerada 1.
        double tempo = (2.0 * distancia) / alpha;

        temposPorEstacao[idEstacao].push_back(tempo);
    }

    std::unordered_map<int, double> resultado;

    for (auto& entrada : temposPorEstacao) {
        int idEstacao = entrada.first;
        std::vector<double>& tempos = entrada.second;

        // Distribui primeiro as entregas mais demoradas.
        std::sort(tempos.begin(), tempos.end(), std::greater<double>());

        // Cada posição representa um drone e sua carga de trabalho.
        std::vector<double> cargaDrones(dronesPorEstacao, 0.0);

        for (double tempo : tempos) {
            auto droneMenosOcupado = std::min_element(
                cargaDrones.begin(),
                cargaDrones.end()
            );

            *droneMenosOcupado += tempo;
        }

        // Tempo para concluir todas as entregas daquela estação.
        resultado[idEstacao] = *std::max_element(
            cargaDrones.begin(),
            cargaDrones.end()
        );
    }

    return resultado;
}

Solucao executarHeuristica(
    const Instancia& instancia,
    int dronesPorEstacao,
    double alpha
) {
    // 1. Seleciona as estacoes disponiveis.
    std::vector<int> estacoesSelecionadas =
        selecionarEstacoes(instancia);

    // 2. Atribui os clientes que podem ser atendidos por drones.
    std::vector<std::pair<int, int>> atribuicoes =
        atribuirClientes(instancia, estacoesSelecionadas);

    // 3. Constroi as rotas dos caminhoes.
    std::vector<RotaCaminhao> rotas =
        construirRotas(
            instancia,
            estacoesSelecionadas,
            atribuicoes
        );

    // 4. Calcula o tempo de operacao dos drones por estacao.
    std::unordered_map<int, double> temposDrones =
        calcularTempoDrones(
            instancia,
            atribuicoes,
            dronesPorEstacao,
            alpha
        );

    // 5. Monta a solucao.
    Solucao solucao{};
    solucao.rotasCaminhoes = rotas;
    solucao.atendimentoDrones = atribuicoes;
    solucao.makespan = 0.0;

    // Converte o ID de um no em sua coordenada.
    // 0 = deposito, positivo = cliente, negativo = estacao.
    auto obterPosicao = [&](int idNo) -> Coordenada {
        if (idNo == 0) {
            return instancia.deposito.posicao;
        }

        if (idNo > 0) {
            for (const Cliente& cliente : instancia.clientes) {
                if (cliente.idCliente == idNo) {
                    return cliente.posicao;
                }
            }
        } else {
            int idEstacao = -idNo;

            for (const EstacaoDrone& estacao : instancia.estacoes) {
                if (estacao.idEstacao == idEstacao) {
                    return estacao.posicao;
                }
            }
        }

        throw std::invalid_argument("ID de no inexistente.");
    };

    // 6. Calcula o makespan considerando caminhoes e drones.
    for (const RotaCaminhao& rota : solucao.rotasCaminhoes) {
        // O tempo de retorno do caminhao e um limite para o makespan.
        solucao.makespan = std::max(
            solucao.makespan,
            rota.tempoViagem
        );

        if (rota.sequenciaNos.empty()) {
            continue;
        }

        double tempoAcumulado = 0.0;

        for (std::size_t i = 1;
             i < rota.sequenciaNos.size();
             ++i) {

            int noAnterior = rota.sequenciaNos[i - 1];
            int noAtual = rota.sequenciaNos[i];

            // Tempo acumulado ate a chegada ao no atual.
            tempoAcumulado += calcularDistancia(
                obterPosicao(noAnterior),
                obterPosicao(noAtual)
            );

            // Nos negativos representam estacoes de drones.
            if (noAtual < 0) {
                int idEstacao = -noAtual;

                auto tempoDrone = temposDrones.find(idEstacao);

                if (tempoDrone != temposDrones.end()) {
                    // As entregas comecam quando o caminhao
                    // chega e ativa a estacao.
                    double conclusaoDrones =
                        tempoAcumulado + tempoDrone->second;

                    solucao.makespan = std::max(
                        solucao.makespan,
                        conclusaoDrones
                    );
                }
            }
        }
    }

    return solucao;
}