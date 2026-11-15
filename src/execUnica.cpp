#include "../include/execUnica.hpp"
#include "../include/heuristica.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

std::string normalizarCaminho(std::string caminho) {
    const auto primeiro = caminho.find_first_not_of(" \t\r\n");
    if (primeiro == std::string::npos) {
        return {};
    }
    const auto ultimo = caminho.find_last_not_of(" \t\r\n");
    caminho = caminho.substr(primeiro, ultimo - primeiro + 1);

    if (caminho.size() >= 2 &&
        ((caminho.front() == '"' && caminho.back() == '"') ||
         (caminho.front() == '\'' && caminho.back() == '\''))) {
        caminho = caminho.substr(1, caminho.size() - 2);
    }

    return caminho;
}

bool lerInteiro(const std::string& entrada, int& valor) {
    std::istringstream stream(entrada);
    char caractereExtra;
    return (stream >> valor) && !(stream >> caractereExtra);
}

bool lerVelocidade(double& velocidade) {
    std::string entrada;
    while (true) {
        std::cout << "Velocidade do drone (0.5, 1 ou 2): ";
        if (!std::getline(std::cin, entrada)) {
            return false;
        }

        if (entrada == "0.5") {
            velocidade = 0.5;
            return true;
        }
        if (entrada == "1") {
            velocidade = 1.0;
            return true;
        }
        if (entrada == "2") {
            velocidade = 2.0;
            return true;
        }

        std::cout << "Velocidade invalida. Escolha 0.5, 1 ou 2.\n";
    }
}

bool lerOpcaoInteira(const std::string& pergunta,
                     const std::vector<int>& opcoes,
                     int& escolha) {
    std::string entrada;
    while (true) {
        std::cout << pergunta;
        if (!std::getline(std::cin, entrada)) {
            return false;
        }

        if (lerInteiro(entrada, escolha)) {
            for (int opcao : opcoes) {
                if (escolha == opcao) {
                    return true;
                }
            }
        }

        std::cout << "Opcao invalida. Tente novamente.\n";
    }
}

bool executarUnica() {
    std::string caminhoArquivo;
    do {
        std::cout << "Arquivo da instancia (ex.: clustered\\B-n34-k5-r8.mtspds "
                     "ou scattered/A-n36-k5-r12.mtspds): ";
        std::string entrada;
        if (!std::getline(std::cin, entrada)) {
            return false;
        }
        caminhoArquivo = normalizarCaminho(entrada);
        if (caminhoArquivo.empty()) {
            std::cout << "O caminho do arquivo nao pode ficar vazio.\n";
        }
    } while (caminhoArquivo.empty());

    double velocidadeDrone = 0.0;
    if (!lerVelocidade(velocidadeDrone)) {
        return false;
    }

    int dronesPorEstacao = 0;
    if (!lerOpcaoInteira(
            "Quantidade de drones por estacao (1, 2 ou 3): ",
            {1, 2, 3},
            dronesPorEstacao)) {
        return false;
    }

    Instancia instancia{};
    if (!lerInstancia(caminhoArquivo, instancia)) {
        return false;
    }

    try {
        const Solucao solucao =
            executarHeuristica(instancia, dronesPorEstacao, velocidadeDrone);
        const std::unordered_map<int, double> temposDrones =
            calcularTempoDrones(
                instancia,
                solucao.atendimentoDrones,
                dronesPorEstacao,
                velocidadeDrone);

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "\n=== Informacoes da instancia ===\n"
                  << "Arquivo: " << instancia.nomeArquivo << '\n'
                  << "Deposito: (" << instancia.deposito.posicao.x << ", "
                  << instancia.deposito.posicao.y << ")\n"
                  << "Clientes: " << instancia.clientes.size() << '\n'
                  << "Estacoes: " << instancia.estacoes.size() << '\n'
                  << "Caminhoes: " << instancia.deposito.qntCaminhoes << '\n'
                  << "Alcance das estacoes: " << instancia.raio << '\n'
                  << "Velocidade do drone: " << velocidadeDrone << '\n'
                  << "Drones por estacao: " << dronesPorEstacao << "\n\n";

        std::cout << "=== Clientes ao alcance de cada estacao ===\n";
        for (const EstacaoDrone& estacao : instancia.estacoes) {
            const std::vector<int> clientesAlcancaveis =
                clientesNoAlcance(estacao, instancia.clientes);

            std::cout << "Estacao " << estacao.idEstacao << ": ";
            if (clientesAlcancaveis.empty()) {
                std::cout << "nenhum cliente";
            } else {
                for (int i = 0; i < clientesAlcancaveis.size(); ++i) {
                    if (i > 0) {
                        std::cout << ", ";
                    }
                    std::cout << clientesAlcancaveis[i];
                }
            }
            std::cout << '\n';
        }

        std::cout << "\n=== Atribuicoes dos drones ===\n";
        if (solucao.atendimentoDrones.empty()) {
            std::cout << "Nenhum cliente atribuido a uma estacao.\n";
        } else {
            for (const auto& atribuicao : solucao.atendimentoDrones) {
                std::cout << "Estacao " << atribuicao.first
                          << " -> Cliente " << atribuicao.second << '\n';
            }
        }

        std::cout << "\n=== Tempo de atendimento por estacao ===\n";
        for (const EstacaoDrone& estacao : instancia.estacoes) {
            const auto tempo = temposDrones.find(estacao.idEstacao);
            const double tempoAtendimento =
                tempo == temposDrones.end() ? 0.0 : tempo->second;
            std::cout << "Estacao " << estacao.idEstacao << ": "
                      << tempoAtendimento << '\n';
        }

        std::cout << "\n=== Rotas dos caminhoes ===\n"
                  << "No 0 representa o deposito, nos positivos representam "
                     "clientes e nos negativos representam estacoes "
                     "(por exemplo, -1 e a estacao 1).\n";
        for (const RotaCaminhao& rota : solucao.rotasCaminhoes) {
            std::cout << "Caminhao " << rota.id << ": ";
            for (int i = 0; i < rota.sequenciaNos.size(); ++i) {
                if (i > 0) {
                    std::cout << " -> ";
                }
                std::cout << rota.sequenciaNos[i];
            }
            std::cout << " | distancia: " << rota.tempoViagem << '\n';
        }

        std::cout << "\nMakespan: " << solucao.makespan << '\n';
        
    } catch (const std::exception& erro) {
        std::cerr << "Erro ao executar a heuristica: " << erro.what() << '\n';
        return false;
    }

    return true;
}
