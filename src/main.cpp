#include <iostream>
#include <string>
#include "../include/entidades.hpp"
#include "../include/leitorArquivos.hpp"
#include "../include/heuristica/heuristica.hpp"
#include "../include/algExato.hpp"
#include "../include/heuristica/execUnica.hpp"
#include "../include/salvarExecExata.hpp"
#include <algorithm>

int main() {
    std::string caminhoArquivo;
    double velocidadeDrone = 1.0;
    int dronesPorEstacao = 2;

    std::cout << "========================================\n";
    std::cout << "    mTSP-DS: Algoritmo Exato (Branch and Bound)\n";
    std::cout << "========================================\n";
    
    do {
        std::cout << "Arquivo da instancia (ex.: instancias/clustered/B-n31-k5-r8.mtspds): ";
        std::string entrada;
        if (!std::getline(std::cin, entrada)) {
            return 1;
        }
        
        caminhoArquivo = entrada;
        if (caminhoArquivo.empty()) {
            std::cout << "O caminho nao pode ficar vazio.\n";
        }
    } while (caminhoArquivo.empty());

    if (!lerVelocidade(velocidadeDrone)) { 
        velocidadeDrone = 1.0;
    }

    if (!lerOpcaoInteira("Quantidade de drones por estacao (1, 2 ou 3): ", {1, 2, 3}, dronesPorEstacao)) {
        dronesPorEstacao = 2;
    }

    // Carrega a instância
    Instancia instancia{};
    if (!lerInstancia(caminhoArquivo, instancia)) {
        std::cerr << "ERRO: Falha ao carregar ou ler o arquivo da instancia.\n";
        return 1;
    }

    // Executa a Heurística para obter o upper bound
    Solucao solucaoHeuristica = executarHeuristica(instancia, dronesPorEstacao, velocidadeDrone);
    double upperBoundInicial = solucaoHeuristica.makespan;
    std::cout << "Makespan inicial obtido (Upper Bound): " << upperBoundInicial << "\n";

    SolExata solver(instancia, velocidadeDrone, dronesPorEstacao);
    Solucao melhorSolucao = solucaoHeuristica;

    bool encontrouSolucao = solver.resolver(melhorSolucao, upperBoundInicial);

    // Normaliza a solução heuristica da base 1 para base 0
    if (encontrouSolucao) {
        for (auto& a : melhorSolucao.atendimentoDrones) a.second += 1;
    } else {
        for (auto& a : melhorSolucao.atendimentoDrones) a.first -= 1;
        
        for (auto& rota : melhorSolucao.rotasCaminhoes) {
            rota.sequenciaNos.erase(
                std::remove(rota.sequenciaNos.begin(), rota.sequenciaNos.end(), 0),
                rota.sequenciaNos.end());
        }
    }

    if (encontrouSolucao || melhorSolucao.makespan > 0) {
        std::cout << "\n========================================\n";
        std::cout << "         RESULTADO OTIMO FINAL\n";
        std::cout << "========================================\n";
        std::cout << "Makespan Otimo: " << melhorSolucao.makespan << "\n";
        std::cout << "========================================\n";

        // Salva o relatorio da instancia
        salvarDetalhesExecUnica(caminhoArquivo, instancia, melhorSolucao, velocidadeDrone, dronesPorEstacao, "resultados");

    }

    return 0;
}