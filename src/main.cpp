#include <iostream>
#include <string>
#include <cmath>
#include "../include/entidades.hpp"
#include "../include/leitorArquivos.hpp"
#include "../include/heuristica.hpp"
#include "../include/algExato.hpp"
#include "../include/execUnica.hpp"
#include "../include/salvarExecUnica.hpp"

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

    // Leitura dos parâmetros operacionais
    if (!lerVelocidade(velocidadeDrone)) { 
        velocidadeDrone = 1.0;
    }

    if (!lerOpcaoInteira("Quantidade de drones por estacao (1, 2 ou 3): ", {1, 2, 3}, dronesPorEstacao)) {
        dronesPorEstacao = 2;
    }

    std::cout << "----------------------------------------\n";
    std::cout << "[Main] Carregando instancia de: " << caminhoArquivo << "\n";

    // Carrega a instância
    Instancia instancia{};
    if (!lerInstancia(caminhoArquivo, instancia)) {
        std::cerr << "[Erro] Falha ao carregar ou ler o arquivo da instancia.\n";
        return 1;
    }

    // Executa a Heurística para obter o upper bound
    std::cout << "[Main] Executando a Heuristica para gerar o Upper Bound inicial...\n";
    Solucao solucaoHeuristica = executarHeuristica(instancia, dronesPorEstacao, velocidadeDrone);
    
    double upperBoundInicial = solucaoHeuristica.makespan;
    std::cout << "[Main] Heuristica concluida com sucesso!\n";
    std::cout << "[Main] Makespan inicial obtido (Upper Bound): " << upperBoundInicial << "\n";

    // Configura o algoritmo exato
    SolExata solver(instancia, velocidadeDrone, dronesPorEstacao);

    Solucao melhorSolucao = solucaoHeuristica; 

    std::cout << "[Main] Iniciando o Branch and Bound...\n";

    // Executa o método resolver passando o Upper Bound da heurística
    bool encontrouSolucao = solver.resolver(melhorSolucao, upperBoundInicial);

    // Exibe os resultados definitivos
    if (encontrouSolucao) {
        std::cout << "\n========================================\n";
        std::cout << "  RESULTADO OTIMO FINAL (B&B)\n";
        std::cout << "========================================\n";
        std::cout << "Makespan Otimo: " << melhorSolucao.makespan << "\n";
        std::cout << "Total de rotas de caminhao ativas: " << melhorSolucao.rotasCaminhoes.size() << "\n";
        
        for (const auto& rota : melhorSolucao.rotasCaminhoes) {
            std::cout << "  - Caminhao " << rota.id << " | Tempo/Distancia: " << rota.tempoViagem << "\n";
        }
        std::cout << "========================================\n";

    } else {
        std::cout << "\n[Main] O solver exato encerrou sem encontrar melhora (mantendo o resultado da heuristica).\n";
    }

    return 0;
}