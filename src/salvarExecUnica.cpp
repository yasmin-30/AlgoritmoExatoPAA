#include "../include/salvarExecUnica.hpp"
#include "../include/heuristica.hpp"

#include <fstream>
#include <iostream>
#include <iomanip>

void salvarResultadoCSV(const std::string& nomeArquivoCSV, const Instancia& instancia, double velocidadeDrone, 
                        int dronesPorEstacao, const Solucao& solucao, const std::unordered_map<int, double>& temposDrones) {
    
    // Abre o arquivo ou cria um novo
    std::ofstream arquivo(nomeArquivoCSV, std::ios::app);
    if (!arquivo.is_open()) {
        std::cerr << "Erro ao abrir o arquivo para escrita: " << nomeArquivoCSV << '\n';
        return;
    }

    arquivo << std::fixed << std::setprecision(2);

    // Informações da instância
    arquivo << "=== Informacoes da instancia ===\n";
    arquivo << "Arquivo: " << instancia.nomeArquivo << "\n";
    arquivo << "Deposito: (" << instancia.deposito.posicao.x << ", " << instancia.deposito.posicao.y << ")\n";
    arquivo << "Clientes: " << instancia.clientes.size() << "\n";
    arquivo << "Estacoes: " << instancia.estacoes.size() << "\n";
    arquivo << "Caminhoes: " << instancia.deposito.qntCaminhoes << "\n";
    arquivo << "Alcance das estacoes: " << instancia.raio << "\n";
    arquivo << "Velocidade do drone: " << velocidadeDrone << "\n";
    arquivo << "Drones por estacao: " << dronesPorEstacao << "\n\n";

    // Clientes ao alcance de cada estação
    arquivo << "=== Clientes ao alcance de cada estacao ===\n";
    for (const EstacaoDrone& estacao : instancia.estacoes) {
        const std::vector<int> clientesAlcancaveis = clientesNoAlcance(estacao, instancia.clientes);

        arquivo << "Estacao " << estacao.idEstacao << ": ";
        if (clientesAlcancaveis.empty()) {
            arquivo << "nenhum cliente";
        } else {
            for (std::size_t i = 0; i < clientesAlcancaveis.size(); ++i) {
                
                if (i > 0) {
                    arquivo << ", ";
                }
                arquivo << clientesAlcancaveis[i];
            }
        }
        arquivo << "\n";
    }
    arquivo << "\n";

    // Atribuições dos drones
    arquivo << "=== Atribuicoes dos drones ===\n";
    if (solucao.atendimentoDrones.empty()) {
        arquivo << "Nenhum cliente atribuido a uma estacao.\n";
    } else {
        for (const auto& atribuicao : solucao.atendimentoDrones) {
            arquivo << "Estacao " << atribuicao.first << " -> Cliente " << atribuicao.second << "\n";
        }
    }
    arquivo << "\n";

    // Tempo de atendimento por estação
    arquivo << "=== Tempo de atendimento por estacao ===\n";
    for (const EstacaoDrone& estacao : instancia.estacoes) {
        const auto it = temposDrones.find(estacao.idEstacao);
        double tempoAtendimento = (it == temposDrones.end()) ? 0.0 : it->second;
        arquivo << "Estacao " << estacao.idEstacao << ": " << tempoAtendimento << "\n";
    }
    arquivo << "\n";

    // Rotas dos caminhões
    arquivo << "=== Rotas dos caminhoes ===\n";
    arquivo << "No 0 representa o deposito, nos positivos representam clientes e nos negativos representam estacoes (por exemplo, -1 e a estacao 1).\n";
    for (const RotaCaminhao& rota : solucao.rotasCaminhoes) {
        arquivo << "Caminhao " << rota.id << ": ";
        for (std::size_t i = 0; i < rota.sequenciaNos.size(); ++i) {
            
            if (i > 0) {
                arquivo << " -> ";
            }
            arquivo << rota.sequenciaNos[i];
        }
        arquivo << " | distancia: " << rota.tempoViagem << "\n";
    }

    // Makespan 
    arquivo << "\nMakespan: " << solucao.makespan << "\n";
    arquivo << "==================================================\n\n";

    arquivo.close();
}