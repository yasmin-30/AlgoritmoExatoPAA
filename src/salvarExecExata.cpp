#include "../include/leitorArquivos.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <vector>

// FAZER PARA LINUX TAMBEM ***************************
bool salvarDetalhesExecUnica(const std::string& caminhoArquivoInstancia,
                               const Instancia& instancia,
                               const Solucao& solucao,
                               double velocidadeDrone,
                               int dronesPorEstacao,
                               const std::string& pastaSaida) {
    
    // Extrai só o nome do arquivo
    int posUltimaBarra = caminhoArquivoInstancia.find_last_of("/\\");
    std::string nomeArquivo = (posUltimaBarra == std::string::npos) ? 
                               caminhoArquivoInstancia : caminhoArquivoInstancia.substr(posUltimaBarra + 1);


    int posExt = nomeArquivo.find_last_of('.');
    std::string nomeBase = (posExt == std::string::npos) ? nomeArquivo : nomeArquivo.substr(0, posExt);

    // Monta o caminho final do arquivo
    std::string caminhoFinal = pastaSaida + "/" + nomeBase + "_detalhes.txt";

    std::ofstream arquivo(caminhoFinal);
    if (!arquivo.is_open()) {
        std::cerr << "[Erro] Nao foi possivel criar o arquivo em: " << caminhoFinal 
                  << " (Verifique se a pasta '" << pastaSaida << "' existe!)\n";
        return false;
    }

    arquivo << std::fixed << std::setprecision(2);

    // Informacoes da instancia
    arquivo << "=== Informacoes da instancia ===\n";
    arquivo << "Arquivo: " << caminhoArquivoInstancia << "\n";
    arquivo << "Deposito: (" << instancia.deposito.posicao.x << ", " << instancia.deposito.posicao.y << ")\n";
    arquivo << "Clientes: " << instancia.clientes.size() << "\n";
    arquivo << "Estacoes: " << instancia.estacoes.size() << "\n";
    arquivo << "Caminhoes: " << instancia.deposito.qntCaminhoes << "\n";
    arquivo << "Alcance das estacoes: " << instancia.raio << "\n";
    arquivo << "Velocidade do drone: " << velocidadeDrone << "\n";
    arquivo << "Drones por estacao: " << dronesPorEstacao << "\n\n";

    // Clientes ao alcance de cada estacao
    arquivo << "=== Clientes ao alcance de cada estacao ===\n";
    for (int i = 0; i < instancia.estacoes.size(); ++i) {
        arquivo << "Estacao " << (i + 1) << ":";
        bool primeiro = true;
        for (const auto& cli : instancia.clientes) {
            double dx = instancia.estacoes[i].posicao.x - cli.posicao.x;
            double dy = instancia.estacoes[i].posicao.y - cli.posicao.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist <= instancia.raio) {
                if (!primeiro) arquivo << ",";
                arquivo << " " << cli.idCliente;
                primeiro = false;
            }
        }
        arquivo << "\n";
    }
    arquivo << "\n";

    // Atribuicoes dos drones
    arquivo << "=== Atribuicoes dos drones ===\n";
    for (const auto& droneAtend : solucao.atendimentoDrones) {
        arquivo << "Estacao " << (droneAtend.first + 1) << " -> Cliente " << droneAtend.second << "\n";
    }
    arquivo << "\n";

    //Tempo de atendimento por estacao
    arquivo << "=== Tempo de atendimento por estacao ===\n";
    std::vector<double> tempoEstacoes(instancia.estacoes.size(), 0.0);
    for (const auto& droneAtend : solucao.atendimentoDrones) {
        int idEstacao = droneAtend.first;
        int idCliente = droneAtend.second;
        
        auto itCli = std::find_if(instancia.clientes.begin(), instancia.clientes.end(), [idCliente](const Cliente& c){
            return c.idCliente == idCliente;
        });

        if (itCli != instancia.clientes.end()) {
            double dx = instancia.estacoes[idEstacao].posicao.x - itCli->posicao.x;
            double dy = instancia.estacoes[idEstacao].posicao.y - itCli->posicao.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            double tempoViagem = (2.0 * dist) / (1.0 * velocidadeDrone);
            tempoEstacoes[idEstacao] += tempoViagem;
        }
    }

    for (int i = 0; i < instancia.estacoes.size(); ++i) {
        arquivo << "Estacao " << (i + 1) << ": " << tempoEstacoes[i] << "\n";
    }
    arquivo << "\n";

    // Rotas dos caminhoes 
    arquivo << "=== Rotas dos caminhoes ===\n";
    arquivo << "No 0 representa o deposito, nos positivos representam clientes e nos negativos representam estacoes (por exemplo, -1 e a estacao 1).\n";
    for (const auto& rota : solucao.rotasCaminhoes) {
        arquivo << "Caminhao " << rota.id << ": 0";
        
        // Percorre todos os nós da rota
        for (int no : rota.sequenciaNos) {
            arquivo << " -> " << no;
        }
        
        arquivo << " -> 0 | distancia: " << rota.tempoViagem << "\n";
    }
    arquivo << "\n";

    // Makespan final
    arquivo << std::fixed << std::setprecision(2);
    arquivo << "Makespan: " << solucao.makespan << "\n\n";
    arquivo << "[Info] Relatorio salvo com sucesso em '" << caminhoFinal << "'\n";

    arquivo.close();
    std::cout << "[Info] Relatorio salvo com sucesso em '" << caminhoFinal << "'\n";
    return true;
}