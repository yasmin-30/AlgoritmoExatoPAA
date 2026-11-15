#include "algExato.hpp"
#include "leitorArquivos.hpp"

// Processamento de opções de Caminhão 
void SolExata::processarOpcoesCaminhao(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan) {
    int clienteAtual = estado.clienteAtual;
    int totalRotas = (int)estado.solucaoParcial.rotasCaminhoes.size();
    int totalEstacoes = (int)instancia_.estacoes.size();

    // Opção 1: O caminhão atende o cliente diretamente na rota existente
    for (int i = 0; i < totalRotas; ++i) {
        processarOpcaoCaminhaoExistente(estado, melhorSolucao, melhorMakespan, clienteAtual, i);
    }

    // Opção 2: O caminhão ativa uma estação e atende um cliente
    for (int i = 0; i < totalEstacoes; ++i) {
        for (int j = 0; j < totalRotas; ++j) {
            processarOpcaoAtivarEstacao(estado, melhorSolucao, melhorMakespan, clienteAtual, i, j);
        }
    }

    // Opção 3: Abre uma nova rota para o caminhão
    if (estado.caminhoesEmUso < instancia_.deposito.qntCaminhoes) {
        processarOpcaoNovaRota(estado, melhorSolucao, melhorMakespan, clienteAtual);
    }
}

// O caminhão atende o cliente diretamente na rota existente
void SolExata::processarOpcaoCaminhaoExistente(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan, int clienteAtual, int i) {
    std::vector<bool> visitadosCaminhao = estado.visitados;
    visitadosCaminhao[clienteAtual] = true;

    estado.solucaoParcial.rotasCaminhoes[i].sequenciaNos.push_back(clienteAtual);
    double novoCustoRota = calcularCustoRotaCaminhao(estado.solucaoParcial.rotasCaminhoes[i]);
    double tempoAntigo = estado.solucaoParcial.rotasCaminhoes[i].tempoViagem;
    estado.solucaoParcial.rotasCaminhoes[i].tempoViagem = novoCustoRota;

    double makespanGlobalTemp = 0.0;
    for (const auto& rota : estado.solucaoParcial.rotasCaminhoes) {
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

    estado.solucaoParcial.rotasCaminhoes[i].sequenciaNos.pop_back();
    estado.solucaoParcial.rotasCaminhoes[i].tempoViagem = tempoAntigo;
}

// O caminhão ativa uma estação e atende um cliente
void SolExata::processarOpcaoAtivarEstacao(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan, int clienteAtual, int estacao, int rotaIdx) {
    std::vector<bool> visitadosCaminhao = estado.visitados;
    visitadosCaminhao[clienteAtual] = true;

    estado.solucaoParcial.rotasCaminhoes[rotaIdx].estacoesVisitadas.push_back(estacao);
    estado.solucaoParcial.rotasCaminhoes[rotaIdx].sequenciaNos.push_back(clienteAtual);

    double novoCustoRota = calcularCustoRotaCaminhao(estado.solucaoParcial.rotasCaminhoes[rotaIdx]);
    double tempoAntigo = estado.solucaoParcial.rotasCaminhoes[rotaIdx].tempoViagem;
    estado.solucaoParcial.rotasCaminhoes[rotaIdx].tempoViagem = novoCustoRota;

    std::vector<bool> novasEstacoesAtivadas = estado.estacoesAtivadas;
    novasEstacoesAtivadas[estacao] = true; 

    double makespanGlobalTemp = 0.0;
    for (const auto& rota : estado.solucaoParcial.rotasCaminhoes) {
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

    estado.solucaoParcial.rotasCaminhoes[rotaIdx].estacoesVisitadas.pop_back();
    estado.solucaoParcial.rotasCaminhoes[rotaIdx].sequenciaNos.pop_back();
    estado.solucaoParcial.rotasCaminhoes[rotaIdx].tempoViagem = tempoAntigo;
}

// Abre uma nova rota para o caminhão
void SolExata::processarOpcaoNovaRota(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan, int clienteAtual) {
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

// Processamento de opções de Drone
void SolExata::processarOpcoesDrone(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan) {
    int clienteAtual = estado.clienteAtual;
    double velocidadeCaminhao = 1.0; 
    int totalEstacoes = (int)instancia_.estacoes.size();

    for (int i = 0; i < totalEstacoes; ++i) {
        double distEstacaoCliente = calcularDistancia(instancia_.estacoes[i].posicao, instancia_.clientes[clienteAtual].posicao);
        
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