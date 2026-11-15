#ifndef ALGEXATO_HPP
#define ALGEXATO_HPP

#include "entidades.hpp"
#include "leitorArquivos.hpp"
#include <chrono>

class SolExata {

    public:
        // Construtor
        SolExata(const Instancia& instancia, double fatorVelocidadeDrone = 1.0, int dronesPorEstacao = 2);

        // Limite de tempo de 10 min
        bool resolver(Solucao& melhorSolucao, double upperBoundInicial, double limiteTempoSegundos = 600.0);

    private:
        Instancia instancia_;
        double fatorVelocidadeDrone_;
        int dronesPorEstacao_;

        // Variáveis para controle de tempo e timeout
        std::chrono::steady_clock::time_point tempoInicio_;
        double limiteTempoSegundos_;
        bool timeoutAtingido_;
        int contadorNos_;

        struct EstadoBusca {
            int clienteAtual = 0;
            std::vector<bool> visitados;
            int caminhoesEmUso = 0;
            std::vector<bool> estacoesAtivadas;
            std::vector<std::vector<double>> cargaDrones;
            double makespanAtual = 0.0;
            Solucao solucaoParcial;
        };
                
        void branchAndBound(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan);
        
        // Funcoes auxiliares do branch and bound
        void processarOpcoesCaminhao(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan);
        void processarOpcoesDrone(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan);
        
        double calcularDistancia(Coordenada c1, Coordenada c2) const;

        // Funções Caminhão
        double calcularCustoRotaCaminhao(const RotaCaminhao& rota) const;
        void processarOpcaoCaminhaoExistente(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan, int clienteAtual, int i);
        void processarOpcaoAtivarEstacao(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan, int clienteAtual, int estacao, int rotaIdx);
        void processarOpcaoNovaRota(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan, int clienteAtual);
        double calcularMakespanParcial(const Solucao& sol, const std::vector<std::vector<double>>& carga) const;

        // Métodos do Lower Bound
        void precalcularCustosMinimos();
        double calcularLowerBound(const EstadoBusca& estado) const;
        std::vector<double> custoMinimoCliente_;
};

#endif