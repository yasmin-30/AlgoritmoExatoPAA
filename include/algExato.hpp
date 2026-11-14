#ifndef ALGEXATO_HPP
#define ALGEXATO_HPP

#include "entidades.hpp"
#include "leitorArquivos.hpp"
#include <cmath>

class SolExata {

    public:
        // Construtor
        SolExata(const Instancia& instancia, double fatorVelocidadeDrone = 1.0, int dronesPorEstacao);

        // Executa o Branch and Bound para encontrar a solucao otima
        bool resolver(Solucao& melhorSolucao, double upperBoundInicial = INFINITY);

    private:
        Instancia instancia_;
        double fatorVelocidadeDrone_;
        int dronesPorEstacao_;

        struct EstadoBusca {
            int clienteAtual; 
            const std::vector<bool> visitados; 
            int caminhoesEmUso; 
            std::vector<bool> estacoesAtivadas;
            const std::vector<int> dronesEmUsoPorEstacao;
            double makespanAtual;
            Solucao solucaoParcial;

            EstadoBusca() = default;
        };
                
        void branchAndBound(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan);
        
        // Funcoes auxiliares do branch and bound
        void processarOpcoesCaminhao(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan);
        void processarOpcoesDrone(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan);
        
        double calcularDistancia(Coordenada c1, Coordenada c2) const;
        double calcularCustoRotaCaminhao(const RotaCaminhao& rota) const;
};

#endif