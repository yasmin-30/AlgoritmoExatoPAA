#ifndef ALGEXATO_HPP
#define ALGEXATO_HPP

#include "entidades.hpp"
#include "leitorArquivos.hpp"
#include <cmath>

class SolExata {

    public:
        // Construtor
        SolExata(const Instancia& instancia, double fatorVelocidadeDrone = 1.0, int dronesPorEstacao = 2);

        // Executa o Branch and Bound para encontrar a solucao otima
        bool resolver(Solucao& melhorSolucao, double upperBoundInicial = INFINITY);

    private:
        Instancia instancia_;
        double fatorVelocidadeDrone_;
        int dronesPorEstacao_;

        struct EstadoBusca {
            int clienteAtual = 0;
            std::vector<bool> visitados;
            int caminhoesEmUso = 0;
            std::vector<bool> estacoesAtivadas;
            std::vector<int> dronesEmUsoPorEstacao;
            double makespanAtual = 0.0;
            Solucao solucaoParcial;
        };
                
        void branchAndBound(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan);
        
        // Funcoes auxiliares do branch and bound
        void processarOpcoesCaminhao(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan);
        void processarOpcoesDrone(EstadoBusca& estado, Solucao& melhorSolucao, double& melhorMakespan);
        
        double calcularDistancia(Coordenada c1, Coordenada c2) const;
        double calcularCustoRotaCaminhao(const RotaCaminhao& rota) const;
};

#endif