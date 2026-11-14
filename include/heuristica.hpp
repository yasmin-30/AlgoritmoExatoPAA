#ifndef HEURISTICA_HPP
#define HEURISTICA_HPP
#include <string>
#include <vector>
#include "entidades.hpp"
#include "leitorArquivos.hpp"
#include <utility>
#include <unordered_map>

double calcularDistancia(const Coordenada& a, const Coordenada& b);

std::vector<int> clientesNoAlcance(
    const EstacaoDrone& estacao,
    const std::vector<Cliente>& clientes
);

std::vector<int> selecionarEstacoes(const Instancia& instancia);

// Cada atribuicao tem o formato <idEstacao, idCliente>.
std::vector<std::pair<int, int>> atribuirClientes(
    const Instancia& instancia,
    const std::vector<int>& estacoesSelecionadas
);

std::vector<RotaCaminhao> construirRotas(
    const Instancia& instancia,
    const std::vector<int>& estacoesSelecionadas,
    const std::vector<std::pair<int, int>>& atribuicoes
);

std::unordered_map<int, double> calcularTempoDrones(
    const Instancia& instancia,
    const std::vector<std::pair<int, int>>& atribuicoes,
    int dronesPorEstacao,
    double alpha
);

Solucao executarHeuristica(
    const Instancia& instancia,
    int dronesPorEstacao,
    double alpha
);
#endif