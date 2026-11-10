#ifndef ENTIDADES_HPP
#define ENTIDADES_HPP
#include <string>
#include <vector>

struct Coordenada {
    float x;
    float y;
};

struct Cliente {
    int idCliente;
    Coordenada posicao;
    bool entregue;
};

struct EstacaoDrone {
    int idEstacao;
    Coordenada posicao;
    int alcance;
    bool ativado;
};

struct Deposito {
    Coordenada posicao;
    int qntCaminhoes;
};

struct RotaCaminhao {
    int id;
    std::vector<int> sequenciaNos;
    double tempoViagem;
};

// Estrutura que armazena a solução completa para o problema
struct Solucao {
    std::vector<RotaCaminhao> rotasCaminhoes;
    std::vector<std::pair<int, int>> atendimentoDrones; // <idEstacao, idCliente>
    double makespan;
};

#endif