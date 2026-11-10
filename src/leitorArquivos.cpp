#include "../include/leitorArquivos.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

namespace {

// Troca "\\" por "/" (o Windows tambem aceita "/", entao o resultado serve
// para os dois sistemas).
std::string normalizarSeparadores(std::string caminho) {
    std::replace(caminho.begin(), caminho.end(), '\\', '/');
    return caminho;
}

bool arquivoLegivel(const std::string& caminho) {
    std::ifstream arquivo(caminho.c_str());
    // peek() falha em diretorios (o ifstream abre diretorios no Linux)
    return arquivo.is_open() &&
           arquivo.peek() != std::ifstream::traits_type::eof();
}

}  // namespace

bool resolverCaminhoInstancia(const std::string& entrada, std::string& caminhoResolvido) {
    const std::string caminho = normalizarSeparadores(entrada);

    const std::vector<std::string> candidatos = {
        caminho,                          // como digitado (absoluto ou relativo)
        "instancias/" + caminho,          // ex.: clustered/X.mtspds
        "../instancias/" + caminho        // executando a partir de uma subpasta
    };

    for (const std::string& candidato : candidatos) {
        if (arquivoLegivel(candidato)) {
            caminhoResolvido = candidato;
            return true;
        }
    }

    std::cerr << "Erro ao abrir o arquivo: " << entrada << "\nCaminhos tentados:\n";
    for (const std::string& candidato : candidatos) {
        std::cerr << "  - " << candidato << '\n';
    }
    return false;
}

bool extrairParametrosDoNome(const std::string& caminhoArquivo, int& qntClientes, int& qntCaminhoes, int& raio) {
    const std::size_t separador = caminhoArquivo.find_last_of("/\\");
    const std::string nomeArquivo =
        separador == std::string::npos
            ? caminhoArquivo
            : caminhoArquivo.substr(separador + 1);
    int extraidos = std::sscanf(nomeArquivo.c_str(), "%*[^-]-n%d-k%d-r%d.mtspds", &qntClientes, &qntCaminhoes, &raio);
    return (extraidos == 3);
}

bool extrairConteudo(std::ifstream& arquivo, Instancia& instancia, int qntClientes, int raio) {
    int coordDeposito = 0;
    int idCliente = 1;
    int idEstacao = 1;

    std::string linha;
    char virgula;

    while (std::getline(arquivo, linha)) {
        if (linha.empty()) continue;
        
        const unsigned char primeiroCarac = static_cast<unsigned char>(linha[0]);
        if (!std::isdigit(primeiroCarac)) continue;

        std::stringstream ss(linha);

        if (coordDeposito == 0) {
            ss >> instancia.deposito.posicao.x >> virgula >> instancia.deposito.posicao.y;
            coordDeposito = 1;
        } 
        else if (idCliente <= qntClientes - 1) {
            Cliente cliente;
            cliente.entregue = false;
            cliente.idCliente = idCliente++;
            
            ss >> cliente.posicao.x >> virgula >> cliente.posicao.y;
            instancia.clientes.push_back(cliente);
        } 
        else {
            EstacaoDrone estacao;
            estacao.ativado = false;
            estacao.alcance = raio;
            estacao.idEstacao = idEstacao++;
            
            ss >> estacao.posicao.x >> virgula >> estacao.posicao.y;
            instancia.estacoes.push_back(estacao);
        }
    }

    return true;
}

bool lerInstancia(const std::string& caminhoArquivo, Instancia& instancia) {
    instancia.nomeArquivo = caminhoArquivo;
    instancia.clientes.clear();
    instancia.estacoes.clear();

    instancia.clientes.clear();
    instancia.estacoes.clear();

    // 1. Resolve o caminho (aceita "clustered\\X.mtspds", "scattered/X.mtspds",
    //    caminhos completos, etc.) e abre o arquivo
    std::string caminhoReal;
    if (!resolverCaminhoInstancia(caminhoArquivo, caminhoReal)) {
        return false;
    }
    instancia.nomeArquivo = caminhoReal;

    std::ifstream arquivo(caminhoReal.c_str());
    if (!arquivo.is_open()) {
        std::cerr << "Erro ao abrir o arquivo: " << caminhoReal << std::endl;
        return false;
    }

    // 2. Extrai as informações do nome do arquivo
    int qntClientes = 0, qntCaminhoes = 0, raio = 0;
    if (!extrairParametrosDoNome(caminhoReal, qntClientes, qntCaminhoes, raio)) {
        std::cerr << "\nErro: O padrão ou a extensão do nome do arquivo não coincidem!" << std::endl;
        arquivo.close();
        return false;
    }

    instancia.deposito.qntCaminhoes = qntCaminhoes;
    instancia.raio = raio;
    instancia.numClientes = qntClientes - 1;

    // 3. Processa o conteúdo do arquivo
    extrairConteudo(arquivo, instancia, qntClientes, raio);

    // 4. Fecha o arquivo
    arquivo.close();

    // 5. Validação final
    if (instancia.clientes.empty() || instancia.estacoes.empty()) {
        std::cerr << "Erro: Não foi possível ler os dados da instância " << caminhoReal << std::endl;
        return false;
    }

    return true;
}