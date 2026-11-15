#ifndef SALVAR_EXEC_EXATA_HPP
#define SALVAR_EXEC_EXATA_HPP

#include "entidades.hpp"
#include "leitorArquivos.hpp"
#include <string>

// Declaração da função que gera o relatório por instância
bool salvarDetalhesExecUnica(const std::string& caminhoArquivoInstancia,
                               const Instancia& instancia,
                               const Solucao& solucao,
                               double velocidadeDrone,
                               int dronesPorEstacao,
                               const std::string& pastaSaida = "resultados");

#endif