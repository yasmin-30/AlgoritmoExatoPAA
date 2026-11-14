#ifndef SALVAREXECUNICA_HPP
#define SALVAREXECUNICA_HPP

#include <string>
#include <unordered_map>
#include "entidades.hpp"
#include "leitorArquivos.hpp"


void salvarResultadoCSV(const std::string& nomeArquivoCSV, const Instancia& instancia, double velocidadeDrone, 
                        int dronesPorEstacao, const Solucao& solucao, const std::unordered_map<int, double>& temposDrones);

#endif 