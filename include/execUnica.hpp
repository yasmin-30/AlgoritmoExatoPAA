#ifndef EXEC_UNICA_HPP
#define EXEC_UNICA_HPP

#include <string>
#include <vector>

bool lerOpcaoInteira(const std::string& pergunta, const std::vector<int>& opcoes, int& escolha);
bool lerVelocidade(double& velocidade); // <--- ADICIONE ESTA LINHA
bool executarUnica();

#endif