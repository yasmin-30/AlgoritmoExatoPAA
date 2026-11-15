# Problema do Caixeiro Viajante Múltiplo com Estações de Drone (mTSP-DS)

Este repositório contém uma implementação modular e eficiente em C++ desenvolvida para resolver o Problema do Caixeiro Viajante Múltiplo com Estações de Drones (mTSP-DS). O objetivo principal é a otimização e minimização do makespan em diferentes configurações de instâncias por meio de um algoritmo exato.

## 📁 Estrutura do Repositório

```
AlgoritmoExatoPAA/
│
├── include/                   # Declarações e cabeçalhos (.hpp)
│   ├── algExato.hpp           # Declaração das funções e estruturas usadas no algoritmo
│   ├── entidades.hpp          # Definição das estruturas do problema (Vértices, Drones, Caixeiros, etc.)
│   ├── execUnica.hpp          # Interface para execução de uma instância individual
│   ├── heuristica.hpp         # Declaração da Heurística Construtiva Gulosa
│   ├── leitorArquivos.hpp     # Declaração da leitura dos arquivos .mtspds
│   └── salvarExecExata.hpp    # Declaração da função para salvar os relatórios .csv
│
├── instancias/                # Conjunto de dados e instâncias de teste
│   ├── clustered/             # Instâncias com topologia espacial agrupada
│   └── scattered/             # Instâncias com topologia espacial dispersa
│
├── resultados/                # Relatórios e métricas de saída gerados
│
├── src/                       # Implementação das lógicas em C++ (.cpp)
│   ├── algExato.cpp           # Lógica do algoritmo exato (branch and bound)
│   ├── execUnica.cpp          # Lógica para teste e depuração individual
│   ├── heuristica.cpp         # Lógica da Heurística Construtiva Gulosa
│   ├── leitorArquivos.cpp     # Leitura dos dados das instâncias
│   └── main.cpp               # Ponto de entrada da aplicação
│   ├── salvarExecExata.cpp    # Salva os resultados da execução do algoritmo
│
├── .gitignore                  
```


## ▶️ Como Compilar e Executar

### Requisitos Mínimos
Compilador g++ com suporte ao padrão C++11 ou superior.

### Compilando
Na pasta raiz do repositório, execute:

```
g++ -std=c++17 -O3 src/main.cpp src/leitorArquivos.cpp src/heuristica.cpp src/algExato.cpp src/execUnica.cpp src/salvarExecExata.cpp -I include -o mTSP_DS
```

### Executando
O programa deve ser executado a partir da pasta raiz do repositório, pois usa os caminhos relativos `instancias/` e `resultados/`.

- **Linux, macOS:** `./mTSP_DS`
- **Windows:** `./mTSP_DS.exe                                                                      `

Ao iniciar, o programa exibe um menu que pede as seguintes informações:

- o arquivo da instância, informado com a pasta e o nome, por exemplo `clustered\B-n34-k5-r8.mtspds` ou `scattered/A-n36-k5-r12.mtspds` (as duas barras são aceitas em qualquer sistema);
- a velocidade dos drones: 0.5, 1 ou 2;
- a quantidade de drones por estação: 1, 2 ou 3.

Em seguida, é exibido o makespan para a instância dada. Além disso, os dados da instância, as atribuições dos drones, as rotas dos caminhões e o *makespan* são salvos no arquivo nome_do_arquivo_detalhes.txt.
