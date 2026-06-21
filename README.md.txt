# Otimização do Algoritmo de Dijkstra para Redes de Transporte Urbano (TDSP)

Repositório contendo o código-fonte do simulador de roteamento dinâmico desenvolvido para o artigo **"Otimização do Algoritmo de Dijkstra Aplicado a Redes de Transporte Urbano com Janelas de Espera Dinâmicas"**. 

Este projeto propõe uma variante otimizada do Algoritmo de Dijkstra para solucionar o Problema de Caminho Mínimo Dependente do Tempo (TDSP), focado em redes urbanas com restrições temporais intermitentes (semáforos).

## 🚀 Principais Características

* **Aritmética Modular:** Cálculo de atrasos semafóricos em tempo real (complexidade `O(1)`) integrados na fase de relaxação das arestas.
* **Otimização de Cache (Forward Star):** Estrutura orientada a dados utilizando vetores contíguos para evitar *cache misses* no processador, ideal para malhas viárias esparsas.
* **Lazy Insertion:** Implementação de fila de prioridade (*Binary Min-Heap*) que tolera duplicatas, evitando o massivo *overhead* de atualização de chaves em rotas extensas.
* **Alta Performance:** Escrito em C++17, capaz de rotear uma grade sintética com 100.000 nós e 398.000 arestas em menos de 30 milissegundos.

## 🛠️ Como Compilar e Executar

O simulador é nativo em C++ e requer o compilador GCC configurado na máquina.

**1. Compilando o código**
Para garantir a performance demonstrada no artigo, é imprescindível compilar com a flag de otimização `-O3`:

```bash
g++ -O3 -std=c++17 -o simulador simulador_v3.cpp