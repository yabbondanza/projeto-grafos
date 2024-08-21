# Análise e Manipulação de Grafos

## Descrição

Este projeto fornece uma implementação em Python para análise e manipulação de grafos. Inclui funcionalidades para visualização, verificação de propriedades e algoritmos clássicos de grafos.

## Funcionalidades

### Visualização de Grafos

- **Função:** `visualizar_grafo(grafo, tipo_grafo)`
- **Descrição:** Cria e exibe um grafo usando `networkx` e `matplotlib`.
- **Parâmetros:**
  - `grafo`: Dicionário representando o grafo.
  - `tipo_grafo`: Tipo do grafo, 'direcionado' ou 'nao_direcionado'.

### Leitura do Grafo

- **Função:** `ler_grafo()`
- **Descrição:** Lê a entrada padrão para montar o grafo.
- **Retorno:** Dicionário representando o grafo, o tipo de grafo e o número de vértices.

### Verificações de Propriedades

- **Conectividade:** `conexo(grafo, num_vertices)`
- **Bipartição:** `bipartido(grafo)`
- **Eulerianidade:** `euleriano(grafo, num_vertices)`
- **Ciclos:**
  - Não direcionado: `possui_ciclo_nao_direcionado(grafo)`
  - Direcionado: `possui_ciclo_direcionado(grafo, num_vertices)`
- **Componentes Conexas:** `num_componentes_conexas(grafo)`
- **Componentes Fortemente Conexas:** `componentes_fortemente_conexas(grafo)`
- **Vertices de Articulação:** `vertices_de_articulacao(grafo)`
- **Arestas Ponte:** `arestas_ponte(grafo)`

### Algoritmos de Grafos

- **DFS e BFS:**
  - **DFS:** `dfs_tree(grafo, origem=0)`
  - **BFS:** `bfs(grafo, origem=0)`
- **Árvore Geradora Mínima (Kruskal):** `arvore_geradora_minima(grafo, num_vertices)`
- **Ordenação Topológica:** `ordenacao_topologica(grafo)`
- **Caminho Mínimo (Dijkstra):** `caminho_minimo(grafo, num_vertices)`
- **Fluxo Máximo (Edmonds-Karp):** `edmonds_karp(grafo, origem, destino, num_vertices)`
- **Fecho Transitivo:** `fecho_transitivo(grafo, origem=0)`

## Instalação e Dependências

### Instalação

Para instalar o projeto, siga estes passos:

1. **Clone o repositório:**

    ```bash
    git clone https://github.com/yabbondanza/projeto-grafos.git
    ```

2. **Navegue até o diretório do projeto:**

    ```bash
    cd projeto-grafos
    ```

### Dependências

O projeto utiliza as seguintes bibliotecas:

- **`networkx`**: Para manipulação e análise de grafos.
- **`matplotlib`**: Para visualização de grafos.

Para instalar essas dependências manualmente, você pode usar:

```bash
pip install networkx matplotlib
