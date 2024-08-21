'''
Código desenvolvido por:
Gabriel Camargos Alves
Yasmim Danzieri Abbondanza Laurentino
'''

import heapq
from collections import deque
import networkx as nx
import matplotlib.pyplot as plt


# função para criar e visualizar o grafo
def visualizar_grafo(grafo, tipo_grafo):
    G = nx.DiGraph() if tipo_grafo == 'direcionado' else nx.Graph()

    for u in grafo:
        for v, peso, id_aresta in grafo[u]:
            G.add_edge(u, v, weight=peso, label=id_aresta)

    nro_vertices = len(G.nodes)

    if nro_vertices <= 10:
        k = 1.2
        tamanhoFigura = (4, 5)
    elif nro_vertices <= 20:
        k = 0.8
        tamanhoFigura = (6, 8)
    else:
        k = 0.5
        tamanhoFigura = (8, 10)


    pos = nx.spring_layout(G,k=k)

    # Desenho do grafo
    plt.figure(figsize=tamanhoFigura)
    nx.draw(G, pos, with_labels=True, node_color='cyan', font_weight='bold', node_size=500, font_size=10)

    # Desenho das arestas com pesos
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

    plt.title(f"Grafo {tipo_grafo}")
    plt.show()


# leitura da entrada para montagem do grafo
def ler_grafo():
    num_vertices, num_arestas = map(int, input().split())
    tipo_grafo = input().strip()
    grafo = {}

    for _ in range(num_arestas):
        id_aresta, u, v, peso = map(int, input().split())
        if tipo_grafo == 'nao_direcionado':
            grafo.setdefault(u, []).append((v, peso, id_aresta))
            grafo.setdefault(v, []).append((u, peso, id_aresta))
        else:
            grafo.setdefault(u, []).append((v, peso, id_aresta))

    return grafo, tipo_grafo, num_vertices

# função auxiliar
def possui_pesos_diferentes(grafo):
    pesos = set()
    for u in grafo:
        for v, peso, _ in grafo[u]:
            pesos.add(peso)
            if len(pesos) > 1:
                return True
    return False

# função auxiliar
def find(pai, i):
    if pai[i] == i:
        return i
    return find(pai, pai[i])

# função auxiliar
def union(pai, rank, x, y):
    raiz_x = find(pai, x)
    yroot = find(pai, y)

    if rank[raiz_x] < rank[yroot]:
        pai[raiz_x] = yroot
    elif rank[raiz_x] > rank[yroot]:
        pai[yroot] = raiz_x
    else:
        pai[yroot] = raiz_x
        rank[raiz_x] += 1

# validação se o grafo é conexo
def conexo(grafo, num_vertices):
    visitado = set()

    def dfs(v):
        visitado.add(v)
        for vizinho, _, _ in grafo.get(v, []):
            if vizinho not in visitado:
                dfs(vizinho)

    inicio = next(iter(grafo))
    dfs(inicio)
    
    return len(visitado) == num_vertices


# validação se o grafo é bipartido
def bipartido(grafo):
    cores = {}
    
    def dfs(vertice, cor):
        cores[vertice] = cor
        for vizinho, _, _ in grafo.get(vertice, []):
            if vizinho not in cores:
                if not dfs(vizinho, 1 - cor):
                    return False
            elif cores[vizinho] == cor:
                return False
        return True

    for vertice in grafo:
        if vertice not in cores:
            if not dfs(vertice, 0):
                return False

    return True


# validação se o grafo é euleriano
def euleriano(grafo, num_vertices):
    if not conexo(grafo, num_vertices):
        return False

    graus_impares = sum(1 for vertice in grafo if len(grafo[vertice]) % 2 == 1)

    return graus_impares == 0

# validação se o grafo não direcionado possui ciclo
def possui_ciclo_nao_direcionado(grafo):
    visitados = set()
    parent = {}

    def dfs(vertice, pai):
        visitados.add(vertice)
        for vizinho, _, _ in grafo.get(vertice, []):
            if vizinho not in visitados:
                parent[vizinho] = vertice
                if dfs(vizinho, vertice):
                    return True
            elif vizinho != pai:
                return True
        return False

    for vertice in grafo:
        if vertice not in visitados:
            if dfs(vertice, None):
                return True

    return False

# validação se o grafo direcionado possui ciclo
def possui_ciclo_direcionado(grafo, num_vertices):
    branco, cinza, preto = 0, 1, 2
    cores = {v: branco for v in range(num_vertices)}

    def dfs(vertice):
        cores[vertice] = cinza
        for vizinho, _, _ in grafo.get(vertice, []):
            if cores[vizinho] == cinza:
                return True
            if cores[vizinho] == branco and dfs(vizinho):
                return True
        cores[vertice] = preto
        return False

    for vertice in range(num_vertices):
        if cores[vertice] == branco:
            if dfs(vertice):
                return True
    return False

# calculo da quantidade de componentes conexas no grafo
def num_componentes_conexas(grafo):
    visitados = set()
    num_componentes = 0
    vertices = set(grafo.keys())

    def dfs(vertice):
        visitados.add(vertice)
        for vizinho, _, _ in grafo.get(vertice, []):
            if vizinho not in visitados:
                dfs(vizinho)

    for vertice in vertices:
        if vertice not in visitados:
            dfs(vertice)
            num_componentes += 1

    for vertice in range(max(vertices) + 1):
        if vertice not in vertices:
            num_componentes += 1

    return num_componentes


# calculo do numero de componentes fortemente conexas em grafo direcionado usando tarjan
def componentes_fortemente_conexas(grafo):
    indice = 0
    pilha = []
    indices = {}
    baixos = {}
    num_componentes = 0

    def tarjan(v):
        nonlocal indice, num_componentes
        indices[v] = baixos[v] = indice
        indice += 1
        pilha.append(v)
        
        for vizinho, _, _ in grafo.get(v, []):
            if vizinho not in indices:
                tarjan(vizinho)
                baixos[v] = min(baixos[v], baixos[vizinho])
            elif vizinho in pilha:
                baixos[v] = min(baixos[v], indices[vizinho])
        
        if baixos[v] == indices[v]:
            num_componentes += 1
            while True:
                w = pilha.pop()
                if w == v:
                    break

    for v in range(len(grafo)):
        if v not in indices:
            tarjan(v)

    return num_componentes

# encontra os vertices de articulaçao em grafo nao direcionado
def vertices_de_articulacao(grafo):
    visitado = {}
    descoberta = {}
    baixo = {}
    pai = {}
    tempo = [0]
    articulacoes = []

    def dfs(u):
        filhos = 0
        visitado[u] = True
        descoberta[u] = baixo[u] = tempo[0]
        tempo[0] += 1

        for v, _, _ in grafo.get(u, []):
            if v not in visitado:
                pai[v] = u
                filhos += 1
                dfs(v)
                baixo[u] = min(baixo[u], baixo[v])

                if (pai.get(u) is None and filhos > 1) or (pai.get(u) is not None and baixo[v] >= descoberta[u]):
                    articulacoes.append(u)
            elif v != pai.get(u):
                baixo[u] = min(baixo[u], descoberta[v])

    for i in grafo:
        if i not in visitado:
            dfs(i)

    return sorted(articulacoes)

# encontra as arestas ponte em grafo nao direcionado
def arestas_ponte(grafo):
    visitado = set()
    descoberta = {}
    baixo = {}
    pai = {}
    pontes = []
    tempo = [0]
    def dfs(u):
        visitado.add(u)
        descoberta[u] = baixo[u] = tempo[0]
        tempo[0] += 1

        for v, _, _ in grafo.get(u, []):
            if v not in descoberta:
                pai[v] = u
                dfs(v)
                baixo[u] = min(baixo[u], baixo[v])

                if baixo[v] > descoberta[u]:
                    pontes.append((u, v))
            elif v != pai.get(u):
                baixo[u] = min(baixo[u], descoberta[v])

    for i in grafo:
        if i not in descoberta:
            dfs(i)

    return sorted(pontes)

# realiza a busca em profundidade e retorna a árvore com os identificadores das arestas
def dfs_tree(grafo, origem=0):
    visitados = set()
    arvore = []
    
    def dfs(vertice, id_aresta_pai=None):
        visitados.add(vertice)
        if id_aresta_pai is not None:
            arvore.append(id_aresta_pai)

        for vizinho, _, id_aresta in grafo.get(vertice, []):
            if vizinho not in visitados:
                dfs(vizinho, id_aresta)

    dfs(origem)
    return arvore

# verifica a arvore de largura e imprime o identificador das arestas
def bfs(grafo, origem=0):
    visitados = set()
    fila = [origem]
    arvore = []

    while fila:
        vertice = fila.pop(0)
        visitados.add(vertice)

        for vizinho, peso, id_aresta in sorted(grafo.get(vertice, [])):
            if vizinho not in visitados:
                fila.append(vizinho)
                visitados.add(vizinho)
                arvore.append((vertice, vizinho, id_aresta))

    return arvore


# algoritmo de Kruskal para calcular o valor da MST
def arvore_geradora_minima(grafo, num_vertices):
    arestas = []
    for u in grafo:
        for v, peso, _ in grafo[u]:
            if u < v:
                arestas.append((peso, u, v))
    
    arestas.sort()

    pai = [i for i in range(num_vertices)]
    rank = [0] * num_vertices

    valor_mst = 0
    arestas_incluidas = 0

    for peso, u, v in arestas:
        raiz_u = find(pai, u)
        raiz_v = find(pai, v)

        if raiz_u != raiz_v:
            valor_mst += peso
            union(pai, rank, raiz_u, raiz_v)
            arestas_incluidas += 1

        if arestas_incluidas == num_vertices - 1:
            break

    return valor_mst if arestas_incluidas == num_vertices - 1 else -1

# faz a ordenação topologica dos vertices
def ordenacao_topologica(grafo):
    graus_entrada = {nó: 0 for nó in grafo}
    for nó in grafo:
        for vizinho, _, _ in grafo[nó]:
            graus_entrada[vizinho] = graus_entrada.get(vizinho, 0) + 1

    grau_zero = sorted([nó for nó in graus_entrada if graus_entrada[nó] == 0])

    ordem_topologica = []
    while grau_zero:
        nó = grau_zero.pop(0)
        ordem_topologica.append(nó)
        for vizinho, _, _ in grafo.get(nó, []):
            graus_entrada[vizinho] -= 1
            if graus_entrada[vizinho] == 0:
                grau_zero.append(vizinho)
                grau_zero = sorted(grau_zero)

    if len(ordem_topologica) == len(graus_entrada):
        return ordem_topologica
    else:
        return -1

# algoritmo de edmonds-karp para calcular o fluxo máximo
def edmonds_karp(grafo, origem, destino, num_vertices):
    fluxo_maximo = 0
    capacidade_residual = {u: {v: peso for v, peso, _ in grafo.get(u, [])} for u in range(num_vertices)}

    while True:
        fila = deque([origem])
        pais = {origem: None}
        while fila:
            u = fila.popleft()
            for v, capacidade in capacidade_residual[u].items():
                if v not in pais and capacidade > 0:
                    pais[v] = u
                    if v == destino:
                        break
                    fila.append(v)
            else:
                continue
            break
        else:
            break

        v = destino
        capacidade_minima = float('Inf')
        while v != origem:
            u = pais[v]
            capacidade_minima = min(capacidade_minima, capacidade_residual[u][v])
            v = u

        v = destino
        while v != origem:
            u = pais[v]
            capacidade_residual[u][v] -= capacidade_minima
            capacidade_residual[v][u] = capacidade_residual.get(v, {}).get(u, 0) + capacidade_minima
            v = u

        fluxo_maximo += capacidade_minima

    return fluxo_maximo

# calcula o fecho transitivo do vértice 0 em grafos direcionados
def fecho_transitivo(grafo, origem=0):
    visitados = set()
    pilha = [origem]

    while pilha:
        vertice = pilha.pop()
        if vertice not in visitados:
            visitados.add(vertice)
            for vizinho, _, _ in sorted(grafo.get(vertice, [])):
                if vizinho not in visitados:
                    pilha.append(vizinho)

    visitados.discard(origem)  # remove o vértice de origem 0 do conjunto visitados
    return sorted(visitados)

# função para calcular o caminho mínimo entre 0 e n-1 usando dijkstra
def caminho_minimo(grafo, num_vertices):
    dist = {v: float('inf') for v in range(num_vertices)}
    dist[0] = 0
    pq = [(0, 0)]

    visitados = set()

    while pq:
        d, u = heapq.heappop(pq)
        if u in visitados:
            continue
        visitados.add(u)
        
        for v, peso, _ in grafo.get(u, []):
            if v not in visitados and dist[u] + peso < dist[v]:
                dist[v] = dist[u] + peso
                heapq.heappush(pq, (dist[v], v))

    return dist[num_vertices - 1] if dist[num_vertices - 1] != float('inf') else -1

def main():
    propriedades_desejadas = list(map(int, input().split()))
    grafo, tipo_grafo, num_vertices = ler_grafo()

    visualizar_grafo(grafo, tipo_grafo)

    resultados = {}  # para armazenar os resultados e imprimir na ordem correta

    for propriedade in propriedades_desejadas:
        if propriedade == 0:  # verificar se o grafo é conexo
            resultados[propriedade] = 1 if conexo(grafo, num_vertices) else 0
        elif propriedade == 1:  # verificar se um grafo não direcionado é bipartido
            resultados[propriedade] = 1 if bipartido(grafo) else 0
        elif propriedade == 2:  # verificar se um grafo é euleriano
            resultados[propriedade] = 1 if euleriano(grafo, num_vertices) else 0
        elif propriedade == 3:  # verificar se um grafo possui ciclo
            if tipo_grafo == 'nao_direcionado':
                resultados[propriedade] = 1 if possui_ciclo_nao_direcionado(grafo) else 0
            else:
                resultados[propriedade] = 1 if possui_ciclo_direcionado(grafo, num_vertices) else 0
        elif propriedade == 4:  # calcular quantidade de componentes conexas em grafo não direcionado
            if tipo_grafo == 'nao_direcionado':
                resultados[propriedade] = num_componentes_conexas(grafo)
            else:
                resultados[propriedade] = -1
        elif propriedade == 5:  # calcular quantidade de componentes fortemente conexas em grafo direcionado
            if tipo_grafo == 'direcionado':
                resultados[propriedade] = componentes_fortemente_conexas(grafo)
            else:
                resultados[propriedade] = -1
        elif propriedade == 6:  # imprimir vertices de articulação de grafo não direcionado
            if tipo_grafo == 'nao_direcionado':
                articulacoes = vertices_de_articulacao(grafo)
                if articulacoes:
                    resultados[propriedade] = " ".join(map(str, articulacoes))
                else:
                    resultados[propriedade] = 0
            else:
                resultados[propriedade] = -1
        elif propriedade == 7:  # calcular quantidade de arestas ponte em grafo não direcionado
            if tipo_grafo == 'nao_direcionado':
                pontes = arestas_ponte(grafo)
                resultados[propriedade] = len(pontes)
            else:
                resultados[propriedade] = -1
        elif propriedade == 8:  # arvore em profundidade
            if conexo(grafo, num_vertices):
                arvore_profundidade = dfs_tree(grafo)
            else:
                arvore_profundidade = dfs_tree(grafo, origem=0)
            if arvore_profundidade:
                resultados[propriedade] = " ".join(map(str, arvore_profundidade))
            else:
                resultados[propriedade] = -1
        elif propriedade == 9:  # arvore de largura
            if 0 in grafo:
                arvore_largura = bfs(grafo)
                if arvore_largura:
                    resultados[propriedade] = " ".join(
                        map(str, [id_aresta for _, _, id_aresta in arvore_largura])
                    )
                else:
                    resultados[propriedade] = 0
            else:
                resultados[propriedade] = -1
        elif propriedade == 10:  # calcular valor final da MST
            if tipo_grafo == 'nao_direcionado':
                resultados[propriedade] = arvore_geradora_minima(grafo, num_vertices)
            else:
                resultados[propriedade] = -1
        elif propriedade == 11:  # realiza ordenação topologica
            if tipo_grafo == 'nao_direcionado':
                resultados[propriedade] = -1
            else:
                resultados[propriedade] = ordenacao_topologica(grafo)
        elif propriedade == 12:  # calcular o valor do caminho mínimo entre os vértices 0 e n-1
            if tipo_grafo == 'nao_direcionado' and possui_pesos_diferentes(grafo):
                resultados[propriedade] = caminho_minimo(grafo, num_vertices)
            else:
                resultados[propriedade] = -1
        elif propriedade == 13:  # calcular o fluxo máximo
            if tipo_grafo == 'direcionado':
                fluxo_maximo = edmonds_karp(grafo, 0, num_vertices - 1, num_vertices)
                resultados[propriedade] = fluxo_maximo
            else:
                resultados[propriedade] = -1
        elif propriedade == 14:  # fecho transitivo do vértice 0
            if tipo_grafo == 'direcionado':
                fecho = fecho_transitivo(grafo, origem=0)
                if fecho:
                    resultados[propriedade] = " ".join(map(str, fecho))
                else:
                    resultados[propriedade] = -1
            else:
                resultados[propriedade] = -1

    for propriedade in propriedades_desejadas:
        print(resultados[propriedade])

if __name__ == "__main__":
    main()