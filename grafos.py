# montagem do grafo com a entrada
def ler_grafo():
    num_vertices, num_arestas = map(int, input().split())
    tipo_grafo = input()
    grafo = {}

    for _ in range(num_arestas):
        id_aresta, u, v, peso = map(int, input().split())
        if tipo_grafo == 'nao_direcionado':
            grafo.setdefault(u, {}).update({v: id_aresta})
            grafo.setdefault(v, {}).update({u: id_aresta})
        else:
            grafo.setdefault(u, {}).update({v: id_aresta})

    return grafo, tipo_grafo, num_vertices

# validação se o grafo é conexo
def eh_conexo(grafo):
    visitados = set()

    def dfs(vertice):
        visitados.add(vertice)
        for vizinho in grafo.get(vertice, set()):
            if vizinho not in visitados:
                dfs(vizinho)

    dfs(next(iter(grafo)))

    return len(visitados) == len(grafo)

# validação se o grafo é bipartido
def eh_bipartido(grafo):
    cores = {}

    def dfs(vertice, cor):
        cores[vertice] = cor
        for vizinho in grafo.get(vertice, set()):
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
def eh_euleriano(grafo):
    if not eh_conexo(grafo):
        return False

    graus_impares = sum(1 for vertice in grafo if len(grafo[vertice]) % 2 == 1)

    return graus_impares == 0

# validação se o grafo não direcionado possui ciclo
def possui_ciclo_nao_direcionado(grafo):
    visitados = set()
    parent = {}

    def dfs(vertice, pai):
        visitados.add(vertice)
        for vizinho in grafo.get(vertice, set()):
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
        for vizinho in grafo.get(vertice, set()):
            if cores[vizinho] == cinza:
                return True
            if cores[vizinho] == branco and dfs(vizinho):
                return True
        cores[vertice] = preto
        return False

    for vertice in grafo:
        if cores[vertice] == branco:
            if dfs(vertice):
                return True

    return False

# calculo da qtd de componentes conexas
def num_componentes_conexas(grafo):
    visitados = set()
    num_componentes = 0

    def dfs(vertice):
        visitados.add(vertice)
        for vizinho in grafo.get(vertice, set()):
            if vizinho not in visitados:
                dfs(vizinho)

    for vertice in grafo:
        if vertice not in visitados:
            dfs(vertice)
            num_componentes += 1

    return num_componentes if num_componentes > 0 else -1

# calcula numero de componentes fortemente conexas em grafo direcionado usando tarjan
def componentes_fortemente_conexas(grafo):
    max_vertex_id = max(grafo.keys()) 
    n = max_vertex_id + 1 
    index = 0
    indices = [-1] * n
    baixos = [-1] * n
    na_pilha = [False] * n
    pilha = []
    componentes = []

    def tarjan(v):
        nonlocal index
        indices[v] = index
        baixos[v] = index
        index += 1
        pilha.append(v)
        na_pilha[v] = True

        for vizinho in grafo.get(v, set()):
            if indices[vizinho] == -1:
                tarjan(vizinho)
                baixos[v] = min(baixos[v], baixos[vizinho])
            elif na_pilha[vizinho]:
                baixos[v] = min(baixos[v], indices[vizinho])

        if baixos[v] == indices[v]:
            componente = []
            while True:
                w = pilha.pop()
                na_pilha[w] = False
                componente.append(w)
                if w == v:
                    break
            componentes.append(componente)

    for v in range(n):
        if indices[v] == -1:
            tarjan(v)

    return len(componentes)

# encontra os vertices de articulaçao em grafo nao direcionado
def vertices_de_articulacao(grafo):
    n = len(grafo)
    visitado = [False] * n
    descoberta = [-1] * n
    baixo = [-1] * n
    pai = [-1] * n
    tempo = 0
    articulacoes = []

    def dfs(u):
        nonlocal tempo
        filhos = 0
        visitado[u] = True
        descoberta[u] = baixo[u] = tempo
        tempo += 1

        for v in grafo.get(u, []):
            if not visitado[v]:
                pai[v] = u
                filhos += 1
                dfs(v)
                baixo[u] = min(baixo[u], baixo[v])

                if (pai[u] == -1 and filhos > 1) or (pai[u] != -1 and baixo[v] >= descoberta[u]):
                    articulacoes.append(u)
            elif v != pai[u]:
                baixo[u] = min(baixo[u], descoberta[v])

    for i in grafo: 
        if not visitado[i]:
            dfs(i)

    return sorted(articulacoes)

# encontra as arestas ponte em grafo nao direcionado
def arestas_ponte(grafo):
    n = len(grafo)
    visitado = [False] * n
    descoberta = [-1] * n
    baixo = [-1] * n
    pai = [-1] * n
    tempo = 0
    pontes = []

    def dfs(u):
        nonlocal tempo
        filhos = 0
        visitado[u] = True
        descoberta[u] = baixo[u] = tempo
        tempo += 1

        for v in grafo.get(u, []):
            if not visitado[v]:
                pai[v] = u
                filhos += 1
                dfs(v)
                baixo[u] = min(baixo[u], baixo[v])

                if baixo[v] > descoberta[u]:
                    pontes.append((u, v))
            elif v != pai[u]:
                baixo[u] = min(baixo[u], descoberta[v])

    for i in grafo:
        if not visitado[i]:
            dfs(i)

    return pontes

# verifica a arvore de largura e imprime o identificador das arestas
def bfs(grafo, origem=0):
    visitados = set()
    fila = [origem]
    arvore = []

    while fila:
        vertice = fila.pop(0)
        visitados.add(vertice)

        for vizinho in sorted(grafo.get(vertice, set())):
            if vizinho not in visitados:
                fila.append(vizinho)
                visitados.add(vizinho)
                id_aresta = grafo[vertice][vizinho] 
                arvore.append((vertice, vizinho, id_aresta))

    return arvore

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

# calcular MST
def kruskal(grafo):
    result = [] 
    i = 0
    e = 0

    arestas = [(u, v, peso) for u in grafo for v, peso in grafo[u].items() if v < u]

    arestas.sort(key=lambda item: item[2])

    pai = []
    rank = []

    for node in range(len(grafo)):
        pai.append(node)
        rank.append(0)

    while e < len(grafo) - 1:
        u, v, w = arestas[i]
        i = i + 1
        x = find(pai, u)
        y = find(pai, v)

        if x != y:
            e = e + 1
            result.append([u, v, w])
            union(pai, rank, x, y)

    peso_total = sum(w for u, v, w in result)
    return peso_total


def main():
    propriedades_desejadas = list(map(int, input().split()))
    grafo, tipo_grafo, num_vertices = ler_grafo()

    resultados = {}  # para armazenar os resultados e imprimi-los na ordem correta

    for propriedade in propriedades_desejadas:
        if propriedade == 0: # verificar se o grafo é conexo
            resultados[propriedade] = 1 if eh_conexo(grafo) else 0
        elif propriedade == 1: # verificar se um grafo não direcionado é bipartido
            resultados[propriedade] = 1 if eh_bipartido(grafo) else 0
        elif propriedade == 2: # verificar se um grafo é euleriano
            resultados[propriedade] = 1 if eh_euleriano(grafo) else 0
        elif propriedade == 3: # verificar se um grafo possui ciclo
            if tipo_grafo == 'nao_direcionado':
                resultados[propriedade] = 1 if possui_ciclo_nao_direcionado(grafo) else 0
            else:
                resultados[propriedade] = 1 if possui_ciclo_direcionado(grafo, num_vertices) else 0
        elif propriedade == 4:  # calcular qtd de componentes conexas em grafo não direcionado
            if tipo_grafo == 'nao_direcionado':
                resultados[propriedade] = num_componentes_conexas(grafo)
            else:
                resultados[propriedade] = -1
        elif propriedade == 5: # calcular qtd de componentes fortemente conexas em grafo direcionado
            if tipo_grafo == 'direcionado':
                resultados[propriedade] = componentes_fortemente_conexas(grafo)
            else:
                resultados[propriedade] = -1
        elif propriedade == 6: # imprimir vertices de articulação de grafo não direcionado
            if tipo_grafo == 'nao_direcionado':
                articulacoes = vertices_de_articulacao(grafo)
                if articulacoes:
                    resultados[propriedade] = " ".join(map(str, articulacoes)) 
                else:
                    resultados[propriedade] = 0
            else:
                resultados[propriedade] = -1
        elif propriedade == 7:  # calcular qtd de arestas ponte em grafo não direcionado
            if tipo_grafo == 'nao_direcionado':
                pontes = arestas_ponte(grafo)
                resultados[propriedade] = len(pontes)
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
        elif propriedade == 10:  # calcular o valor final da MST para grafos não direcionados
            if tipo_grafo == 'nao_direcionado':
                resultados[propriedade] = kruskal(grafo)
            else:
                resultados[propriedade] = -1

    for propriedade in propriedades_desejadas:
        print(resultados[propriedade])


if __name__ == "__main__":
    main()