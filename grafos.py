def ler_grafo(): # leitura do grafo
    num_vertices, num_arestas = map(int, input().split())
    tipo_grafo = input()
    grafo = {}  # lista de adjacências

    for _ in range(num_arestas):
        id_aresta, u, v, peso = map(int, input().split())
        if tipo_grafo == 'nao_direcionado':
            grafo.setdefault(u, []).append(v)
            grafo.setdefault(v, []).append(u)
        else:
            grafo.setdefault(u, []).append(v)

    return grafo, tipo_grafo


# verificação se o grafo é conexo
def eh_conexo(grafo):
    visitados = set()

    def dfs(vertice):
        visitados.add(vertice)
        for vizinho in grafo.get(vertice, []):
            if vizinho not in visitados:
                dfs(vizinho)

    dfs(next(iter(grafo)))  

    return len(visitados) == len(grafo)


# verifica se o grafo é bipartido
def eh_bipartido(grafo):
    cores = {}

    def dfs(vertice, cor):
        cores[vertice] = cor
        for vizinho in grafo.get(vertice, []):
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

# verifica se o grafo é euleriano
def eh_euleriano(grafo):
    if not eh_conexo(grafo):
        return False

    graus_impares = sum(1 for vertice in grafo if len(grafo[vertice]) % 2 == 1)

    return graus_impares == 0 

# verifica se um grafo nao direcionado possui ciclo
def possui_ciclo_nao_direcionado(grafo):
    visitados = set()
    parent = {}

    def dfs(vertice, pai):
        visitados.add(vertice)
        for vizinho in grafo.get(vertice, []):
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

# verifica se um grafo direcionado possui ciclo
def possui_ciclo_direcionado(grafo):
    branco, cinza, preto = 0, 1, 2
    cores = {v: branco for v in grafo}

    def dfs(vertice):
        cores[vertice] = cinza
        for vizinho in grafo.get(vertice, []):
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

# calcula numero de componentes conexas
def num_componentes_conexas(grafo):
    visitados = set()
    num_componentes = 0

    def dfs(vertice):
        visitados.add(vertice)
        for vizinho in grafo.get(vertice, []):
            if vizinho not in visitados:
                dfs(vizinho)

    for vertice in grafo:
        if vertice not in visitados:
            dfs(vertice)
            num_componentes += 1

    return num_componentes if num_componentes > 0 else -1  # Retorna -1

# calcula numero de componentes fortemente conexas em grafo direcionado usando tarjan
def componentes_fortemente_conexas(grafo):
    n = len(grafo)
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

        for vizinho in grafo.get(v, []):
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


def main():
    propriedades_desejadas = list(map(int, input().split()))
    grafo, tipo_grafo = ler_grafo()

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
                resultados[propriedade] = 1 if possui_ciclo_direcionado(grafo) else 0
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

    for propriedade in propriedades_desejadas:
        print(resultados[propriedade])


if __name__ == "__main__":
    main()