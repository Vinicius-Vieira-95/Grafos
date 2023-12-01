# import time

# import matplotlib.pyplot as plt
# import networkx as nx

import heapq
from collections import deque


class Grafo:
    bfs_visitados = []

    # Função construtora para inicializar a classe grafo
    def __init__(self): 
        self.g_num_vertices = 0
        self.g_num_arestas = 0
        self.grafo = []
        self.result = []

    # Função para adicionar aresta entre dois vértices
    def add_aresta(self, u, v, peso): 
        self.grafo.append([u, v, peso])
        self.g_num_arestas += 1

    def sub_arvore(self, pai, i):
        if pai[i] == i:
            return i
        return self.sub_arvore(pai, pai[i])
    
    # Item a) retorna o número de vértices do grafo
    def n(self):
        # compara qual dos dois vertices são maiores
        for elemento in self.grafo:
            if elemento[1] > elemento[0]:
                self.g_num_vertices = elemento[1]
            else:
                self.g_num_vertices = elemento[0]
        return self.g_num_vertices
    
    # Item b) retorna o número de arestas do grafo    
    def m(self):
        return self.g_num_arestas

    # Item c) retorna o número de vizinhos do grafo
    def viz(self, vertice):
        vizinhos = []
        for aresta in self.grafo:
            if aresta[0] == vertice:
                vizinhos.append(aresta[1])
        return vizinhos

    # Item d) retorna o grau do vértice v, ou seja o número de arestas incidentes a v
    def d(self, vertice):
        lista = self.vizinho_v(vertice)
        return len(lista)
    
    # Item e) retorna o peso da aresta uv
    def pesos_arestas_entre_vertices(self, u, v):
        peso = 0

        for aresta in self.grafo:
            if (aresta[0] == u and aresta[1] == v) or (
                aresta[0] == u and aresta[1] == v
            ):
                peso = aresta[2]
                break
        if peso == 0:
            return "Não existe aresta entre os vértices informados"
        return peso
    
    # Item g) retorna o gráu máximo presente no grafo
    def max_d(self):
        max_grau = 0

        for vertice in range(1, self.num_vertices() + 1):
            grau_vertice = self.grau_de_v(vertice)
            if grau_vertice > max_grau:
                max_grau = grau_vertice

        return max_grau

    # Item f) retorna o grau mínimo presente no grafo
    def min_d(self):
        # Inicializa o menor grau como infinito ou um valor grande o suficiente
        min_grau = float("inf")

        for vertice in range(1, self.num_vertices() + 1):
            grau_vertice = self.grau_de_v(vertice)
            if grau_vertice < min_grau:
                min_grau = grau_vertice

        # Se todos os vértices tiverem grau 0, atualiza o valor mínimo para 0
        if min_grau == float("inf"):
            min_grau = 0

        return min_grau

    

    # def bfs(self, vertice_inicial):
    #     visitados = set()
    #     fila = deque(
    #         [(vertice_inicial, None)]
    #     )  # Incluindo None para representar o pai do vértice inicial
    #     while fila:
    #         vertice_atual, pai_atual = fila.popleft()
    #         if vertice_atual not in visitados:
    #             # print(vertice_atual, end=" ")
    #             visitados.add(vertice_atual)
    #             vizinhos = self.vizinho_v(vertice_atual)

    #             for vizinho in vizinhos:
    #                 # Adiciona o vértice na fila apenas se não estiver na fila e não for pai do vértice atual
    #                 if vizinho not in visitados and vizinho != pai_atual:
    #                     fila.append((vizinho, vertice_atual))

    #                     # Aqui você pode usar o vértice_atual e vizinho como necessário
    #                     print(f"Pai: {vertice_atual}, Filho: {vizinho}")

    # Item h) Busca de profundidade
    def bfs(self, vertice_inicial):
        visitados = set()
        fila = deque(
            [(vertice_inicial, None, 0)]
        )  # Incluindo a distância como o terceiro elemento da tupla
        while fila:
            vertice_atual, pai_atual, distancia = fila.popleft()
            if vertice_atual not in visitados:
                print(
                    f"Vértice: {vertice_atual}, Pai: {pai_atual}, Distância: {distancia}"
                )
                visitados.add(vertice_atual)
                vizinhos = self.vizinho_v(vertice_atual)

                for vizinho in vizinhos:
                    # Adiciona o vértice na fila apenas se não estiver na fila e não for pai do vértice atual
                    if vizinho not in visitados and vizinho != pai_atual:
                        fila.append((vizinho, vertice_atual, distancia + 1))

    # Item i) Busca em largura
    def dfs(self, vertice_inicial):
        visitados = set()
        tempo = 0  # Inicializa o tempo de visita

        # Dicionários para armazenar pi, v.ini e v.fim para cada vértice
        pi = {}
        v_ini = {}
        v_fim = {}

        stack = [(vertice_inicial, None, 0)]  # Pilha para a DFS
        while stack:
            vertice_atual, pai_atual, estado = stack[-1]

            if estado == 0:
                # Estado 0: Vértice encontrado pela primeira vez
                # print(vertice_atual, end=" ")
                visitados.add(vertice_atual)
                v_ini[vertice_atual] = tempo
                tempo += 1
                pi[vertice_atual] = pai_atual

            vizinhos = self.vizinho_v(vertice_atual)
            for vizinho in vizinhos:
                if vizinho not in visitados:
                    stack.append((vizinho, vertice_atual, 0))
                    break
            else:
                # Estado 1: Todos os vizinhos foram visitados
                stack.pop()
                v_fim[vertice_atual] = tempo
                tempo += 1

        # Imprime os resultados
        print("\nPi:", pi)
        print("\nv.ini:", v_ini)
        print("\nv.fim:", v_fim)
        
        
    # Bellman ford
    def bellman_ford(self, vertice_origem):
        # Inicializa as listas de distâncias e predecessores
        distancias = {v: float("inf") for v in range(1, self.num_vertices() + 1)}
        predecessores = {v: None for v in range(1, self.num_vertices() + 1)}

        # Define a distância da origem para ela mesma como 0
        distancias[vertice_origem] = 0

        # Relaxa as arestas repetidamente
        for _ in range(self.num_vertices() - 1):
            for aresta in self.grafo:
                v1, v2, peso = aresta
                if distancias[v1] + peso < distancias[v2]:
                    distancias[v2] = distancias[v1] + peso
                    predecessores[v2] = v1

        # Verifica se há ciclos negativos
        for aresta in self.grafo:
            v1, v2, peso = aresta
            if distancias[v1] + peso < distancias[v2]:
                print("O grafo contém um ciclo negativo.")
                return None

        return distancias, predecessores
    
    
    # Dijkstra
    def dijkstra(self, vertice_origem):
        # Inicializa as listas de distâncias e predecessores
        distancias = {v: float("inf") for v in range(1, self.num_vertices() + 1)}
        predecessores = {v: None for v in range(1, self.num_vertices() + 1)}

        # Define a distância da origem para ela mesma como 0
        distancias[vertice_origem] = 0

        # Inicializa a fila de prioridade
        fila_prioridade = [(0, vertice_origem)]

        while fila_prioridade:
            dist_vertice, vertice = heapq.heappop(fila_prioridade)

            # Se a distância atual for maior que a armazenada, ignore
            if dist_vertice > distancias[vertice]:
                continue

            # Relaxa as arestas
            vizinhos = self.vizinho_v(vertice)
            for vizinho in vizinhos:
                peso_aresta = self.pesos_arestas_entre_vertices(vertice, vizinho)
                nova_distancia = distancias[vertice] + peso_aresta

                if nova_distancia < distancias[vizinho]:
                    distancias[vizinho] = nova_distancia
                    predecessores[vizinho] = vertice
                    heapq.heappush(fila_prioridade, (nova_distancia, vizinho))

        return distancias, predecessores


# coloque o diretorio onde vai ficar seu arquivo txt

arquivo = open(
    "D:\Arquivos_\Documentos\Tiago\Python\vinicius\Grafos\digrafo\teste2-.txt",
    "r",
    encoding="utf-8",
)


num_vertices = int(arquivo.readline().rstrip())

lista_aresta = []

for linha in arquivo:
    linha_sem_a = linha.replace("a", "")
    # fazendo a conversão do tipo string para inteiro
    conversao = list(map(float, linha_sem_a.split()))
    # adicionando as arestas na lista
    lista_aresta.append(tuple(conversao))

grafo = Grafo()

for elem in lista_aresta:
    grafo.add_aresta(int(elem[0]), int(elem[1]), int(elem[2]))

print("numero de arestas/arcos: ", grafo.m())
print("numero de vertces: ", grafo.n())
print("vizinhos de v: ", grafo.viz(2))
print("grau de vertice: ", grafo.d(2))
print("peso da aresta: ", grafo.w(1, 2))
print("\n BFS: ")
grafo.bfs(2)
print("\n DFS: ")
grafo.dfs(2)
print("\n Bellman-Ford: ")
print(grafo.bellman_ford(8))
print("\n Dijkstra: ")
print(grafo.dijkstra(8))

# print("\n Grau máximo e mínimo:")
# print("maior grau: ", grafo.grau_maximo())
# print("menor grau: ", grafo.grau_minimo())