# import time

# import matplotlib.pyplot as plt
# import networkx as nx

from collections import deque


class Grafo:
    bfs_visitados = []

    def __init__(self):
        self.g_num_vertices = 0
        self.g_num_arestas = 0
        self.grafo = []
        self.result = []

    def add_aresta(self, v1, v2, peso):
        self.grafo.append([v1, v2, peso])
        self.g_num_arestas += 1

    def sub_arvore(self, pai, i):
        if pai[i] == i:
            return i
        return self.sub_arvore(pai, pai[i])

    def num_vertices(self):
        # compara qual dos dois vertices são maiores
        for elemento in self.grafo:
            if elemento[1] > elemento[0]:
                self.g_num_vertices = elemento[1]
            else:
                self.g_num_vertices = elemento[0]
        return self.g_num_vertices

    def num_aresta(self):
        return self.g_num_arestas

    def vizinho_v(self, vertice):
        vizinhos = []
        for aresta in self.grafo:
            if aresta[0] == vertice:
                vizinhos.append(aresta[1])
        return vizinhos

    def grau_de_v(self, vertice):
        lista = self.vizinho_v(vertice)
        return len(lista)

    def grau_maximo(self):
        max_grau = 0

        for vertice in range(1, self.num_vertices() + 1):
            grau_vertice = self.grau_de_v(vertice)
            if grau_vertice > max_grau:
                max_grau = grau_vertice

        return max_grau

    def grau_minimo(self):
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

    def pesos_arestas_entre_vertices(self, v1, v2):
        peso = 0

        for aresta in self.grafo:
            if (aresta[0] == v1 and aresta[1] == v2) or (
                aresta[0] == v2 and aresta[1] == v1
            ):
                peso = aresta[2]
                break
        if peso == 0:
            return "Não existe aresta entre os vértices informados"
        return peso

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

    def dfs(self, vertice_inicial):
        visitados = set()

        def dfs_recursivo(vertice):
            print(vertice, end=" ")
            visitados.add(vertice)

            vizinhos = self.vizinho_v(vertice)
            for vizinho in vizinhos:
                if vizinho not in visitados:
                    dfs_recursivo(vizinho)

        dfs_recursivo(vertice_inicial)


# coloque o diretorio onde vai ficar seu arquivo txt
arquivo = open(
    "C:\\Users\\Pichau\\Desktop\\Projeto\\Grafos\\digrafo\\teste-av2.txt",
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

print("numero de arestas: ", grafo.num_aresta())
print("numero de vertces: ", grafo.num_vertices())
print("vizinhos de v: ", grafo.vizinho_v(2))
print("grau de vertice: ", grafo.grau_de_v(2))
print("peso da aresta: ", grafo.pesos_arestas_entre_vertices(1, 2))
grafo.bfs(2)
print()
# grafo.dfs(4)
# print("maior grau: ", grafo.grau_maximo())
# print("menor grau: ", grafo.grau_minimo())
