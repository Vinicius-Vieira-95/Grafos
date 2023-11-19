import networkx as nx
import matplotlib.pyplot as plt
import time

class Grafo:
    
    def __init__(self):
        self.g_num_vertices = 0
        self.g_num_arestas = 0
        self.grafo = []
        self.result = []
    
    
    def add_aresta(self, v1, v2, peso):
        self.grafo.append([v1, v2, peso])
        
    def sub_arvore(self, pai, i):
        if pai[i] == i:
            return i
        return self.sub_arvore(pai, pai[i])
    
    def num_vertices(self):
        # compara qual dos dois vertices são maiores
        for elemento in self.grafo:
            if(elemento[1] > elemento[0]):
                self.g_num_vertices = elemento[1]
            else:
                self.g_num_vertices = elemento[0]
        return self.g_num_vertices
    
    def num_aresta(self):
        self.g_num_arestas = len(self.grafo)/2
        return self.g_num_arestas
    
    def vizinho_v(self, vertice):
        vizinhos = []
        for aresta in self.grafo:
            if aresta[0] == vertice:
                vizinhos.append(aresta[1])
        return vizinhos   


#coloque o diretorio onde vai ficar seu arquivo txt
arquivo = open('C:\\Users\\Pichau\\Desktop\\Projeto\\Grafos\\av2\\grafo-main-av2.txt', 'r', encoding='utf-8')

num_vertices = int(arquivo.readline().rstrip())

lista_aresta = []

for linha in arquivo:
    linha_sem_a = linha.replace('a', '')
    # fazendo a conversão do tipo string para inteiro
    conversao = list(map(float, linha_sem_a.split()))
    # adicionando as arestas na lista
    lista_aresta.append(tuple(conversao))
    
grafo = Grafo()

for elem in lista_aresta:
    grafo.add_aresta(int(elem[0]), int(elem[1]), int(elem[2]))
    
print('numero de arestas: ', grafo.num_aresta())
print('numero de vertces',grafo.num_vertices())
print('vizinhos de v', grafo.vizinho_v(1))