class Edge(object):
    def __init__(self, _from, to, weight):
        self.src = _from
        self.dst = to
        self.weight = weight

    def __eq__(self, other):
        return (self.src == other.src and self.dst == other.dst and self.weight == other.weight)

    def __hash__(self):
        return hash((self.src, self.dst, self.weight))
    
    def __str__(self) -> str:
        return " "+str(self.src)+" "+str(self.dst)+" "+str(self.weight)


class Graph:

    def __init__(self, nodes, arg_edgelist):
        self.nodes = nodes
        self.num_nodes = len(self.nodes)
        self.edgelist = arg_edgelist
        self.parent = []
        self.rank = []
        # mst stores edges of the minimum spanning tree
        self.mst = []

    def FindParent(self, node):
        # With path-compression.

        if node != self.parent[node]:
            self.parent[node] = self.FindParent(self.parent[node])
        return self.parent[node]

    def KruskalMST(self):

        # Sort objects of an Edge class based on attribute (weight)
        self.edgelist.sort(key=lambda Edge: Edge.weight)

        self.parent = [None] * self.num_nodes
        self.rank = [None] * self.num_nodes

        for n in self.nodes:
            self.parent[n] = n  # Every node is the parent of itself at the beginning
            self.rank[n] = 0   # Rank of every node is 0 at the beginning

        for edge in self.edgelist:
            root1 = self.FindParent(edge.src)
            root2 = self.FindParent(edge.dst)

            # Parents of the source and destination nodes are not in the same subset
            # Add the edge to the spanning tree
            if root1 != root2:
                self.mst.append(edge)
                if self.rank[root1] < self.rank[root2]:
                    self.parent[root1] = root2
                    self.rank[root2] += 1
                else:
                    self.parent[root2] = root1
                    self.rank[root1] += 1

        cost = 0
        for edge in self.mst:
            cost += edge.weight

#ben_modif
    #do not take delete into account
    def partial_Kruskal_MST(self, old_mst_precovered) : 

        self.edgelist.sort(key=lambda Edge: Edge.weight)

        self.parent = [None] * self.num_nodes
        self.rank = [None] * self.num_nodes

        for n in self.nodes:
            self.parent[n] = n  # Every node is the parent of itself at the beginning
            self.rank[n] = 0   # Rank of every node is 0 at the beginning


        #Setting up edges from the previous MST for the pre-covered cells
        for i in range(len(old_mst_precovered)) :
            edge = old_mst_precovered[ (len(old_mst_precovered)) - i -1 ]
            print("INSIDE")
            print(self.edgelist)
            print(IsEdgeinList(edge, self.edgelist))
            if not (IsEdgeinList(edge, self.edgelist)) : 
                print("ULTIMATE ERROR "+str(edge))
                print(self.edgelist)
                exit(1)

            root1 = self.FindParent(edge.src)
            root2 = self.FindParent(edge.dst)

            # Parents of the source and destination nodes are not in the same subset
            # Add the edge to the spanning tree
            if root1 != root2:
                self.mst.append(edge)
                if self.rank[root1] < self.rank[root2]:
                    self.parent[root1] = root2
                    self.rank[root2] += 1
                else:
                    self.parent[root2] = root1
                    self.rank[root1] += 1

        #Building the rest of the MST
        for edge in self.edgelist:
            if (not (edge in self.mst)) : 
                root1 = self.FindParent(edge.src)
                root2 = self.FindParent(edge.dst)

                # Parents of the source and destination nodes are not in the same subset
                # Add the edge to the spanning tree
                if root1 != root2:
                    self.mst.append(edge)
                    if self.rank[root1] < self.rank[root2]:
                        self.parent[root1] = root2
                        self.rank[root2] += 1
                    else:
                        self.parent[root2] = root1
                        self.rank[root1] += 1

        cost = 0
        for edge in self.mst:
            cost += edge.weight


def IsEdgeinList(edge, edge_list) :

    for edge_test in edge_list : 
        print("EDGE TEST" +str(edge_test))
        print(edge)
        if edge.src == edge_test.src and edge.dst == edge_test.dst : 
            return True
        
    return False


