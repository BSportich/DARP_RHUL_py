import numpy as np
from Edges import Edge, Graph, IsEdgeinList
import sys


class CalculateTrajectories():
    def __init__(self, r, c, MST):
        self.MAX_NODES = 4*r*c
        self.PathSequence = []
        self.rows = r
        self.cols = c
        self.MSTvector = MST
        self.MSTedges = len(self.MSTvector)
        self.allEdges = set()
        self.nodes = {}

        self.visited_nodes = []
        for node in range(self.MAX_NODES):
            self.nodes[node] = None

    def initializeGraph(self, A, connect4):
        for i in range(2*self.rows):
            for j in range(2*self.cols):
                if A[i, j]:
                    if i > 0 and A[i-1][j]:
                        self.AddToAllEdges(i*2*self.cols+j, (i-1)*2*self.cols+j, 1)
                    if i < 2*self.rows-1 and A[i+1][j]:
                        self.AddToAllEdges(i*2*self.cols+j, (i+1)*2*self.cols+j, 1)
                    if j > 0 and A[i][j-1]:
                        self.AddToAllEdges(i*2*self.cols+j, i*2*self.cols+j-1, 1)
                    if j < 2*self.cols-1 and A[i][j+1]:
                        self.AddToAllEdges(i*2*self.cols+j, i*2*self.cols+j+1, 1)

                    if not connect4:
                        exit()
                        if i > 0 and j > 0 and A[i-1][j-1]:
                            self.AddToAllEdges(i*2*self.cols+j, (i-1)*2*self.cols+j-1, 1)
                        if i < 2*self.rows-1 and j < 2*self.cols-1 and A[i+1][j+1]:
                            self.AddToAllEdges(i*2*self.cols+j, (i+1)*2*self.cols+j+1, 1)
                        if i > 2*self.rows-1 and j > 0 and A[i+1][j-1]:
                            self.AddToAllEdges(i*2*self.cols+j, (i+1)*2*self.cols+j-1, 1)
                        if i > 0 and j < 2*self.cols-1 and A[i-1][j+1]:
                            self.AddToAllEdges(i*2*self.cols+j, (i-1)*2*self.cols+j+1, 1)

    def AddToAllEdges(self, _from: int, to: int, cost):
        self.allEdges.add(Edge(_from, to, cost))

        if (self.nodes[_from]) is None:
            self.nodes[_from] = set()

        self.nodes[_from].add(to)

        if (self.nodes[to]) is None:
            self.nodes[to] = set()

        self.nodes[to].add(_from)

    def RemoveTheAppropriateEdges(self):
        for i in range(self.MSTedges):
            e = self.MSTvector[i]
            maxN = max(e.src, e.dst)
            minN = min(e.src, e.dst)

            #print("Analyzing edge "+str(e.src)+" "+str(e.dst))

            if np.absolute(e.src - e.dst) == 1:
                alpha = (4*minN+3) - 2*(maxN % self.cols)
                eToRemove = Edge(alpha, alpha+2*self.cols, 1)
                eToRemoveMirr = Edge(alpha+2*self.cols, alpha, 1)
                eToRemove2 = Edge(alpha+1, alpha+1+2*self.cols, 1)
                eToRemove2Mirr = Edge(alpha+1+2*self.cols, alpha+1, 1)

            else:
                alpha = (4*minN+2*self.cols) - 2*(maxN % self.cols)
                eToRemove = Edge(alpha, alpha+1, 1)
                eToRemoveMirr = Edge(alpha+1, alpha, 1)
                eToRemove2 = Edge(alpha+2*self.cols, alpha+1+2*self.cols, 1)
                eToRemove2Mirr = Edge(alpha+1+2*self.cols, alpha+2*self.cols, 1)

            if eToRemove in self.allEdges:
                #print("removing "+str(eToRemove))
                self.SafeRemoveEdge(eToRemove)

            if eToRemoveMirr in self.allEdges:
                #print("removing "+str(eToRemoveMirr))

                self.SafeRemoveEdge(eToRemoveMirr)

            if eToRemove2 in self.allEdges:
                #print("removing "+str(eToRemove2))
                self.SafeRemoveEdge(eToRemove2)

            if eToRemove2Mirr in self.allEdges:
                #print("removing "+str(eToRemove2Mirr))
                self.SafeRemoveEdge(eToRemove2Mirr)

        self.AdditionalCleaningEdges()

    def SafeRemoveEdge(self, curEdge):

        try:
            self.allEdges.remove(curEdge)
            # successful removal from priority queue: allEdges
            if curEdge.dst in self.nodes[curEdge.src]:
                self.nodes[curEdge.src].remove(curEdge.dst)
            if curEdge.src in self.nodes[curEdge.dst]:
                self.nodes[curEdge.dst].remove(curEdge.src)

        except KeyError:
            # This is a serious problem
            print("TreeSet should have contained this element!!")
            sys.exit(1)


    def AdditionalCleaningEdges(self) : 
        #print("MST LIST HEEEEEERE") 

        #for u in self.MSTvector : 
        #    print(str(u))
        edge_to_remove = []
        for edge in self.allEdges : 
            print("1 : Cleaning edge ", edge)
            dst = edge.dst
            src = edge.src
            
            dst_4 = (dst // (2 * self.cols), dst % (2*self.cols) )
            src_4 = (src // (2* self.cols), src % (2*self.cols) ) 
            print("2 : Cleaning edge "+str(src_4)+" "+str(dst_4))

            dst_4 = int(np.floor( dst_4[0] * 0.5 ) * self.cols + np.floor(dst_4[1] * 0.5))
            src_4 = int(np.floor( src_4[0] * 0.5 ) *self.cols + np.floor(src_4[1] * 0.5))
            print("3 : Cleaning edge "+str(src_4)+" "+str(dst_4))
            if dst_4 != src_4 : 
                if (not isEdgein( src_4, dst_4, self.MSTvector))  and (not isEdgein( dst_4, src_4, self.MSTvector) ) : 

                    edge_to_remove.append(edge)

        #print("secondal edge read")
        #for u in self.allEdges : 
        #    print(str(u))
        
        for edge in edge_to_remove :
            self.SafeRemoveEdge(edge)
            print("removed edge ", edge)

        print("final edge read")
        for u in self.allEdges : 
            print(str(u))

        return
    



    def updateVisitedNodes(self, performed_paths) : 

        #edge = 20 *20 ; ex (16,18,17,18 )
        for edge in performed_paths : 
            cell_a = edge[0] * self.cols * 2 + edge[1]
            cell_b = edge[2] * self.cols * 2 + edge[3]

            if cell_a in self.nodes : 
                self.visited_nodes.append(cell_a)

            if cell_b in self.nodes : 
                self.visited_nodes.append(cell_b)

        print("VISITED NODES")
        print(self.visited_nodes)

    def CalculatePathsSequence(self, StartingNode, previous_offset = None):

        currentNode = StartingNode
        RemovedNodes = set()
        movement = []
        PathSequence = []

        movement.append(2*self.cols)
        movement.append(-1)
        movement.append(-2*self.cols)
        movement.append(1)

        found = False
        prevNode = 0

        if previous_offset!= None : 
            for idx in range(4):
                print("Starting node "+str(StartingNode))
                print("Trying offset "+str(idx + previous_offset))
                print("Checking node "+str(currentNode+movement[(idx+previous_offset) % 4] ))
                print((currentNode+movement[(idx+previous_offset) % 4] in list(self.nodes[currentNode])))
                if (currentNode+movement[(idx+previous_offset) % 4] in list(self.nodes[currentNode])) and ( not ( currentNode+movement[(idx+previous_offset) % 4] in self.visited_nodes)):

                    prevNode = currentNode + movement[(idx+previous_offset) % 4]
                    found = True
                    break
        else : 
            for idx in range(4):
                if (currentNode + movement[idx]) in list(self.nodes[currentNode]):
                    prevNode = currentNode + movement[idx]
                    found = True
                    break

        if not found:
            return None

        while True:
            if currentNode != StartingNode:
                RemovedNodes.add(currentNode)

            offset = movement.index(prevNode-currentNode)

            prevNode = currentNode

            found = False
            for idx in range(4):
                if (prevNode+movement[(idx+offset) % 4] in self.nodes[prevNode]) and not (prevNode+movement[(idx+offset) % 4] in RemovedNodes) and not (prevNode+movement[(idx+offset) % 4] in self.visited_nodes):

                    currentNode = prevNode + movement[(idx+offset) % 4]
                    found = True
                    break

            if not found:
                return offset

            if (prevNode in self.nodes[currentNode]):
                self.nodes[currentNode].remove(prevNode)

            if (currentNode in self.nodes[prevNode]):
                self.nodes[prevNode].remove(currentNode)

            i = int(currentNode/(2*self.cols))
            j = currentNode % (2*self.cols)
            previ = int(prevNode/(2*self.cols))
            prevj = prevNode % (2*self.cols)
            self.PathSequence.append((previ, prevj, i, j))


def isEdgein(src, dst, edge_list) : 
    #print("testing if "+str(src)+" "+str(dst)+" is in list")
    for edge_test in edge_list : 
        #print("ICI EDGE TEST" +str(edge_test))
        #print(edge)
        if src == edge_test.src and dst == edge_test.dst : 
            print(True)
            return True
    print(False)
    return False