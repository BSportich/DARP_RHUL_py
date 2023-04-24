import numpy as np
import sys
import random as random
import cv2

#Tarjan algorithm to identify cut-vertex
#Evaluate recursively all the cells of the grid starting from the current_cell
#All the bridge cells are put in the bridge_cells structure
def BridgeUtil(gridcells, rows, cols, current_cell, visited, parent, low, depth, bridge_cells, d) : 

    visited[ current_cell[0] ][ current_cell[1] ] = True
    depth[ current_cell[0] ][ current_cell[1] ] = d
    low[ current_cell[0] ][ current_cell[1] ] = d
    child_count = 0 
    isBridge= False

    neighbours = getNeighbours(gridcells, current_cell, rows, cols)
    for neighbour in neighbours : 

        if visited[ neighbour[0] ][ neighbour[1] ] == False :

            parent[ neighbour ] = current_cell
            
            BridgeUtil(gridcells, rows, cols, neighbour, visited, parent, low, depth, bridge_cells, d+1)
            child_count = child_count + 1

            if low[ neighbour[0] ][ neighbour[1] ] >= depth[ current_cell[0] ][ current_cell[1] ] :
                isBridge = True

            low[ current_cell[0] ][ current_cell[1] ] = min( low[ current_cell[0] ][ current_cell[1] ], low[ neighbour[0] ][ neighbour[1] ])

        elif parent[ current_cell ] != neighbour : 
            low[ current_cell[0] ][ current_cell[1] ] = min( low[ current_cell[0] ][ current_cell[1] ], depth[ neighbour[0] ][ neighbour[1] ])

    if (current_cell in parent.keys() and isBridge) or (( not ( current_cell in parent.keys() ) ) and child_count > 1 ) : 
        bridge_cells.append(current_cell)



#Get list of cut-vertex / bridge cells in the grid
#Special case of the starting cell (removed from the bridge list) ; might to be put back again to avoid rejection of this cell
def FindBridgeCells(gridcells, rows, cols, current_cell) : 


    #print(gridcells)
    #print(rows)
    #print(cols)
    #print(current_cell)
    bridge_cells = []
    parent = {}
    parent[current_cell] = None 
    visited = np.zeros((rows, cols))
    depth = np.zeros((rows, cols))
    low = np.zeros((rows, cols))

    d = 0 

    BridgeUtil(gridcells, rows, cols, current_cell, visited, parent, low, depth, bridge_cells, d)

    grid = np.copy(gridcells)
    mask = np.where(grid == 1)
    grid[mask[0], mask[1]] = 255
    image = np.uint8(grid)
    original_num_labels, labels_im = cv2.connectedComponents(image, connectivity=4)

    grid = np.copy(gridcells)
    grid [current_cell[0]][current_cell[1]] = 0 
    mask = np.where(grid == 1)
    grid[mask[0], mask[1]] = 255
    image = np.uint8(grid)
    num_labels, labels_im = cv2.connectedComponents(image, connectivity=4)

    if num_labels <= original_num_labels : 
        
        if not ( current_cell in bridge_cells) : 
            print("WATCH OUT : STARTING CELL NOT IN LIST")
            print(gridcells)
            print(bridge_cells)
            #print( current_cell )
        else :
            bridge_cells.remove( current_cell )



    return bridge_cells, depth, low


#Among the non bridge/cut-vertex cells, select cells to reject
#Cells are selected among two criterias : 1) valuation 2) degrees
#Cells with lowest valuation and lowest degrees are the ones selected for rejection (in that order)
#Valuation of cut vertex cells is put to maximum value to prevent them of being selected
def findCelltoReject(grid, value_grid, rows, cols, bridges, starting_cell) : 

    value_copy = np.copy(value_grid)

    for i in range(rows) : 
        for j in range(cols) : 
            if value_grid[i][j] >0 and grid[i][j] > 0: 
                degree = getNeighbours( grid ,(i,j), rows, cols)

                value_copy[i][j] = value_copy[i][j] + ( len(degree) / 100.0 ) 
            else :

                value_copy[i][j] = 255

    for bridge in bridges : 
        value_copy[ bridge[0] ][ bridge[1] ] = 255
    value_copy[ starting_cell[0] ][ starting_cell[1] ] = 255

    min_value = np.min(value_copy)
    for i in range(rows) : 
        for j in range(cols) : 
            if value_copy[i][j] == min_value : 
                return (i,j)
            
    print("NO NON-BRIDGE CELLS FOUND TO BE REJECTED")
    

    return (-1,-1)

#Reject enough cells for the cell count to be <= max_nb_cells
def RejectionProcess(binarymap, value_grid, rows, cols, max_nb_cells, starting_cell, precovered_cells) : 

    current_cell_count = np.sum(binarymap)
    grid_copy = np.copy(binarymap)
    value_copy = np.copy(value_grid)
    rejected_cells = []
    rejected_value = 0 

    if max_nb_cells == 0 :
        #print("here")
        #exit(1)
        grid_0_cells = np.zeros(( rows, cols ))
        return [], grid_0_cells, -99999999
        


    while current_cell_count > max_nb_cells : 

        bridges, depth, low = FindBridgeCells(grid_copy, rows, cols, starting_cell)

        cell = findCelltoReject(grid_copy, value_copy, rows, cols, (bridges + precovered_cells) , starting_cell)

        if cell == starting_cell : 
            print("STARTING CELL CAN NOT BE REJECTED")
            exit(1)

        grid_copy[ cell[0] ][ cell[1] ] = 0 
        value_copy[ cell[0] ][ cell[1] ] = 0
        rejected_cells.append( cell )
        rejected_value = rejected_value + value_grid[ cell[0] ][ cell[1] ]
        current_cell_count = current_cell_count - 1 

    return rejected_cells, grid_copy, rejected_value

def EvaluateRejection() : 
    
    num_labels = 100

    while num_labels > 2  : 
        grid, value = GenerateInstanceTest()
        grid3 = np.copy(grid)
        mask = np.where(grid3 == 1)
        grid3[mask[0], mask[1]] = 255
        image = np.uint8(grid3)
        num_labels, labels_im = cv2.connectedComponents(image, connectivity=4)


    nb_cells = np.sum(grid)
    starting_cell = ( random.randint(0,9), random.randint(0,9))
    while grid[ starting_cell[0] ][ starting_cell[1] ] != 1 :
        starting_cell = ( random.randint(0,9), random.randint(0,9))
    print(grid)
    print(starting_cell)
    print(nb_cells)
    bridges, depth, low = FindBridgeCells(grid, 10, 10, starting_cell)

    rejected_cells, grid_copy, rejected_value = RejectionProcess(grid, value, 10,10, nb_cells/2, starting_cell)
    print(rejected_cells)
    print(value)
    for cell in rejected_cells : 
        print(cell)
        print(value[ cell[0] ][ cell[1] ])
        



#RejectionProcess(grid, value, 10,10, nb_cells/2, starting_cell )

def GenerateInstanceTest() : 

    grid_cells = np.ones((10,10))
    value_grid = np.zeros((10,10))
    count_value = 1
    for i in range(10) : 
        for j in range(10) : 

            value_grid[i][j] = count_value
            grid_cells[i][j] = random.random()

            if grid_cells[i][j] < 0.3 :
                grid_cells[i][j] = 0
                value_grid[i][j] = 0
            else : 
                grid_cells[i][j] = 1
            
            count_value = count_value +1 

    return grid_cells, value_grid


def EvaluationQuality(starting_cell):
    grid, values = GenerateInstanceTest()
    
    if grid[ starting_cell[0] ][ starting_cell[1] ] == 0 or len(getNeighbours(grid, starting_cell , 10,10)) == 0:
        return False, 0

    #print(grid)

    bridges, depth, low = FindBridgeCells(grid,10,10, starting_cell )
    #print(bridges)
    #grid2 = np.copy(grid)
    #for cell in bridges : 
    #    grid2[ cell[0]][cell[1]] = grid2[ cell[0]][cell[1]] +100
    #print(grid2)


    #print(depth)
    #print(low)

    grid3 = np.copy(grid)
    mask = np.where(grid3 == 1)
    grid3[mask[0], mask[1]] = 255
    image = np.uint8(grid3)
    original_num_labels, original_labels_im = cv2.connectedComponents(image, connectivity=4)

    correct_bridge_cnt = 0 
    for cell in bridges : 

        grid3 = np.copy(grid)
        grid3 [cell[0]][cell[1]] = 0 
        mask = np.where(grid3 == 1)
        grid3[mask[0], mask[1]] = 255
        image = np.uint8(grid3)
        num_labels, labels_im = cv2.connectedComponents(image, connectivity=4)

        if num_labels > original_num_labels : 
            correct_bridge_cnt = correct_bridge_cnt + 1
        else : 
            print("INCORRECT "+str(cell))
            print(grid)
    
    non_bridges = []
    for i in range(10) : 
        for j in range (10) : 
            if original_labels_im[i][j] == original_labels_im[ starting_cell[0] ][ starting_cell[1] ] and (not ( (i,j) in bridges )) : 
                non_bridges.append( (i,j) ) 

    for cell in non_bridges : 

        grid3 = np.copy(grid)
        grid3 [cell[0]][cell[1]] = 0 
        mask = np.where(grid3 == 1)
        grid3[mask[0], mask[1]] = 255
        image = np.uint8(grid3)
        num_labels, labels_im = cv2.connectedComponents(image, connectivity=4)

        if num_labels > original_num_labels : 
            print("MISSED BRIDGE ON THE GRAPH "+str(cell))
            print(starting_cell)
            print(original_labels_im)
            print(labels_im)
            print(bridges)
            print(grid)

    #print(grid)
    print("Out of "+str(len(bridges))+" "+str(correct_bridge_cnt)+" are correct")
    if len(bridges) == 0 :
        return True, 1
    return True, ( len(bridges) / correct_bridge_cnt )
    
def ReviewQuality(n) : 

    i = 0 
    sum_tot = 0 
    while i < n : 
        starting_cell = ( random.randint(0,9), random.randint(0,9))
        Test, coverage = EvaluationQuality(starting_cell)
        if Test == True : 
            i = i + 1 
            sum_tot = sum_tot + coverage
    
    print("Correct bridges found "+str( (sum_tot/n) * 100 ) +" %")


#Get neighbour cells in a grid in 4 directions
def getNeighbours(BinaryMap, position, rows, cols) : 
    Neighbours = []
    x = position[0]
    y = position[1]

    if (x-1) >= 0 and BinaryMap[x-1][y] == 1 : 
        Neighbours.append( (x-1,y) ) 

    if (y-1) >= 0 and BinaryMap[x][y-1] == 1 : 
        Neighbours.append( (x,y-1) )

    if (x+1) < rows and BinaryMap[x+1][y] == 1 : 
        Neighbours.append( (x+1,y) )  

    if (y+1) < cols and BinaryMap[x][y+1] == 1 : 
        Neighbours.append( (x,y+1) )

    return Neighbours


    