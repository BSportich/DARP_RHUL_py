

from multiRobotPathPlanner import MultiRobotPathPlanner, Energy_MRPP, BuildMSTs, generate_instance_initial_random
from Visualization import visualize_paths
import numpy as np
import random as random
import Edges

class DARP_step : 
    def __init__(self, step, drones_energy, current_position, obstacle_pos, current_position_precise = []) :
        #pre resolution
        self.step = step 
        self.drones_energy = drones_energy
        self.current_position = current_position
        self.current_position_precise = current_position_precise
        self.obstacle_pos = obstacle_pos

        self.pre_covered_cells = []  #Not precise format
        self.full_covered_cells = [] #Not precise format

        #resolution data storage
        self.MRPP = None   #instance object containing DARP algorithm 
        self.DARP_results = None
        self.DARP_sucess = None 
        self.iterations = -1 

        #result to be used across steps
        self.subcell_asignment = None
        self.MSTs = None
        self.future_paths = None

        self.robots_offsets = None

        #Post resolution post execution
        self.performed_paths = []
        self.accumulated_paths = None

    def solve_step(self, nx, ny, visualization ) : 

        pre_covered_cells_encoded = []
        full_covered_cells_encoded = [] 

        print("full covered cells")
        print(self.full_covered_cells)
        #Encoding pre covered cells and full covered cells 
        if self.full_covered_cells != [] : 
            for r in range(len(self.drones_energy)) : 
                for fcc in self.full_covered_cells[r] : 
                    if fcc != [] : 
                        full_covered_cells_encoded.append( fcc[0] * ny + fcc[1] )

        print("pre covered cells")
        print(self.pre_covered_cells)
        if self.pre_covered_cells != [] : 
            for r in range(len(self.drones_energy)) : 
                pre_covered_cells_encoded_r = []
                for pcc in self.pre_covered_cells[r] : 
                    pre_covered_cells_encoded_r.append( pcc[0] * ny + pcc[1] )

                pre_covered_cells_encoded.append( pre_covered_cells_encoded_r )
                
            
             

        self.MRPP = Energy_MRPP(nx, ny, True, self.current_position, np.ones((1,len(self.drones_energy))) , (self.obstacle_pos + full_covered_cells_encoded), self.drones_energy , visualization, pre_covered_cells= pre_covered_cells_encoded )
        #self.MRPP = Energy_MRPP(nx, ny, True, self.current_position, np.ones((1,len(self.drones_energy))) , self.obstacle_pos, self.drones_energy , visualization )
        
        self.DARP_results, self.DARP_sucess, self.iterations = self.MRPP.divide()
        
        return self.DARP_results, self.DARP_sucess, self.iterations

        


class DARP_instance : 
    def __init__(self, nx, ny, start_positions, obstacles, cost_one_cell, strategy = 3, opt_ass_type = 2):
        self.rows = nx
        self.cols = ny
        self.obstacles_start = obstacles 
        self.cost_one_cell = cost_one_cell
        self.dronesNo = len(start_positions)
        self.start_positions = start_positions


        #parameters of the algorithm
        self.opt_ass_type = opt_ass_type #Normalized or non-normalized
        self.strategy =  strategy  #full replanning ? distance to robots or distance to MSTs

        self.DARP_steps = []
        self.createInitialStep()
    
    
    def createInitialStep(self) : 
        initial_step = DARP_step(0, [ ((nx * ny ) - len(self.obstacles_start)) / self.dronesNo] * self.dronesNo, self.start_positions, self.obstacles_start)

        self.DARP_steps.append( initial_step )


    def addStep(self, drones_energy, current_position, current_position_precise, obstacles_pos, rewrite_obstacle = False ) : 

        if rewrite_obstacle == True : 
                        new_step = DARP_step( len(self.DARP_steps), drones_energy, current_position, obstacles_pos )

        else : 
            new_step = DARP_step( len(self.DARP_steps), drones_energy, current_position, self.DARP_steps[-1].obstacle_pos + obstacles_pos, current_position_precise )
            print(self.DARP_steps[-1].obstacle_pos + obstacles_pos)
            print(obstacles_pos)
            input()

        print("New step created : "+str(drones_energy))
        self.DARP_steps.append( new_step)

    def generateNewStep(self, visualization = True) : 

        drones_pos = []
        drones_pos_precise = []
        drones_energy = []
        performed_paths = []
        robots_offsets = []
        
        for r in range(self.dronesNo) : 
            generation_factor = random.randint(0,10)
            consumption_factor = random.randint(80,120) / 100 
            new_pos = self.DARP_steps[-1].future_paths[r][generation_factor][2:]
            performed_paths.append( self.DARP_steps[-1].future_paths[r][:generation_factor+1] )

            r_offset = computeOffset( self.DARP_steps[-1].future_paths[r][:generation_factor+1] , self.cols )
            robots_offsets.append(r_offset)
            #Transform coordinates
            x, y = new_pos
            drones_pos_precise.append( (x,y) )
            #print("NEW POS "+str(new_pos))
            x , y = transformCoordinates(x,y)
            #print("NEW POS "+str( (x,y) ))
            new_pos_one = x * self.cols + y
            #print("NEW POS "+str(new_pos_one))

            drones_pos.append( new_pos_one )
            r_energy = self.DARP_steps[-1].drones_energy[r] - ( generation_factor * consumption_factor ) 
            if r_energy < 0 : 
                r_energy = 0
            drones_energy.append( r_energy )

        #blabla
        print(drones_pos)
        print(drones_energy)

        if visualization == True :
            image = visualize_paths(self.DARP_steps[-1].future_paths , self.DARP_steps[-1].subcell_assignment, self.dronesNo , self.DARP_steps[-1].DARP_results.color)
            
            print("START POSITIONS " +str(self.start_positions))
            print("CURRENT POSITIONS "+str(drones_pos_precise))
            print(performed_paths)
            #input()
            image.visualize_paths_with_pos("Combined Modes", self.start_positions, drones_pos_precise, self.cols, performed_paths )

        #performed path added to the PREVIOUS step
        self.DARP_steps[-1].performed_paths = performed_paths
        self.DARP_steps[-1].robots_offsets = robots_offsets
        self.addStep( drones_energy, drones_pos, drones_pos_precise, [])

        return performed_paths



    def divide_regions_last_step(self, vis = True) : 

        last_step = self.DARP_steps[-1]
        
        darp_results, success, iteration = last_step.solve_step(self.rows, self.cols, vis)

        return darp_results

    
    def solveMSTs_new(self, old_MSTs = [], robots_offsets_previous_step = []) :
        last_step = self.DARP_steps[-1] #actually current step = last insolved step

        if old_MSTs != [] : 
            best_case_paths, subcell_assignment, MSTs = BuildMSTs( last_step.MRPP,  self.start_positions , old_MSTs = old_MSTs, precise_positions= last_step.current_position_precise, r_offsets= robots_offsets_previous_step, full_covered_cells= last_step.full_covered_cells, accumulated_paths= self.DARP_steps[-2].accumulated_paths )
        else : 
            best_case_paths, subcell_assignment, MSTs = BuildMSTs( last_step.MRPP,  self.start_positions )


        return best_case_paths, subcell_assignment, MSTs

    def finalize_step(self, visualization = True) : 
        DARP_result = self.divide_regions_last_step()


        #if not step 0, insert pre-covered cells part of the path
        if self.DARP_steps[-1].step !=0 : 

            previous_predicted_paths = self.DARP_steps[-2].future_paths
            robots_offsets_previous_step = self.DARP_steps[-2].robots_offsets
            
            reducedMSTs = self.extractfromMST_robots( self.DARP_steps[-1].pre_covered_cells, self.DARP_steps[-1].full_covered_cells)

            future_paths, subcell_assignment, MSTs = self.solveMSTs_new(reducedMSTs, robots_offsets_previous_step)

            #Verification

            #VerificationPaths(previous_predicted_paths)


            
        
        else :

            future_paths, subcell_assignment, MSTs = self.solveMSTs_new()


        self.DARP_steps[-1].future_paths = future_paths #Not sure

        self.DARP_steps[-1].subcell_assignment = subcell_assignment
        self.DARP_steps[-1].MSTs = MSTs
        

        #visualize
        if visualization == True :
                image = visualize_paths(self.DARP_steps[-1].future_paths , subcell_assignment,
                                        self.dronesNo , DARP_result.color)
                
                image.visualize_paths_with_pos("Combined Modes", self.start_positions, self.DARP_steps[-1].current_position_precise, self.cols, self.DARP_steps[-1].performed_paths )

        #print results 

    def extractfromMST_robots( self, pre_covered_cells, full_covered_cells) : 
        reducedMSTs = []
        #print(self.DARP_steps[-2].MSTs)
        #print(pre_covered_cells)
        for r in range(self.dronesNo) : 
            reducedMST = extractfromMST( self.DARP_steps[-2].MSTs[r], pre_covered_cells[r],  full_covered_cells[r], self.cols)
            #print("REDUCED MST")
            #print("Core")
            #printMST(reducedMST[0])
            #print("Extension")
            #printMST(reducedMST[1])
            #print("ORIGINAL MST")
            #printMST(self.DARP_steps[-2].MSTs[r])
            reducedMSTs.append( reducedMST )
            #input()

        return reducedMSTs
    
    #To be executed after new step was created but before its resolution 
    def PreProcessSolveStep( self ) : 

        if len(self.DARP_steps) >= 2 : 

            accumulated_paths = []
            for r in range(self.dronesNo) : 
                accumulated_paths.append( [] )


            for i in range(2,  len(self.DARP_steps)) : 

                for r in range(self.dronesNo) : 
                    
                    performed_paths = self.DARP_steps[-i].performed_paths
                    accumulated_paths[r]= accumulated_paths[r] + performed_paths[r]
            
            print("accumulated performed paths")
            print(accumulated_paths)
            self.DARP_steps[-2].accumulated_paths = accumulated_paths
            input()
            #performed_paths = self.DARP_steps[-2].performed_paths #last step is the one not solved yet 

            #print(performed_paths)
            pre_covered_cells, full_covered_cells = sortCellsfromPaths_robots( accumulated_paths )

            for r in range(self.dronesNo) : 
                self.DARP_steps[-1].pre_covered_cells.append( pre_covered_cells[r] )
                self.DARP_steps[-1].full_covered_cells.append( full_covered_cells[r] )

            print( str(pre_covered_cells)+" "+str( full_covered_cells ))

    



def transformCoordinates(x,y) : 
     new_x = int( x/ 2)
     new_y = int (y / 2 )
     return new_x, new_y


def sortCellsfromPaths(performed_path) : 
    cells_dict = {}
    cells_dict_precise = {}
    non_covered_cells = []
    pre_covered_cells = []
    full_covered_cells = []

    for cell in performed_path : 
        cell_a = cell[:2] 
        cell_b = cell[2:]
        #print(cell_a)
        #print(cell_b)
        cell_a = transformCoordinates( cell_a[0], cell_a[1])
        cell_b = transformCoordinates( cell_b[0], cell_b[1] ) 

        if cell_a in cells_dict : 
            cells_dict[cell_a] = cells_dict[cell_a] +1
        else : 
            cells_dict[cell_a] = 1 

        if not cell_b in cells_dict : 
            cells_dict[cell_b] = 0 
    
    for cell in cells_dict :
        
        if cells_dict[cell] > 3 : 
            full_covered_cells.append(cell)
        else : 
            pre_covered_cells.append(cell)

    print(cells_dict)
    return pre_covered_cells, full_covered_cells

def sortCellsfromPaths_robots(performed_paths) : 
    pre_covered_cells = []
    full_covered_cells = []

    print(len(performed_paths))
    for r in range(len(performed_paths)) : 
        pre_covered_cells_r, full_covered_cells_r = sortCellsfromPaths(performed_paths[r])
        pre_covered_cells.append(pre_covered_cells_r)
        full_covered_cells.append(full_covered_cells_r)

    return pre_covered_cells, full_covered_cells



def extractfromMST( original_MST, pre_covered_cells, full_covered_cells, cols) : 

    reduced_MST_core = []
    reduced_MST_extension = []
    correct = True
    print(pre_covered_cells)
    for edge in original_MST : 
            print("EDGE IS "+str(edge), end=" ")
            src = (edge.src // cols, edge.src % cols)
            dst = (edge.dst // cols, edge.dst % cols)
            if src in pre_covered_cells and dst in pre_covered_cells : 

                reduced_MST_core.append( edge)
                print("added")
            
            elif src in pre_covered_cells or dst in pre_covered_cells : 
            
                reduced_MST_extension.append( edge )
                print("added")
            else : 
                print("not added")


    #JUST ADDED : IS IT WORKING ? 
    for edge in reduced_MST_core  : 
        src = (edge.src // cols, edge.src % cols)
        dst = (edge.dst // cols, edge.dst % cols)
        if src in full_covered_cells or dst in full_covered_cells :

            reduced_MST_core.remove(edge) 
            print("EDGE IS "+str(edge)+" removed")


    for edge in reduced_MST_extension : 
        src = (edge.src // cols, edge.src % cols)
        dst = (edge.dst // cols, edge.dst % cols)
        if src in full_covered_cells or dst in full_covered_cells :

            reduced_MST_extension.remove(edge) 
            print("EDGE IS "+str(edge)+" removed")


    return (reduced_MST_core, reduced_MST_extension)








def printMST(MST) : 
    for edge in MST : 
        print(str(edge))
    
    return


def computeOffset( performed_paths, cols ) :
    last_cell = performed_paths[-1] 
    print(performed_paths)
    print(last_cell)
    currentNode = last_cell[0] * 2 * cols + last_cell[1]
    next_node = last_cell[2] * 2 * cols + last_cell[3]

    movement = []
    movement.append(2*cols)
    movement.append(-1)
    movement.append(-2*cols)
    movement.append(1)
    
    for idx in range(4):
        if ((currentNode + movement[idx]) == next_node ):
            return idx
            
    print(currentNode)
    print(next_node)
    print(performed_paths)
    exit(1)


if __name__ == '__main__':
    nx, ny, dronesNo, initial_positions, obs_pos, drones_energy = generate_instance_initial_random(10,10,3)

    print("Instance is "+str(drones_energy))
    print ("Energy \t", " Initial Positions \t", " Obstacles Positions \t")
    print( (drones_energy, initial_positions, obs_pos) )
    DARP_instance_obj = DARP_instance(nx, ny, initial_positions, obs_pos, 1 )
    DARP_instance_obj.createInitialStep()
    DARP_instance_obj.finalize_step()
    while True : 
        input()
        performed_paths = DARP_instance_obj.generateNewStep()

        DARP_instance_obj.PreProcessSolveStep()
    
        DARP_instance_obj.finalize_step()

                

