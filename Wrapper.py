

from multiRobotPathPlanner import MultiRobotPathPlanner, Energy_MRPP, BuildMSTs, generate_instance_initial_random
from Visualization import visualize_paths
import numpy as np
import random as random

class DARP_step : 
    def __init__(self, step, drones_energy, current_position, obstacle_pos, ) :
        self.step = step 
        self.drones_energy = drones_energy
        self.current_position = current_position
        self.obstacle_pos = obstacle_pos

        self.MRPP = None
        self.DARP_results = None
        self.DARP_sucess = None 
        self.iterations = -1 

        self.MSTs = None

    def solve_step(self, nx, ny, visualization ) : 

        self.MRPP = Energy_MRPP(nx, ny, True, self.current_position, np.ones((1,len(self.drones_energy))) , self.obstacle_pos, self.drones_energy , visualization )
        
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


    def addStep(self, drones_energy, current_position, obstacles_pos, rewrite_obstacle = False ) : 

        if rewrite_obstacle == True : 
                        new_step = DARP_step( len(self.DARP_steps), drones_energy, current_position, obstacles_pos )

        else : 
            new_step = DARP_step( len(self.DARP_steps), drones_energy, current_position, self.DARP_steps[-1].obstacle_pos + obstacles_pos )

        print("New step created : "+str(drones_energy))
        self.DARP_steps.append( new_step)

    def generateNewStep(self) : 

        drones_pos = []
        drones_energy = []
        for r in range(self.dronesNo) : 
            generation_factor = random.randint(0,10)
            consumption_factor = random.randint(80,120) / 100 
            new_pos = self.DARP_steps[-1].MST[r][generation_factor][2:]
            drones_pos.append( new_pos )
            r_energy = self.DARP_steps[-1].drones_energy[r] - ( generation_factor * consumption_factor ) 
            if r_energy < 0 : 
                r_energy = 0
            drones_energy.append( r_energy )

        #blabla
        print(drones_pos)
        self.addStep( drones_energy, drones_pos, [])



    def divide_regions_last_step(self, vis = True) : 

        last_step = self.DARP_steps[-1]
        
        darp_results, success, iteration = last_step.solve_step(self.rows, self.cols, vis)

        return darp_results

    def solveMSTs(self) : 

        filtered_old_MSTs = []
        new_MSTs = []

        #Extracting remaining path from the MST of the last step
        old_MSTs = self.DARP_steps[-2]
        for r in range(self.dronesNo) : 
            for i in range(old_MSTs[r]) : 
                if old_MSTs[r][i][0] == self.DARP_steps[-1].current_positions[r][0] and old_MSTs[r][i][1] == self.DARP_steps[-1].current_positions[r][1] :
                    filtered_old_MSTs.append( old_MSTs[r][i:] ) 

        
        #Computing new MSTs

        #Remove pre covered cells of assigned cells
        cells_allocation = self.DARP_steps[-1].DARP_results.corrected_cell_assignment
        old_cells_allocation = self.DARP_steps[-2].DARP_results.corrected_cell_assignment




        #Inserting new MSTs
    
    def solveMSTs_new(self) :
        last_step = self.DARP_steps[-1]

        best_case_paths, subcell_assignment = BuildMSTs( last_step.MRPP )

        return best_case_paths, subcell_assignment

    def finalize_step(self, visualization = True) : 
        DARP_result = self.divide_regions_last_step()
        MST_paths, subcell_assignment = self.solveMSTs_new()


        #if not step 0, insert pre-covered cells part of the path
        if self.DARP_steps[-1].step !=0 : 

            previousMST_paths = self.DARP_steps[-2].MSTs
            mergedMST_paths = mergeMSTs(MST_paths, previousMST_paths, self.dronesNo)
            self.DARP_steps[-1].MST = mergedMST_paths
        
        else :

            self.DARP_steps[-1].MST = MST_paths


        #visualize
        if visualization == True :
                image = visualize_paths(self.DARP_steps[-1].MST , subcell_assignment,
                                        self.dronesNo , DARP_result.color)
                image.visualize_paths("Combined Modes")

        #print results 



def mergeMSTs( MST_paths, previousMST_paths, number_robots) : 
    full_MST = []
    for r in range(number_robots) : 

        bridge_cell = MST_paths[r][-1][1]
        for i in range(len(previousMST_paths[r])) : 

            if previousMST_paths[r][i][1] == bridge_cell :

                full_MST.append( ( MST_paths[r] + previousMST_paths[r][i+1:]) )
                break
    
    return full_MST



if __name__ == '__main__':
    nx, ny, dronesNo, initial_positions, obs_pos, drones_energy = generate_instance_initial_random(10,10,3)

    print("Instance is "+str(drones_energy))
    print ("Energy \t", " Initial Positions \t", " Obstacles Positions \t")
    print( (drones_energy, initial_positions, obs_pos) )
    DARP_instance_obj = DARP_instance(nx, ny, initial_positions, obs_pos, 1 )
    DARP_instance_obj.createInitialStep()
    DARP_instance_obj.finalize_step()
    input()
    DARP_instance_obj.generateNewStep()
    DARP_instance_obj.finalize_step()

                

