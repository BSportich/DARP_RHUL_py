import pickle

from darp import DARP
import numpy as np
from kruskal import Kruskal
from CalculateTrajectories import CalculateTrajectories
from Visualization import visualize_paths
import sys
import argparse
from turns import turns
from PIL import Image
import time
import cv2
import random as random
import csv

def get_area_map(path, area=0, obs=-1):
    """
    Creates an array from a given png-image(path).
    :param path: path to the png-image
    :param area: non-obstacles tiles value; standard is 0
    :param obs: obstacle tiles value; standard is -1
    :return: an array of area(0) and obstacle(-1) tiles
    """
    le_map = np.array(Image.open(path))
    ma = np.array(le_map).mean(axis=2) != 0
    le_map = np.int8(np.zeros(ma.shape))
    le_map[ma] = area
    le_map[~ma] = obs
    return le_map

def get_area_indices(area, value, inv=False, obstacle=-1):
    """
    Returns area tiles indices that have value
    If inv(erted), returns indices that don't have value
    :param area: array with value and obstacle tiles
    :param value: searched tiles with value
    :param inv: if True: search will be inverted and index of non-value tiles will get returned
    :param obstacle: defines obstacle tiles
    :return:
    """
    try:
        value = int(value)
        if inv:
            return np.concatenate([np.where((area != value))]).T
        return np.concatenate([np.where((area == value))]).T
    except:
        mask = area == value[0]
        if inv:
            mask = area != value[0]
        for v in value[1:]:
            if inv:
                mask &= area != v
            else:
                mask |= area == v
        mask &= area != obstacle
        return np.concatenate([np.where(mask)]).T

def PositionTransformer(x,y, rows, cols): 
    return (x * cols + y )

def CollectData(DARP_instance, success, iteration) : 
    data_table = []

    #Convergence data
    convergence_status = success
    print(convergence_status)
    if convergence_status == False :
        return [ convergence_status, iteration, -1, DARP_instance.drones_energy , DARP_instance.initial_positions, DARP_instance.obstacles_positions, DARP_instance.DesireableAssign, {}, 0]
    nb_iterations = iteration

    #Instance info
    robot_energy = DARP_instance.drones_energy 
    starting_pos = DARP_instance.initial_positions
    obstacles = DARP_instance.obstacles_positions

    #Quality of solution
    desired_assignment = DARP_instance.DesireableAssign
    final_assignment = DARP_instance.corrected_cell_assignment
    print(final_assignment)
    assignment_value = 0


            

    #Rejection Values - Is it necessary ?

    #Max instance data
    print(robot_energy)
    max_instance_coverage = np.sum(robot_energy)
    value_grid = DARP_instance.valuation_grid
    max_instance_value = 0
    reached_assignment = []
    for r in range(DARP_instance.droneNo) : 
        assignment_value = assignment_value + np.sum(final_assignment[r])
        reached_assignment.append( np.sum(final_assignment[r]))
        for cell in final_assignment[r] : 
            max_instance_value = max_instance_value + value_grid[ cell[0] ][ cell[1] ]


    #value_performance = (max_instance_value - assignment_value) / max_instance_value)  
    print(max_instance_coverage)
    print(assignment_value)
    coverage_performance = (max_instance_coverage - assignment_value) / max_instance_coverage
    
    #Save image

    data_table = [ convergence_status, nb_iterations, coverage_performance, desired_assignment, reached_assignment, assignment_value, final_assignment, max_instance_value]
    print(data_table)
    return data_table

def Experiments(number, nb_robots) : 

    with open('experiments_review', mode='w') as csv_file:
        experiments_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        for r in range(nb_robots,nb_robots+1) : 

            total_iterations_normalized = 0 
            total_errors_normalized = 0 

            total_iterations_non_normalized = 0 
            total_errors_non_normalized = 0 

            for i in range(number) : 
                nx, ny, dronesNo, initial_positions, obs_pos, drones_energy  = generate_instance_initial_random(10,10,r)

                print("Instance is "+str(drones_energy))
                experiments_writer.writerow( ("Energy \t", " Initial Positions \t", " Obstacles Positions \t"))
                experiments_writer.writerow( (drones_energy, initial_positions, obs_pos) )

                valuation_grid = createValuationGrid(nx, ny, obs_pos)

                visualization = False
                notEqualPortions = True
                portions = np.ones((1,r))
                DARP_energy = True

                #Normalized
                darp = DARP(nx, ny, notEqualPortions , initial_positions, portions, obs_pos, visualization,
                                            DARP_energy, opt_ass_type=1, drones_energy=drones_energy, 
                                            valuation_grid = valuation_grid,
                                            MaxIter=80000, CCvariation=0.01,
                                            randomLevel=0.0001, dcells=2, importance=False)

                # Divide areas based on robots initial positions
                success, iteration = darp.divideRegions()

                if success : 
                    total_iterations_normalized = total_iterations_normalized + iteration
                else : 
                    total_iterations_normalized = total_iterations_normalized + 80000
                    total_errors_normalized = total_errors_normalized +1



                data_table  = CollectData(darp, success, iteration)

                verif_data = all(x == y for x, y in zip(drones_energy, data_table[-4]))
                if not verif_data : 
                    experiments_writer.writerow(("ERROR : ENERGY DOES NOT FIT ASSIGNMENT"))
                if np.sum(drones_energy) != np.sum(data_table[-3]) : 
                    experiments_writer.writerow(("ERROR : ENERGY DOES NOT FIT ASSIGNMENT"))


                map_found_1 = data_table[0]
                map_final_1 = data_table[-2]
                experiments_writer.writerow( (data_table[:-2], data_table[-1]) )
                
                #Not normalized
                darp = DARP(nx, ny, notEqualPortions , initial_positions, portions, obs_pos, visualization,
                                            DARP_energy=True, opt_ass_type=2, drones_energy=drones_energy, opt_threshold = "C",
                                            valuation_grid = valuation_grid,
                                            MaxIter=80000, CCvariation=0.01,
                                            randomLevel=0.0001, dcells=2, importance=False)

                # Divide areas based on robots initial positions
                success, iteration = darp.divideRegions()

                if success : 
                    total_iterations_non_normalized = total_iterations_non_normalized + iteration
                else : 
                    total_iterations_non_normalized = total_iterations_non_normalized + 80000
                    total_errors_non_normalized = total_errors_non_normalized +1

                data_table = CollectData(darp, success, iteration)
                map_final_2 = data_table[-2]
                map_found_2 = data_table[0]
                experiments_writer.writerow( (data_table[:-2], data_table[-1]) )
                if data_table[2] > 0 :
                    experiments_writer.writerow(("ERROR"))

                verif_data = all(x == y for x, y in zip(drones_energy, data_table[-4]))
                if not verif_data : 
                    experiments_writer.writerow(("ERROR : ENERGY DOES NOT FIT ASSIGNMENT"))
                if np.sum(drones_energy) != np.sum(data_table[-4]) : 
                    experiments_writer.writerow(("ERROR : ENERGY DOES NOT FIT ASSIGNMENT"))
                
                if map_found_1 : 
                    experiments_writer.writerow( (map_final_1) )
                if map_found_2: 
                    experiments_writer.writerow( (map_final_2) )

            experiments_writer.writerow(("ITERATIONS NORMALIZED"+str(total_iterations_normalized/number)))
            experiments_writer.writerow(("ITERATIONS NON NORMALIZED"+str(total_iterations_non_normalized/number)))

            experiments_writer.writerow(("ERRORS NORMALIZED"+str(total_errors_normalized)))
            experiments_writer.writerow(("ERRORS NON NORMALIZED"+str(total_errors_non_normalized)))



    return


def generate_instance_initial_random(rows, cols, nb_robots, full_random = False) : 

    if full_random == True : 
        rows = np.random.randint(10,20)
        cols = np.random.randint(10,20)
        nb_robots = np.random.randint(3,7)

    nb_obstacles =  np.random.randint(3,7)
    #nb_obstacles = 30


    num_labels = 3

    while num_labels > 2 : 

        obstacle_list = []
        robots_start_pos_list = []

        x = np.random.randint(0,rows)
        y = np.random.randint(0, cols)

        robots_start_pos_list.append( PositionTransformer(x,y,rows,cols) )
        count = nb_robots - 1 + nb_obstacles

        while count > 0 : 

            temp_grid = np.ones((rows,cols))
            
            while PositionTransformer(x,y, rows, cols) in (robots_start_pos_list + obstacle_list) :

                x = np.random.randint(0,rows)
                y = np.random.randint(0, cols)

            if count <= nb_obstacles : 
                obstacle_list.append( PositionTransformer(x,y,rows, cols))
            else :
                robots_start_pos_list.append( PositionTransformer(x,y, rows, cols) )

            count = count -1

        #print(obstacle_list)
        for cell in obstacle_list : 
            temp_grid[ cell//cols ][ cell%cols ] = 0 
        
        mask = np.where(temp_grid == 1)
        temp_grid[mask[0], mask[1]] = 255
        image = np.uint8(temp_grid)
        num_labels, labels_im = cv2.connectedComponents(image, connectivity=4)

        #print(num_labels)
    #print(temp_grid)

    #Generate energy situation
    robot_energy = []
    for r in range(nb_robots) : 
        robot_energy.append(  random.randint(5, np.floor( 100/nb_robots) ) )


    return rows, cols, nb_robots, robots_start_pos_list, obstacle_list, robot_energy

def createValuationGrid(rows, cols, obstacle_list):

    valuation_grid = np.random.randint(1, 5, size=(rows, cols))

    for cell in obstacle_list : 
        valuation_grid[ cell//cols ][ cell%cols ] = 0 

    return valuation_grid

class Energy_MRPP(DARP):
    def __init__(self, nx, ny, notEqualPortions, initial_positions, portions,
                 obs_pos, drones_energy, visualization, MaxIter=80000, CCvariation=0.01,
                 randomLevel=0.0001, dcells=2, importance=False):

        self.start_time = time.time()
        # Initialize DARP
        self.darp_instance = DARP(nx, ny, notEqualPortions, initial_positions, portions, obs_pos, visualization,
                                    DARP_energy=True, drones_energy = drones_energy, 
                                  MaxIter=MaxIter, CCvariation=CCvariation,
                                  randomLevel=randomLevel, dcells=dcells,
                                  importance=importance)


    def divide(self) :
        # Divide areas based on robots initial positions
        self.DARP_success , self.iterations = self.darp_instance.divideRegions()
        return self.darp_instance, self.DARP_success, self.iterations


class MultiRobotPathPlanner(DARP):
    def __init__(self, nx, ny, notEqualPortions, initial_positions, portions,
                 obs_pos, visualization, drones_energy = [], MaxIter=80000, CCvariation=0.01,
                 randomLevel=0.0001, dcells=2, importance=False):

        energy_mode = False
        if drones_energy != [] : 
            energy_mode = True
        start_time = time.time()
        # Initialize DARP
        self.darp_instance = DARP(nx, ny, notEqualPortions, initial_positions, portions, obs_pos, visualization,
                                    DARP_energy= energy_mode, drones_energy=drones_energy, 
                                  MaxIter=MaxIter, CCvariation=CCvariation,
                                  randomLevel=randomLevel, dcells=dcells,
                                  importance=importance)

        # Divide areas based on robots initial positions
        self.DARP_success , self.iterations = self.darp_instance.divideRegions()

        # Check if solution was found
        if not self.DARP_success:
            print("DARP did not manage to find a solution for the given configuration!")
        else:
            # Iterate for 4 different ways to join edges in MST
            self.mode_to_drone_turns = []
            AllRealPaths_dict = {}
            subCellsAssignment_dict = {}
            for mode in range(4):
                MSTs = calculateMSTs(self.darp_instance.BinaryRobotRegions, self.darp_instance.droneNo, self.darp_instance.rows, self.darp_instance.cols, mode)
                AllRealPaths = []
                for r in range(self.darp_instance.droneNo):
                    ct = CalculateTrajectories(self.darp_instance.rows, self.darp_instance.cols, MSTs[r])
                    ct.initializeGraph(CalcRealBinaryReg(self.darp_instance.BinaryRobotRegions[r], self.darp_instance.rows, self.darp_instance.cols), True)
                    ct.RemoveTheAppropriateEdges()
                    ct.CalculatePathsSequence(4 * self.darp_instance.initial_positions[r][0] * self.darp_instance.cols + 2 * self.darp_instance.initial_positions[r][1])
                    AllRealPaths.append(ct.PathSequence)

                self.TypesOfLines = np.zeros((self.darp_instance.rows*2, self.darp_instance.cols*2, 2))
                for r in range(self.darp_instance.droneNo):
                    flag = False
                    for connection in AllRealPaths[r]:
                        if flag:
                            if self.TypesOfLines[connection[0]][connection[1]][0] == 0:
                                indxadd1 = 0
                            else:
                                indxadd1 = 1

                            if self.TypesOfLines[connection[2]][connection[3]][0] == 0 and flag:
                                indxadd2 = 0
                            else:
                                indxadd2 = 1
                        else:
                            if not (self.TypesOfLines[connection[0]][connection[1]][0] == 0):
                                indxadd1 = 0
                            else:
                                indxadd1 = 1
                            if not (self.TypesOfLines[connection[2]][connection[3]][0] == 0 and flag):
                                indxadd2 = 0
                            else:
                                indxadd2 = 1

                        flag = True
                        if connection[0] == connection[2]:
                            if connection[1] > connection[3]:
                                self.TypesOfLines[connection[0]][connection[1]][indxadd1] = 2
                                self.TypesOfLines[connection[2]][connection[3]][indxadd2] = 3
                            else:
                                self.TypesOfLines[connection[0]][connection[1]][indxadd1] = 3
                                self.TypesOfLines[connection[2]][connection[3]][indxadd2] = 2

                        else:
                            if (connection[0] > connection[2]):
                                self.TypesOfLines[connection[0]][connection[1]][indxadd1] = 1
                                self.TypesOfLines[connection[2]][connection[3]][indxadd2] = 4
                            else:
                                self.TypesOfLines[connection[0]][connection[1]][indxadd1] = 4
                                self.TypesOfLines[connection[2]][connection[3]][indxadd2] = 1

                subCellsAssignment = np.zeros((2*self.darp_instance.rows, 2*self.darp_instance.cols))
                for i in range(self.darp_instance.rows):
                    for j in range(self.darp_instance.cols):
                        subCellsAssignment[2 * i][2 * j] = self.darp_instance.A[i][j]
                        subCellsAssignment[2 * i + 1][2 * j] = self.darp_instance.A[i][j]
                        subCellsAssignment[2 * i][2 * j + 1] = self.darp_instance.A[i][j]
                        subCellsAssignment[2 * i + 1][2 * j + 1] = self.darp_instance.A[i][j]

                drone_turns = turns(AllRealPaths)
                drone_turns.count_turns()
                drone_turns.find_avg_and_std()
                self.mode_to_drone_turns.append(drone_turns)

                AllRealPaths_dict[mode] = AllRealPaths
                subCellsAssignment_dict[mode] = subCellsAssignment


            # Find mode with the smaller number of turns
            averge_turns = [x.avg for x in self.mode_to_drone_turns]
            self.min_mode = averge_turns.index(min(averge_turns))
            
            # Retrieve number of cells per robot for the configuration with the smaller number of turns
            min_mode_num_paths = [len(x) for x in AllRealPaths_dict[self.min_mode]]
            min_mode_returnPaths = AllRealPaths_dict[self.min_mode]

            # Uncomment if you want to visualize all available modes
            
            # if self.darp_instance.visualization:
            #     for mode in range(4):
            #         image = visualize_paths(AllRealPaths_dict[mode], subCellsAssignment_dict[mode],
            #                                 self.darp_instance.droneNo, self.darp_instance.color)
            #         image.visualize_paths(mode)
            #     print("Best Mode:", self.min_mode)

            #Combine all modes to get one mode with the least available turns for each drone
            combined_modes_paths = []
            combined_modes_turns = []
            
            for r in range(self.darp_instance.droneNo):
                min_turns = sys.maxsize
                temp_path = []
                for mode in range(4):
                    if self.mode_to_drone_turns[mode].turns[r] < min_turns:
                        temp_path = self.mode_to_drone_turns[mode].paths[r]
                        min_turns = self.mode_to_drone_turns[mode].turns[r]
                combined_modes_paths.append(temp_path)
                combined_modes_turns.append(min_turns)

            self.best_case = turns(combined_modes_paths)
            self.best_case.turns = combined_modes_turns
            self.best_case.find_avg_and_std()
            
            # Retrieve number of cells per robot for the best case configuration
            best_case_num_paths = [len(x) for x in self.best_case.paths]
            best_case_returnPaths = self.best_case.paths
            
            #visualize best case
            if self.darp_instance.visualization:
                image = visualize_paths(self.best_case.paths, subCellsAssignment_dict[self.min_mode],
                                        self.darp_instance.droneNo, self.darp_instance.color)
                image.visualize_paths("Combined Modes")

            self.execution_time = time.time() - start_time
            
            print(f'\nResults:')
            print(f'Number of cells per robot: {best_case_num_paths}')
            print(f'Minimum number of cells in robots paths: {min(best_case_num_paths)}')
            print(f'Maximum number of cells in robots paths: {max(best_case_num_paths)}')
            print(f'Average number of cells in robots paths: {np.mean(np.array(best_case_num_paths))}')
            print(f'\nTurns Analysis: {self.best_case}')
            print(self.best_case.paths)
            
            
def CalcRealBinaryReg(BinaryRobotRegion, rows, cols):
    temp = np.zeros((2*rows, 2*cols))
    RealBinaryRobotRegion = np.zeros((2 * rows, 2 * cols), dtype=bool)
    for i in range(2*rows):
        for j in range(2*cols):
            temp[i, j] = BinaryRobotRegion[(int(i / 2))][(int(j / 2))]
            if temp[i, j] == 0:
                RealBinaryRobotRegion[i, j] = False
            else:
                RealBinaryRobotRegion[i, j] = True

    return RealBinaryRobotRegion

def calculateMSTs(BinaryRobotRegions, droneNo, rows, cols, mode):
    MSTs = []
    for r in range(droneNo):
        k = Kruskal(rows, cols)
        k.initializeGraph(BinaryRobotRegions[r, :, :], True, mode)
        k.performKruskal()
        MSTs.append(k.mst)
    return MSTs

def BuildMSTs(energy_MRPP) : 
            energy_MRPP.mode_to_drone_turns = []
            AllRealPaths_dict = {}
            subCellsAssignment_dict = {}
            for mode in range(4):
                MSTs = calculateMSTs(energy_MRPP.darp_instance.BinaryRobotRegions, energy_MRPP.darp_instance.droneNo, energy_MRPP.darp_instance.rows, energy_MRPP.darp_instance.cols, mode)
                AllRealPaths = []
                for r in range(energy_MRPP.darp_instance.droneNo):
                    ct = CalculateTrajectories(energy_MRPP.darp_instance.rows, energy_MRPP.darp_instance.cols, MSTs[r])
                    ct.initializeGraph(CalcRealBinaryReg(energy_MRPP.darp_instance.BinaryRobotRegions[r], energy_MRPP.darp_instance.rows, energy_MRPP.darp_instance.cols), True)
                    ct.RemoveTheAppropriateEdges()
                    ct.CalculatePathsSequence(4 * energy_MRPP.darp_instance.initial_positions[r][0] * energy_MRPP.darp_instance.cols + 2 * energy_MRPP.darp_instance.initial_positions[r][1])
                    AllRealPaths.append(ct.PathSequence)

                energy_MRPP.TypesOfLines = np.zeros((energy_MRPP.darp_instance.rows*2, energy_MRPP.darp_instance.cols*2, 2))
                for r in range(energy_MRPP.darp_instance.droneNo):
                    flag = False
                    for connection in AllRealPaths[r]:
                        if flag:
                            if energy_MRPP.TypesOfLines[connection[0]][connection[1]][0] == 0:
                                indxadd1 = 0
                            else:
                                indxadd1 = 1

                            if energy_MRPP.TypesOfLines[connection[2]][connection[3]][0] == 0 and flag:
                                indxadd2 = 0
                            else:
                                indxadd2 = 1
                        else:
                            if not (energy_MRPP.TypesOfLines[connection[0]][connection[1]][0] == 0):
                                indxadd1 = 0
                            else:
                                indxadd1 = 1
                            if not (energy_MRPP.TypesOfLines[connection[2]][connection[3]][0] == 0 and flag):
                                indxadd2 = 0
                            else:
                                indxadd2 = 1

                        flag = True
                        if connection[0] == connection[2]:
                            if connection[1] > connection[3]:
                                energy_MRPP.TypesOfLines[connection[0]][connection[1]][indxadd1] = 2
                                energy_MRPP.TypesOfLines[connection[2]][connection[3]][indxadd2] = 3
                            else:
                                energy_MRPP.TypesOfLines[connection[0]][connection[1]][indxadd1] = 3
                                energy_MRPP.TypesOfLines[connection[2]][connection[3]][indxadd2] = 2

                        else:
                            if (connection[0] > connection[2]):
                                energy_MRPP.TypesOfLines[connection[0]][connection[1]][indxadd1] = 1
                                energy_MRPP.TypesOfLines[connection[2]][connection[3]][indxadd2] = 4
                            else:
                                energy_MRPP.TypesOfLines[connection[0]][connection[1]][indxadd1] = 4
                                energy_MRPP.TypesOfLines[connection[2]][connection[3]][indxadd2] = 1

                subCellsAssignment = np.zeros((2*energy_MRPP.darp_instance.rows, 2*energy_MRPP.darp_instance.cols))
                for i in range(energy_MRPP.darp_instance.rows):
                    for j in range(energy_MRPP.darp_instance.cols):
                        subCellsAssignment[2 * i][2 * j] = energy_MRPP.darp_instance.A[i][j]
                        subCellsAssignment[2 * i + 1][2 * j] = energy_MRPP.darp_instance.A[i][j]
                        subCellsAssignment[2 * i][2 * j + 1] = energy_MRPP.darp_instance.A[i][j]
                        subCellsAssignment[2 * i + 1][2 * j + 1] = energy_MRPP.darp_instance.A[i][j]

                drone_turns = turns(AllRealPaths)
                drone_turns.count_turns()
                drone_turns.find_avg_and_std()
                energy_MRPP.mode_to_drone_turns.append(drone_turns)

                AllRealPaths_dict[mode] = AllRealPaths
                subCellsAssignment_dict[mode] = subCellsAssignment


            # Find mode with the smaller number of turns
            averge_turns = [x.avg for x in energy_MRPP.mode_to_drone_turns]
            energy_MRPP.min_mode = averge_turns.index(min(averge_turns))
            
            # Retrieve number of cells per robot for the configuration with the smaller number of turns
            min_mode_num_paths = [len(x) for x in AllRealPaths_dict[energy_MRPP.min_mode]]
            min_mode_returnPaths = AllRealPaths_dict[energy_MRPP.min_mode]

            # Uncomment if you want to visualize all available modes
            
            # if energy_MRPP.darp_instance.visualization:
            #     for mode in range(4):
            #         image = visualize_paths(AllRealPaths_dict[mode], subCellsAssignment_dict[mode],
            #                                 energy_MRPP.darp_instance.droneNo, energy_MRPP.darp_instance.color)
            #         image.visualize_paths(mode)
            #     print("Best Mode:", energy_MRPP.min_mode)

            #Combine all modes to get one mode with the least available turns for each drone
            combined_modes_paths = []
            combined_modes_turns = []
            
            for r in range(energy_MRPP.darp_instance.droneNo):
                min_turns = sys.maxsize
                temp_path = []
                for mode in range(4):
                    if energy_MRPP.mode_to_drone_turns[mode].turns[r] < min_turns:
                        temp_path = energy_MRPP.mode_to_drone_turns[mode].paths[r]
                        min_turns = energy_MRPP.mode_to_drone_turns[mode].turns[r]
                combined_modes_paths.append(temp_path)
                combined_modes_turns.append(min_turns)

            energy_MRPP.best_case = turns(combined_modes_paths)
            energy_MRPP.best_case.turns = combined_modes_turns
            energy_MRPP.best_case.find_avg_and_std()
            
            # Retrieve number of cells per robot for the best case configuration
            best_case_num_paths = [len(x) for x in energy_MRPP.best_case.paths]
            best_case_returnPaths = energy_MRPP.best_case.paths
            
            #visualize best case
            if energy_MRPP.darp_instance.visualization:
                image = visualize_paths(energy_MRPP.best_case.paths, subCellsAssignment_dict[energy_MRPP.min_mode],
                                        energy_MRPP.darp_instance.droneNo, energy_MRPP.darp_instance.color)
                image.visualize_paths("Combined Modes")

            energy_MRPP.execution_time = time.time() - energy_MRPP.start_time
            
            print(f'\nResults:')
            print(f'Number of cells per robot: {best_case_num_paths}')
            print(f'Minimum number of cells in robots paths: {min(best_case_num_paths)}')
            print(f'Maximum number of cells in robots paths: {max(best_case_num_paths)}')
            print(f'Average number of cells in robots paths: {np.mean(np.array(best_case_num_paths))}')
            print(f'\nTurns Analysis: {energy_MRPP.best_case}')
            #print(energy_MRPP.best_case.paths)

            return energy_MRPP.best_case.paths, subCellsAssignment_dict[energy_MRPP.min_mode], MSTs


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '-grid',
        default=(10, 10),
        type=int,
        nargs=2,
        help='Dimensions of the Grid (default: (10, 10))')
    argparser.add_argument(
        '-obs_pos',
        default=[5, 6, 7],
        nargs='*',
        type=int,
        help='Obstacles Positions (default: None)')
    argparser.add_argument(
        '-in_pos',
        default=[0, 3, 9],
        nargs='*',
        type=int,
        help='Initial Positions of the robots (default: (1, 3, 9))')
    argparser.add_argument(
        '-nep',
        action='store_true',
        help='Not Equal Portions shared between the Robots in the Grid (default: False)')
    argparser.add_argument(
        '-portions',
        default=[0.2, 0.3, 0.5],
        nargs='*',
        type=float,
        help='Portion for each Robot in the Grid (default: (0.2, 0.7, 0.1))')
    argparser.add_argument(
        '-vis',
        default=False,
        action='store_true',
        help='Visualize results (default: False)')
    argparser.add_argument(
        '-energy',
        default=[],
        nargs='*',
        type=int,
        help='Energy level of the robot')
    argparser.add_argument(
        '-exp',
        default=False,
        action='store_true',
        help='Launch experiments and evaluation procedure'
    )
    
    args = argparser.parse_args()

    if args.exp == True :
        print("LAUCHING EXPERIMENTS")
        Experiments(100,5)
    elif args.energy!= []:
        MultiRobotPathPlanner(args.grid[0], args.grid[1], args.nep, args.in_pos,  args.portions, args.obs_pos, args.vis, args.energy)
    else : 
        MultiRobotPathPlanner(args.grid[0], args.grid[1], args.nep, args.in_pos,  args.portions, args.obs_pos, args.vis)

#python3 multiRobotPathPlanner.py -obs 33 28 19 64 34 -in_pos 84 25 12 53 -energy 22 22 24 24
#probleme (len(self(coveredcells))) is never equal to 0 