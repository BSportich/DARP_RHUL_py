import numpy as np
import sys
import cv2
from Visualization import darp_area_visualization
import time
import random
import os
from numba import njit
from DFS import *

np.set_printoptions(threshold=sys.maxsize)

random.seed(3)
os.environ['PYTHONHASHSEED'] = str(1)
np.random.seed(5)

#@njit(fastmath=True)
def assign(droneNo, rows, cols, GridEnv, MetricMatrix, A):

    ArrayOfElements = np.zeros(droneNo)

    #Ben_modif ;;;;;;;;;;; Additional data structure to check for errors
    full_elements = {}
    for r in range(droneNo) :
        full_elements[r] = []
    #Ben_modif_end

    for i in range(rows):
        for j in range(cols):
            if GridEnv[i, j] == -1:
                minV = MetricMatrix[0, i, j]
                indMin = 0
                for r in range(droneNo):
                    if MetricMatrix[r, i, j] < minV:
                        minV = MetricMatrix[r, i, j]
                        indMin = r

                A[i, j] = indMin
                ArrayOfElements[indMin] += 1

                #Ben_modif
                full_elements[indMin].append( (i,j) )
                #Ben_modif_end                

            elif GridEnv[i, j] == -2:
                A[i, j] = droneNo
            #Ben_modif pre covered cells
            elif GridEnv[i,j] <= -100 : 
                # print("HOLAAAAA")
                temp_value =   - ( GridEnv[i,j] / 100 ) - 1 
                #print("TEMP VALUE "+str(temp_value))
                A[i,j] = temp_value
                ArrayOfElements[ int(temp_value) ] += 1
                #print("pre covered treatement")
            #Ben_modif_end
            #print(A)
    return A, ArrayOfElements, full_elements

@njit(fastmath=True)
def inverse_binary_map_as_uint8(BinaryMap):
    # cv2.distanceTransform needs input of dtype unit8 (8bit)
    return np.logical_not(BinaryMap).astype(np.uint8)

@njit(fastmath=True)
def euclidian_distance_points2d(array1: np.array, array2: np.array) -> np.float_:
    # this runs much faster than the (numba) np.linalg.norm and is totally enough for our purpose
    return (
                   ((array1[0] - array2[0]) ** 2) +
                   ((array1[1] - array2[1]) ** 2)
           ) ** 0.5

@njit(fastmath=True)
def constructBinaryImages(labels_im, robo_start_point, rows, cols):
    BinaryRobot = np.copy(labels_im)
    BinaryNonRobot = np.copy(labels_im)
    for i in range(rows):
        for j in range(cols):
            if labels_im[i, j] == labels_im[robo_start_point]:
                BinaryRobot[i, j] = 1
                BinaryNonRobot[i, j] = 0
            elif labels_im[i, j] != 0:
                BinaryRobot[i, j] = 0
                BinaryNonRobot[i, j] = 1

    return BinaryRobot, BinaryNonRobot

@njit(fastmath=True)
def CalcConnectedMultiplier(rows, cols, dist1, dist2, CCvariation):
    returnM = np.zeros((rows, cols))
    MaxV = 0
    MinV = 2**30

    for i in range(rows):
        for j in range(cols):
            returnM[i, j] = dist1[i, j] - dist2[i, j]
            if MaxV < returnM[i, j]:
                MaxV = returnM[i, j]
            if MinV > returnM[i, j]:
                MinV = returnM[i, j]

    for i in range(rows):
        for j in range(cols):
            returnM[i, j] = (returnM[i, j]-MinV)*((2*CCvariation)/(MaxV - MinV)) + (1-CCvariation)

    return returnM

def ModifBinaryRobot(BinaryRobot, BinaryNonRobot, pre_covered_cells):
    NewBinary = np.copy(BinaryRobot)
    NewNonBinary = np.copy(BinaryNonRobot)
    for cell in pre_covered_cells :
        if NewBinary[ cell[0] ][ cell[1] ] ==1 : 
            print("Binary corrected")
            NewBinary[ cell[0] ][ cell[1] ] = 0
            NewNonBinary [ cell[0] ][ cell[1] ] = 1
    
    return NewBinary, NewNonBinary

def correction_connectedMultiplier(rows, cols, ConnectedMultiplier, connectivity_map, initial_position):
    print(connectivity_map)
    print(initial_position)
    init_i = initial_position[0]
    init_j = initial_position[1]
    for i in range(rows) : 
        for j in range(cols) : 
            if connectivity_map[i][j] != connectivity_map[init_i][init_j] : 
                ConnectedMultiplier[i][j] = 10000000000000000
    return ConnectedMultiplier



class DARP:
    def __init__(self, nx, ny, notEqualPortions, given_initial_positions, given_portions, obstacles_positions, visualization ,
                #Ben_modif
                DARP_energy = False, #boolean
                pre_covered_cells = [],
                strategy_no = 3, #int
                opt_ass_type = 2, #int
                drones_energy = [],
                opt_threshold = "C", 
                valuation_grid = [],
                #Ben_modif_end
                 MaxIter=100000, CCvariation=0.01, #normal value 0.01
                 randomLevel=0.0001, dcells=2,
                 printok = False,
                 importance=False,
                 forced_disjoint = False,
                 isFinished = []):

        self.rows = nx
        self.cols = ny

        #Ben_modif
        self.printok = printok
        self.DARP_energy = DARP_energy
        self.strategy_no = strategy_no #Type of strategy 
        self.opt_ass_type = opt_ass_type # 1 or 2 : normalized or non-normalized
        self.opt_threshold = opt_threshold #A or B : smaller interval or large interval

        self.visualization = visualization
        self.MaxIter = MaxIter
        self.CCvariation = CCvariation
        self.randomLevel = randomLevel
        self.dcells = dcells
        self.importance = importance
        self.notEqualPortions = notEqualPortions

        #FINISHED PROCEDURE 
        if isFinished == [] : 
            print("finished ?")
            self.isFinished = [False] * len(drones_energy)
        else : 
            print("finished ! ")
            print(isFinished)
            self.isFinished = isFinished
        input()
        print("INITIAL POSITIONS ARE "+str(given_initial_positions))
        self.initial_positions, self.obstacles_positions, self.portions = self.sanity_check(given_initial_positions, given_portions, obstacles_positions, notEqualPortions)
        self.droneNo = len(self.initial_positions)
        #Ben_modif_end



        #Ben_modif
        #New data structure modification
        if self.DARP_energy == True : 
            self.drones_energy = drones_energy
            if len(pre_covered_cells) == 0 :
                self.pre_covered_cells = [ [] for i in range(self.droneNo) ] 
                self.pre_covered_cells_number = 0
            else : 
                self.pre_covered_cells = []
                for r in range(self.droneNo) : 

                    #FINISHED PROCEDURE
                    print(self.isFinished)
                    print(r)
                    print(range(self.droneNo))
                    print(self.droneNo)
                    print(self.initial_positions)
                    if self.isFinished[r] == True :
                    
                        self.drones_energy[r] = 0
                    
                    temp_pre_covered_cells_r = []

                    print(pre_covered_cells)
                    for cell in pre_covered_cells[r] : 
                        temp_pre_covered_cells_r.append( (cell // self.cols, cell % self.cols) ) 

                    self.pre_covered_cells.append( temp_pre_covered_cells_r )

                self.pre_covered_cells_number = sum(len(x) for x in self.pre_covered_cells)

            self.cell_coverage_energy_cost = 1
            self.full_covered_cells = [] #TO CHANGE
            self.EffectiveSize = (self.rows*self.cols) - self.droneNo - len(self.obstacles_positions) - sum(len(x) for x in self.pre_covered_cells) - len(self.full_covered_cells)
            self.IsNormalized = None  #If normalization is forced for the assignment 
            self.opt_ass = self.ComputeOptimalAssignmentNew()
            print("Optimal assignment "+str(self.opt_ass))
            self.effective_size_portions = []
            for r in range(self.droneNo) : 
                self.effective_size_portions.append( self.opt_ass[r] * self.EffectiveSize)
            print("Effective size portions "+str(self.effective_size_portions))
            print("Energy available "+str(drones_energy))
            print("Effective size total = "+str(self.EffectiveSize))

            self.robots_thresholds = []   #Target interval for each robot

            if len(valuation_grid) == 0 :
                self.valuation_grid = np.ones((self.rows, self.cols))
                print("OBSTACLES = "+str(self.obstacles_positions))
                for obstacle in self.obstacles_positions : 
                    self.valuation_grid[ obstacle[0] ][ obstacle[1] ] = 0 
            else : 
                self.valuation_grid = valuation_grid

            self.RobotLabels = {}
            self.BinaryRobotMainRegion = {}
            self.BinaryRobotSecondaryRegion = {} #CHANGE NAME

            self.RejectedCells = {}
            self.corrected_cell_assignment = {} #Final cell assignment after rejection
            self.RejectedValue = {}
            self.hasRejectionHappened = False

            self.forced = forced_disjoint
            print("FORCED DISJOINT IS "+str(forced_disjoint))

            
        #End new data structure modification
        #Ben_modif_end
    
        self.debug_prints = False
        print("\nInitial Conditions Defined:")
        print("Grid Dimensions:", nx, ny)
        print("Number of Robots:", len(self.initial_positions))
        print("Initial Robots' positions", self.initial_positions)
        #Ben_modif
        print("Obstacles position ", self.obstacles_positions)
        if DARP_energy == True : 
            print("Portions (energy-based) for each Robot:", self.opt_ass, "\n")
            print("Pre-covered cells for each robot "+str(self.pre_covered_cells))
        else : 
        #Ben_modif_end
            print("Portions for each Robot:", self.portions, "\n")

        
        self.A = np.zeros((self.rows, self.cols))
        self.GridEnv = self.defineGridEnv()
   
        self.connectivity = np.zeros((self.droneNo, self.rows, self.cols), dtype=np.uint8)
        self.BinaryRobotRegions = np.zeros((self.droneNo, self.rows, self.cols), dtype=bool)

        self.MetricMatrix, self.termThr, self.Notiles, self.DesireableAssign, self.TilesImportance, self.MinimumImportance, self.MaximumImportance= self.construct_Assignment_Matrix()
        self.ArrayOfElements = np.zeros(self.droneNo)
        self.color = []

        #input() DEBUGGER PRINT

        for r in range(self.droneNo):
            np.random.seed(r)
            self.color.append(list(np.random.choice(range(256), size=3)))
        
        np.random.seed(1)
        #if self.visualization:
        self.assignment_matrix_visualization = darp_area_visualization(self.A, self.droneNo, self.color, self.initial_positions)

    def sanity_check(self, given_initial_positions, given_portions, obs_pos, notEqualPortions, DARP_energy = False):
        initial_positions = []
        for position in given_initial_positions:
            if position < 0 or position >= self.rows * self.cols:
                print("Initial positions should be inside the Grid.")
                sys.exit(1)
            initial_positions.append((position // self.cols, position % self.cols))

        obstacles_positions = []
        for obstacle in obs_pos:
            if obstacle < 0 or obstacle >= self.rows * self.cols:
                print("Obstacles should be inside the Grid.")
                sys.exit(2)
            obstacles_positions.append((obstacle // self.cols, obstacle % self.cols))

        #for i in range(len(initial_positions)) : 
        i=0
        for position in initial_positions:
            #position = initial_positions[i]
            for obstacle in obstacles_positions:
                if position[0] == obstacle[0] and position[1] == obstacle[1]:
                    print("Initial positions should not be on obstacles")
                    if self.isFinished[i] == False : 
                        sys.exit(5)
                    else : 
                        #FINISHED PROCEDURE 
                        print("Robot "+str(i)+" is finished")
                        input()
                        #self.drones_energy[i] = 0
            i=i+1

        portions = []
        if notEqualPortions:
            #Ben_modif
            if self.DARP_energy == True : 
                portions = None
                return initial_positions, obstacles_positions, portions
                
            else : 
            #Ben_modif_end
                print("Ben error")
                exit(1)
                portions = given_portions

        else:
            print("PORTIONS ACTIVATED : EQUALS")
            for drone in range(len(initial_positions)):
                portions.append(1 / len(initial_positions))

        if len(initial_positions) != len(portions):
            print("Portions should be defined for each drone")
            sys.exit(3)

        s = sum(portions)
        if abs(s - 1) >= 0.0001:
            print("Sum of portions should be equal to 1.")
            sys.exit(4)

     

        return initial_positions, obstacles_positions, portions
          
    def defineGridEnv(self):
        GridEnv = np.full(shape=(self.rows, self.cols), fill_value=-1)  # create non obstacle map with value -1
        
        # obstacle tiles value is -2
        for idx, obstacle_pos in enumerate(self.obstacles_positions):
            GridEnv[obstacle_pos[0], obstacle_pos[1]] = -2

        connectivity = np.zeros((self.rows, self.cols))
        
        mask = np.where(GridEnv == -1)
        connectivity[mask[0], mask[1]] = 255
        image = np.uint8(connectivity)
        num_labels, labels_im = cv2.connectedComponents(image, connectivity=4)

        if num_labels > 2:
            print("The environment grid MUST not have unreachable and/or closed shape regions")
            print(labels_im)
            if self.forced == False : 
                sys.exit(6)
            else : 
                print("Disconnected region mode : FORCED ")
                self.printok = True
                self.debug_prints = True
                input()

        self.connectivity_map = labels_im
        self.connectivity_map_nums = num_labels

        #ben_modif
        if self.pre_covered_cells_number > 0 : 
            for r in range(self.droneNo) : 
                for cell in self.pre_covered_cells[r] : 
                    # print(cell)
                    # print( -(r+1) * 100)
                    #if cell[0] != self.initial_positions[r][0] and cell[1] != self.initial_positions[r][1]: #INSTABLE ?
                    GridEnv[ cell[0] ][ cell[1] ] = -(r+1) * 100
                    if self.isFinished[r] == True : 
                        GridEnv[ cell[0] ][ cell[1] ] = -1

            # print("PRE COVERED CELLS "+str(GridEnv))

        
        #ben_modif_end
        
        # initial robot tiles will have their array.index as value
        for idx, robot in enumerate(self.initial_positions):
            if self.isFinished[idx] == False : 
                GridEnv[robot] = idx
                self.A[robot] = idx

        return GridEnv

    def divideRegions(self):
        self.success = False
        cancelled = False
        criterionMatrix = np.zeros((self.rows, self.cols))
        iteration = 0

        while self.termThr <= self.dcells and not self.success and not cancelled:
            downThres = (self.Notiles - self.termThr*(self.droneNo-1))/(self.Notiles*self.droneNo)
            upperThres = (self.Notiles + self.termThr)/(self.Notiles*self.droneNo)


            #Ben_modif
            self.success = True
            if self.DARP_energy == True : 
                self.ComputeThresholdsRobots()
            #Ben_modif_end

            # Main optimization loop

            iteration=0

            while iteration <= self.MaxIter and not cancelled:
                self.A, self.ArrayOfElements, ArrayOfElements_full = assign(self.droneNo,
                                                      self.rows,
                                                      self.cols,
                                                      self.GridEnv,
                                                      self.MetricMatrix,
                                                      self.A)
                ConnectedMultiplierList = np.ones((self.droneNo, self.rows, self.cols),  dtype=np.float64)
                ConnectedRobotRegions = np.zeros(self.droneNo)
                plainErrors = np.zeros((self.droneNo))
                divFairError = np.zeros((self.droneNo))

                self.update_connectivity()
                for r in range(self.droneNo):
                    ConnectedMultiplier = np.ones((self.rows, self.cols))
                    ConnectedRobotRegions[r] = True
                    num_labels, labels_im = cv2.connectedComponents(self.connectivity[r, :, :], connectivity=4)

                    #Ben_modif
                    BinaryRobot, BinaryNonRobot = constructBinaryImages(labels_im, self.initial_positions[r], self.rows, self.cols)
                    if self.DARP_energy == True : 

                        #Compute the main assigned connected component for each robot 
                        self.RobotLabels[r] = num_labels, labels_im
                        self.BinaryRobotMainRegion[r] = BinaryRobot

                        #The current position of the robot should not be considered here (value should be changed back to 1 at rejection stage)
                        self.BinaryRobotMainRegion[r][ self.initial_positions[r][0] ][ self.initial_positions[r][1] ] = 0 
                        self.BinaryRobotSecondaryRegion[r] = BinaryNonRobot

                        #Only enters here in case of problem : 
                        #The main robot region should always be smaller than the total of cells assigned to this robot (all the cells that are not connected to the main components are not here)
                        if np.sum(self.BinaryRobotMainRegion[r]) > self.ArrayOfElements[r] and self.isFinished == False : 
                            print("MAJOR ERROR")
                            print(np.sum(self.BinaryRobotMainRegion[r]))
                            print(self.ArrayOfElements[r])
                            print(BinaryRobot)
                            print(np.sum(BinaryRobot))
                            print(ArrayOfElements_full[r])
                            input()
                            print(self.A)
                            print(self.connectivity[r])
                            input()
                        

                        
                    #Ben_modif_end

                    if num_labels > 2:
                        ConnectedRobotRegions[r] = False
                        #Ben_modif ;;;;;;;;;;; In the original code, BinaryRobot are computed here, so this line is not necessary anymore
                        #BinaryRobot, BinaryNonRobot = constructBinaryImages(labels_im, self.initial_positions[r], self.rows, self.cols)
                        #Ben_modif_end
                        
                        #Test correction for connectivity (part 1)
                        # old_binary_robot, old_binary_non_robot = BinaryRobot, BinaryNonRobot
                        # BinaryRobot, BinaryNonRobot = ModifBinaryRobot(BinaryRobot, BinaryNonRobot, self.pre_covered_cells[r])

                        ConnectedMultiplier = CalcConnectedMultiplier(self.rows, self.cols,
                                                                      self.NormalizedEuclideanDistanceBinary(True, BinaryRobot),
                                                                      self.NormalizedEuclideanDistanceBinary(False, BinaryNonRobot), self.CCvariation)
                        
                        #Tentative correction instable 
                        ConnectedMultiplier = correction_connectedMultiplier(self.rows, self.cols, ConnectedMultiplier, self.connectivity_map, self.initial_positions[r])
                    ConnectedMultiplierList[r, :, :] = ConnectedMultiplier
                    
                    #Test correction for connectivity (part 2)
                    # BinaryRobot = old_binary_robot
                    # BinaryNonRobot = old_binary_non_robot

                    #Ben_modif
                    #In the energy case, the thresholds are computed differently
                    if self.DARP_energy == True : 
                        
                        downThres = self.robots_thresholds[r][0]
                        upperThres = self.robots_thresholds[r][1]
                        #The error also
                        plainErrors[r] = (self.ArrayOfElements[r] - len( self.pre_covered_cells[r]  )  ) /self.EffectiveSize
                        if plainErrors[r] < 0 : plainErrors[r] = 0 #INSTABLE ? 
                        if(self.printok) : print("Array of ",self.ArrayOfElements[r])
                        if(self.printok) : print((self.ArrayOfElements[r] - len( self.pre_covered_cells[r]  )  ) )
                    else :  
                    #Ben_modif_end
                        plainErrors[r] = self.ArrayOfElements[r]/(self.DesireableAssign[r]*self.droneNo) 

                    if plainErrors[r] < downThres:
                        divFairError[r] = downThres - plainErrors[r]
                    elif plainErrors[r] > upperThres:
                        divFairError[r] = upperThres - plainErrors[r]
                
                #Ben_modif
                #USABLE PRINTS
                #self.randomLevel = self.randomLevel * (max(1,iteration / 10000))
                if self.debug_prints == True : 
                    self.printok = True
                    print("Convergence")
                    print(self.isFinished)
                    print("OPT ASS % "+str(self.opt_ass))
                    #print(self.effective_size_portions)
                    print("DESIRABLE ASSIGN "+str(self.DesireableAssign))
                    print("CELLS ASSIGNED : "+str(self.ArrayOfElements))
                    nb_p_c = []
                    #print(self.pre_covered_cells)
                    for u in range(self.droneNo) : 
                        nb_p_c.append( len(self.pre_covered_cells[u]) )

                    print("NB PRE COVERED CELLS "+str(nb_p_c))
                    print("THRESHOLDS : " +str(self.robots_thresholds))
                    #print(self.opt_ass)
                    print("CURRENT SHARE IS "+str(plainErrors))
                    #print("SUM OF PLAIN ERRORS "+str(np.sum(plainErrors)))
                    print("ERROR IS "+str(divFairError))
                    #print(np.sum(self.ArrayOfElements))
                    print("TOTAL CELLS AVAILABLE "+str(self.EffectiveSize))
                    # print(self.ArrayOfElements)
                    #print(criterionMatrix)
                    #print(self.MetricMatrix)
                    if iteration > 0 : 
                        if(self.printok) : print("CORRECTION MULTIPLIER : "+str(correctionMult))

                    #input()
                    temp_sum_old = 0 
                    if iteration > 0 :
                        temp_sum_old =temp_sum
                    temp_sum = 0
                    for r in range(self.droneNo) : 
                        #print( plainErrors[r] > self.robots_thresholds[r][0] and plainErrors[r] < self.robots_thresholds[r][1])
                        temp_sum = np.abs( self.DesireableAssign[r] - self.ArrayOfElements[r])
                    print("DISTANCE TO DESIRABLE ASSIGN "+str(temp_sum))
                    print("CHANGES SINCE LAST ITER "+str(temp_sum-temp_sum_old))
                #print(self.termThr)

                if self.DARP_energy == True :
                    #print(divFairError)
                    if self.IsThisAGoalState_new(self.termThr, ConnectedRobotRegions) : 

                        self.CellsRejectionProcess()
                        self.assignment_matrix_visualization.placeCells_withRejection(self.A, self.corrected_cell_assignment, iteration_number=iteration)  
                        time.sleep(1)
                        
                        break

                    #Error case where the process is stagnant
                    elif sum(divFairError) == 0 and self.IsNormalized == False : 
                        print(divFairError)
                        print("THRESH IS "+str(self.termThr))
                        print("GOAL STATE TEST ")
                        for r in range(self.droneNo) :
                            print("Main region : "+str(np.sum(self.BinaryRobotMainRegion[r])))
                            print(" Full cells : "+str(self.ArrayOfElements[r]))
                            print((self.robots_thresholds[r][0]*self.EffectiveSize) - np.sum(self.BinaryRobotMainRegion[r]))
                            print(str(self.robots_thresholds[r][0] * self.EffectiveSize)+","+str(self.robots_thresholds[r][1] * self.EffectiveSize))
                        #exit(1)
                else :
                #Ben_modif_end
                    if self.IsThisAGoalState(self.termThr, ConnectedRobotRegions):
                        self.assignment_matrix_visualization.placeCells(self.A, iteration_number=iteration)
                        time.sleep(1)
                        break

                TotalNegPerc = 0
                totalNegPlainErrors = 0
                correctionMult = np.zeros(self.droneNo)

                TotalPosPerc = 0 
                totalPosPlainErrors = 0

                for r in range(self.droneNo):
                    if divFairError[r] < 0:  
                        TotalNegPerc += np.absolute(divFairError[r])
                        totalNegPlainErrors += plainErrors[r]

                    correctionMult[r] = 1

                #Ben modif instable
                #     if divFairError[r] > 0 : 
                #         TotalPosPerc += np.absolute(divFairError[r])
                #         totalPosPlainErrors += plainErrors[r]
                        
                # if TotalPosPerc > TotalNegPerc : 
                #     TotalNegPerc = TotalPosPerc
                #     totalNegPlainErrors = totalPosPlainErrors
                #End modif instable
                
                #USABLE PRINTS
                # print("double value check")
                # print(str(totalNegPlainErrors))
                # print(str(TotalNegPerc))

                for r in range(self.droneNo):
                    if totalNegPlainErrors != 0:
                        if divFairError[r] < 0:
                            correctionMult[r] = 1 + (plainErrors[r]/totalNegPlainErrors)*(TotalNegPerc/2)
                        else:
                            correctionMult[r] = 1 - (plainErrors[r]/totalNegPlainErrors)*(TotalNegPerc/2)

                        #USABLE PRINTS
                        # print("correction multiplier "+str(correctionMult[r]))
                        #if no importance, returns correctionMult
                        criterionMatrix = self.calculateCriterionMatrix(
                                self.TilesImportance[r],
                                self.MinimumImportance[r],
                                self.MaximumImportance[r],
                                correctionMult[r],
                                divFairError[r] < 0)

                    self.MetricMatrix[r] = self.FinalUpdateOnMetricMatrix(
                            criterionMatrix,
                            self.generateRandomMatrix(),
                            self.MetricMatrix[r],
                            ConnectedMultiplierList[r, :, :])
                    
                    if self.drones_energy[r] == 0 or self.isFinished[r] == True : 
                        self.MetricMatrix[r] = np.full((self.rows, self.cols), float('inf'))
                        print("Robot "+str(r)+" is finished : 0 cells should be assigned")

                iteration += 1
                if self.visualization:

                    #Ben_modif
                    if self.DARP_energy == True  and self.hasRejectionHappened == True : 
                        self.assignment_matrix_visualization.placeCells_withRejection(self.A, self.corrected_cell_assignment, iteration_number=iteration)  
                        time.sleep(1)
                    else : 
                        self.assignment_matrix_visualization.placeCells(self.A, iteration_number=iteration)
                        time.sleep(0.2)

            if iteration >= self.MaxIter:
                self.MaxIter = self.MaxIter/2
                self.success = False
                self.termThr += 1
                self.debug_prints = True

        self.getBinaryRobotRegions()
        return self.success, iteration

    def getBinaryRobotRegions(self):
        ind = np.where(self.A < self.droneNo)
        temp = (self.A[ind].astype(int),)+ind
        self.BinaryRobotRegions[temp] = True

    def generateRandomMatrix(self):
        RandomMatrix = np.zeros((self.rows, self.cols), dtype=np.float64)
        RandomMatrix = 2*self.randomLevel*np.random.uniform(0, 1,size=RandomMatrix.shape) + (1 - self.randomLevel)
        return RandomMatrix

    def FinalUpdateOnMetricMatrix(self, CM, RM, currentOne, CC):
        MMnew = np.zeros((self.rows, self.cols))
        MMnew = currentOne*CM*RM*CC

        return MMnew

    def IsThisAGoalState(self, thresh, connectedRobotRegions):
        #print("IS THIS A GOAL STATE")
        if(self.printok) : print("thresh ", thresh)
        for r in range(self.droneNo):
            #Ben_modif
            #print("NN "+str(len(self.pre_covered_cells[r])) ) 
            if self.printok : 
                print("Connected robots region : "+str(connectedRobotRegions[r]))
            #print( np.absolute(self.DesireableAssign[r] - self.ArrayOfElements[r] ) )

            #USABLE PRINTS
            # print("Error")
                print("PRE COVERED CELLS "+str(len(self.pre_covered_cells[r])))
                print( "VALUE TESTED FOR GOAL : "+str(np.absolute(self.DesireableAssign[r] - self.ArrayOfElements[r] + len(self.pre_covered_cells[r])) ))

            #print(self.DesireableAssign)
            #print(self.ArrayOfElements)
            #print(thresh)
            
            #if np.absolute(self.DesireableAssign[r] - self.ArrayOfElements[r] + len(self.pre_covered_cells[r])) > thresh or not connectedRobotRegions[r]: #ORIGINAL WORKING
            #ABSOLUTE TEST OMG : 
            if np.absolute(self.DesireableAssign[r] - self.ArrayOfElements[r] ) > thresh or not connectedRobotRegions[r]: #ORIGINAL WORKING

            #Ben_modif_end
                return False
        return True

    def IsThisAGoalState_new(self, thresh, connectedRobotRegions) : #TO MODIFYYYY
        # print(" checkpoint 1 ")
        if self.IsNormalized :
            return self.IsThisAGoalState(thresh, connectedRobotRegions)
        else : 
            
            for r in range(self.droneNo) : 

                pre_covered_cells_count = 0 
                for cell in self.pre_covered_cells[r] : 
                    if self.BinaryRobotMainRegion[r][ cell[0] ][ cell[1] ] == 1 :
                        pre_covered_cells_count = pre_covered_cells_count +1 
                
                # print(" checkpoint 2 ")
                # print("HERE" + str(pre_covered_cells_count))
                #ORIGINAL : if np.floor(self.robots_thresholds[r][0]*self.EffectiveSize) - np.sum(self.BinaryRobotMainRegion[r]) - pre_covered_cells_count  > 0 :
                if np.floor(self.robots_thresholds[r][0]*self.EffectiveSize) - np.sum(self.BinaryRobotMainRegion[r])  > 0 : #ABSOLUTE TEST OMG
                    return False

        # print(" checkpoint 3 ")
        for r in range(self.droneNo) : 
            if(self.printok) : 
                print("IS THIS GOAL STATE")
                print(np.floor(self.robots_thresholds[r][0]*self.EffectiveSize))
                print(np.sum(self.BinaryRobotMainRegion[r]))
                print(thresh)
                  
        
        return True

    def update_connectivity(self):
        self.connectivity = np.zeros((self.droneNo, self.rows, self.cols), dtype=np.uint8)
        for i in range(self.droneNo):
            mask = np.where(self.A == i)
            self.connectivity[i, mask[0], mask[1]] = 255

    # Construct Assignment Matrix
    def construct_Assignment_Matrix(self):
        Notiles = self.rows*self.cols
        fair_division = 1/self.droneNo
        effectiveSize = Notiles - self.droneNo - len(self.obstacles_positions)
        #Ben_modif
        if self.DARP_energy == True : 
            effectiveSize = self.EffectiveSize
        #Ben_modif_end
        

        termThr = 0
        if effectiveSize % self.droneNo != 0 and self.opt_ass_type == 1: #CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
            termThr = 1

        DesireableAssign = np.zeros(self.droneNo)
        MaximunDist = np.zeros(self.droneNo)
        MaximumImportance = np.zeros(self.droneNo)
        MinimumImportance = np.zeros(self.droneNo)

        for i in range(self.droneNo):
            #Ben_modif
            if self.DARP_energy == True : 
                DesireableAssign[i] = effectiveSize * self.opt_ass[i]
            else : 
            #Ben_modif_end
                DesireableAssign[i] = effectiveSize * self.portions[i]

            MinimumImportance[i] = sys.float_info.max
            if (DesireableAssign[i] != int(DesireableAssign[i]) and termThr != 1):
                termThr = 1

        AllDistances = np.zeros((self.droneNo, self.rows, self.cols))
        TilesImportance = np.zeros((self.droneNo, self.rows, self.cols))

        for x in range(self.rows):
            for y in range(self.cols):
                tempSum = 0
                for r in range(self.droneNo):
                    AllDistances[r, x, y] = euclidian_distance_points2d(np.array(self.initial_positions[r]), np.array((x, y))) # E!
                    
                    #Ben change
                    if self.drones_energy[r] == 0 or self.isFinished[r] == True : 
                         AllDistances[r,x,y] = float('inf')
                         print("Robot "+str(r)+" is finished : 0 cells should be assigned")



                    #End ben change

                    if AllDistances[r, x, y] > MaximunDist[r]:
                        MaximunDist[r] = AllDistances[r, x, y]
                    tempSum += AllDistances[r, x, y]

                for r in range(self.droneNo):
                    if tempSum - AllDistances[r, x, y] != 0:
                        TilesImportance[r, x, y] = 1/(tempSum - AllDistances[r, x, y])
                    else:
                        TilesImportance[r, x, y] = 1
                    # Todo FixMe!
                    if TilesImportance[r, x, y] > MaximumImportance[r]:
                        MaximumImportance[r] = TilesImportance[r, x, y]

                    if TilesImportance[r, x, y] < MinimumImportance[r]:
                        MinimumImportance[r] = TilesImportance[r, x, y]

        return AllDistances, termThr, Notiles, DesireableAssign, TilesImportance, MinimumImportance, MaximumImportance

    def calculateCriterionMatrix(self, TilesImportance, MinimumImportance, MaximumImportance, correctionMult, smallerthan_zero,):
        returnCrit = np.zeros((self.rows, self.cols),  dtype=np.float64)
        if self.importance:
            if smallerthan_zero:
                returnCrit = (TilesImportance- MinimumImportance)*((correctionMult-1)/(MaximumImportance-MinimumImportance)) + 1
            else:
                returnCrit = (TilesImportance- MinimumImportance)*((1-correctionMult)/(MaximumImportance-MinimumImportance)) + correctionMult
        else:
            returnCrit[:, :] = correctionMult

        return returnCrit

    def NormalizedEuclideanDistanceBinary(self, RobotR, BinaryMap):
        distRobot = cv2.distanceTransform(inverse_binary_map_as_uint8(BinaryMap), distanceType=2, maskSize=0, dstType=5)
        MaxV = np.max(distRobot)
        MinV = np.min(distRobot)

        #Normalization
        if RobotR:
            distRobot = (distRobot - MinV)*(1/(MaxV-MinV)) + 1
        else:
            distRobot = (distRobot - MinV)*(1/(MaxV-MinV))

        return distRobot

    #Ben_modif
    #Consider that some energy must be kept for the remaining path of the pre-covered cells
    def ComputeOptimalAssignmentNew(self) : 
        opt_ass = np.zeros((self.droneNo))

        print("ENERGY :"+str(self.drones_energy))
        print("PRE COVERED CELLS "+str(self.pre_covered_cells))

        sub_energy = np.zeros((self.droneNo))
        nb_pre_covered_cells = np.zeros((self.droneNo))
        for r in range(self.droneNo) : 
            sub_energy[r] = ( ( self.drones_energy[r] / self.cell_coverage_energy_cost ) - (len(self.pre_covered_cells[r]) *0.5 ) ) 
            if sub_energy[r] < 0 : 
                sub_energy[r] = 0 


        for r in range(self.droneNo) : 

            if ( ( self.drones_energy[r] / self.cell_coverage_energy_cost ) - (len(self.pre_covered_cells[r]) *0.5 )  ) > 0 : 

                #DARP core algorithm can not converge if sum of aimed values is > 1 : 
                #Total sum of energy left across all robots amounts to more than the number of cells uncovered :
                #Forced normalization

                if ( (sum(self.drones_energy) / self.cell_coverage_energy_cost) - ( sum(len(x) for x in self.pre_covered_cells) *0.5)  ) > self.EffectiveSize  : 
                    #Total sum of portion assigments on all drones equals 1 (Normalized = Option 1 )
                    opt_ass[r] = ( ( self.drones_energy[r] / self.cell_coverage_energy_cost ) - (len(self.pre_covered_cells[r]) *0.5 ) ) / ( (sum(self.drones_energy) / self.cell_coverage_energy_cost) -  ( sum(len(x) for x in self.pre_covered_cells) *0.5))
                    opt_ass[r] =sub_energy[r] / ( sum(sub_energy))
                    opt_ass[r] =sub_energy[r] / ( sum(self.drones_energy)) #OMG ABSOLUTE TEST
                    self.IsNormalized = True 
                    print("FORCED NORMALIZATION")
                ##DARP core algorithm can converge (and potentially faster) if sum of aimed values is < 1 : 
                else : 

                    #Total sum of share of drones can be lower or over 1 
                    #Can be normalized (option 1) or non normalized (option 2) (choice of the user)

                    #Option 1 : Normalized
                    if self.opt_ass_type == 1 : 
                        opt_ass[r] = ( ( self.drones_energy[r] / self.cell_coverage_energy_cost ) - (len(self.pre_covered_cells[r]) *0.5 ) ) / ( (sum(self.drones_energy) / self.cell_coverage_energy_cost) -  ( sum(len(x) for x in self.pre_covered_cells) *0.5 )) 
                        opt_ass[r] =sub_energy[r] / ( sum(sub_energy))
                        opt_ass[r] =sub_energy[r] / ( sum(self.drones_energy)) #OMG ABSOLUTE TEST
                        self.IsNormalized = True 

                    #Option 2 : non normalized
                    else : 
                        opt_ass[r] = ( ( self.drones_energy[r] / self.cell_coverage_energy_cost ) - (len(self.pre_covered_cells[r]) *0.5 ) ) / ( self.EffectiveSize )
                        self.IsNormalized = False
            else : 

                opt_ass[r] = 0

            #FINISHED PROCEDURE 
            if self.isFinished[r] == True : 
                print("ROBOT IS FINISHED "+str(r))
                opt_ass[r] = 0

        if(self.printok) : print("Optimal assignment is ", opt_ass)
        if(self.printok) : print(np.sum(opt_ass))

        #OMG ABSOLUTE TEST
        if sum(opt_ass) > 0.99 :
            self.IsNormalized = True
        else : 
            self.IsNormalized = False
        #END

        return opt_ass


    def ComputeThresholdsRobots(self) :
        self.robots_thresholds = [] 
        print("IS NORMALIZED ? ", self.IsNormalized)
        for r in range(self.droneNo) : 
            if self.IsNormalized : 
                LowerThreshold = ( (self.opt_ass[r]*self.EffectiveSize) - self.termThr ) / self.EffectiveSize
                HigherThreshold = ( (self.opt_ass[r] * self.EffectiveSize) + self.termThr) / self.EffectiveSize

    
                if (self.opt_ass[r] * self.EffectiveSize ) < self.termThr or ( (self.opt_ass[r] * self.EffectiveSize)+ self.termThr) > self.EffectiveSize : 
                    print("ERROR : TERMTHR EXCEEDED EXPECTED VALUES ")
                    print("ERROR : LOWER OR UPPER THRESHOLD OUT OF BOUNDS FOR ROBOT "+str(r))
                    print(self.opt_ass)
                    print(self.EffectiveSize)
                    print(LowerThreshold)
                    print(HigherThreshold)
                    self.debug_prints = True
                    #self.visualization = True
                    if self.opt_ass[r] == 0 : 
                        print("ROBOTS AFFECTED 0 CELLS")
                        LowerThreshold = 0
                        input()
                    else :

                        if LowerThreshold < 0 : 
                            LowerThreshold = 0
                            input()
                        else: 
                            exit()

            else : 
                LowerThreshold = (self.opt_ass[r] * self.EffectiveSize) / self.EffectiveSize
                #if self.opt_threshold == "A" : 
                #    HigherThreshold = ((self.opt_ass[r] + 1 - sum(self.opt_ass)) * self.EffectiveSize - self.termThr) / self.EffectiveSize
                #elif self.opt_threshold == "B" :
                #    HigherThreshold = (self.EffectiveSize - self.termThr) / self.EffectiveSize
                #elif self.opt_threshold == "C" : 
                temp_value = ( ( self.drones_energy[r] / self.cell_coverage_energy_cost ) - (len(self.pre_covered_cells[r]) *0.5 ) ) / ( (sum(self.drones_energy) / self.cell_coverage_energy_cost) -  ( sum(len(x) for x in self.pre_covered_cells) *0.5 )) 
                temp_value = ( (self.opt_ass[r] * self.EffectiveSize) + 2* self.termThr) / self.EffectiveSize
                HigherThreshold = temp_value

                if LowerThreshold == 0 :
                    HigherThreshold = 0


                
                if self.success == False : 
                    print("TERMTHR WAS INCREASED WHILE TARGET INTERVAL WAS REALLY LARGE")
                    print("LIKELY NO SOLUTION TO BE FOUND")
            if self.isFinished[r] == True :
                LowerThreshold = 0
                HigherThreshold = 1
            
            print("THRESHOLDS FOR ROBOT "+str(r)+" "+str(LowerThreshold)+" "+str(HigherThreshold))
            self.robots_thresholds .append( (LowerThreshold, HigherThreshold))


    def CellsRejectionProcess(self): 

        # print("REJECTION ")
        # print(self.BinaryRobotMainRegion)
        # print(self.valuation_grid)
        # input()
        print("ENERGY IS "+str(self.drones_energy))
        temp_tab = []
        for r in range(self.droneNo) : 
            temp_tab.append(np.sum(self.BinaryRobotMainRegion[r]))
        print("CELLS SETS ARE :"+str(temp_tab))

        
        for r in range(self.droneNo) : 

            self.BinaryRobotMainRegion[r][ self.initial_positions[r][0] ][ self.initial_positions[r][1] ] = 1
            self.RejectedCells[r], self.corrected_cell_assignment[r], self.RejectedValue[r] = RejectionProcess( self.BinaryRobotMainRegion[r], self.valuation_grid, self.rows, self.cols, self.drones_energy[r], self.initial_positions[r], self.pre_covered_cells[r] ) #CHANGE CHANGE self.drones_energy LEFT ???
            self.hasRejectionHappened = True
            print("Rejected cells for robot "+str(r)+" "+str(len(self.RejectedCells[r])) ) 
            print("Rejected cells :"+str(self.RejectedCells[r]))
            print("Precovered cells :"+str(self.pre_covered_cells[r]))

        #verification
        for r in range(self.droneNo) :
            for cell in self.pre_covered_cells[r] :
                    
                #print(self.corrected_cell_assignment)
                if self.corrected_cell_assignment[r][ cell[0] ][ cell[1] ] != 1 : 
                    print(" FATAL ERROR") 
                    print(self.corrected_cell_assignment)
                    print(cell)
                    print(r)
        
        
        return


