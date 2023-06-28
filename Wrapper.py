

from multiRobotPathPlanner import MultiRobotPathPlanner, Energy_MRPP, BuildMSTs, generate_instance_initial_random, PositionTransformer
from Visualization import visualize_paths
import numpy as np
import random as random
import Edges
import rosplan_interface_copy as ri_copy
from diagnostic_msgs.msg import KeyValue
import math

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

        self.cost_one_cell = None

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


        #information about the DB
        self.drones_name = []
        self.energy_level = []
        self.drones_altitude = []
        #self.ROSplan_obj = ri_copy.Rosplan_object()

        #parameters of the algorithm
        self.opt_ass_type = opt_ass_type #Normalized or non-normalized
        self.strategy =  strategy  #full replanning ? distance to robots or distance to MSTs

        self.DARP_steps = []
        #self.createInitialStep()
    
    
    def createInitialStep(self) : 
        initial_step = DARP_step(0, [ ((nx * ny ) - len(self.obstacles_start)) / self.dronesNo] * self.dronesNo, self.start_positions, self.obstacles_start)
        
        self.DARP_steps.append( initial_step )

    def createInitialStepFromData(self):

        initial_step = DARP_step(0, self.energy_level, self.start_positions, self.obstacles_start)
        initial_step.cost_one_cell = self.cost_one_cell
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

            if self.DARP_steps[-1].drones_energy[r] > 0 : 

                factor = max( int(self.DARP_steps[-1].drones_energy[r]/ 3 ), 10 )

                generation_factor = random.randint(0,factor)
                consumption_factor = random.randint(80,120) / 100 
     
                r_energy = self.DARP_steps[-1].drones_energy[r] - ( generation_factor * consumption_factor ) 

                while r_energy < 0 :
                    generation_factor = random.randint(0,factor)
                    consumption_factor = random.randint(80,120) / 100 

                    r_energy = self.DARP_steps[-1].drones_energy[r] - ( generation_factor * consumption_factor ) 

                    #print("evaluation")
                    #print( generation_factor * consumption_factor )
                    #print(self.DARP_steps[-1].drones_energy[r])
            
            else : 

                r_energy = 0

            #if r == 2 : 
            #    r_energy = 0 
            #    generation_factor = 0 
            #else : 
            #    r_energy = r_energy +12

            drones_energy.append( r_energy )

            if r_energy != 0 : 
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
            
            else : 
                drones_pos.append( self.DARP_steps[-1].current_position[r] )

                print(self.DARP_steps[-1].step)
                performed_paths.append( [] )
                robots_offsets.append(  None  ) 

                if self.DARP_steps[-1].step == 0 : 
                    point = ( (self.start_positions[r] // self.cols) * 2 , (self.start_positions[r] % self.cols) *2 )
                    print( point )
                    drones_pos_precise.append( point ) 
                else : 
                    drones_pos_precise.append( self.DARP_steps[-1].current_position_precise[r]) 
            

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
        obstacles_list = []
        #obstacles_list = generateObstacles(drones_pos, 5, self.rows, self.cols)
        print("OBSTACLES LIST IS ", obstacles_list)
        input()
        self.addStep( drones_energy, drones_pos, drones_pos_precise, obstacles_list)

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

    def finalize_step(self, visualization = True, vis_real_time = False) : 
        print(" Resolution ")
        DARP_result = self.divide_regions_last_step( vis = vis_real_time )   #Change here for DARP visuals in real time


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


    def InsertinDB(self, replan = False) : 

        proxy = ri_copy.get_proxy_update()

        for i in range(len(self.drones_name)) :
                       
            drone_name = self.drones_name[i]

            #create point
            #point_name = str(self.start_positions[i]) #string concatenating coordinates of the point
            #ri_copy.create_point(proxy, point_name) 

            if replan == True : 

                #get old point
                #old_point_name = 

                #remove old predicate
                ri_copy.update_predicate( proxy, "current-location", [{"key": 'ecosub', "value": drone_name},{"key": 'waypoint3d', "value": old_point_name} ],2)  # [drone_name, old_point_name]

                #remove old value (do we need the correct value for that ? )
                ri_copy.update_function( proxy, "current-energy-level", [{"key": 'ecosub', "value": drone_name}], -1 , 2) 


        
            #make this point the current location of the robots
            #(current-location ?r1 - robot ?l1 - waypoint3d)
            #ri_copy.update_predicate( proxy, "current-location", [KeyValue('ecosub', drone_name), KeyValue('waypoint3d', point_name)], 0) #[drone_name, point_name] 


            #make energy level of robots
            #(current-energy-level ?r1 - robot)
            #add new value
            ri_copy.update_function( proxy, "current-energy-level", [KeyValue('ecosub', drone_name)], self.energy_level[i] , 0) 

        
        future_paths = self.DARP_steps[-1].future_paths
        print("HERE")
        print(future_paths)
        wc_count = 0

        for i in range((self.dronesNo)) : 

            drone_name = self.drones_name[i]
            waypoints_from_path = extract_waypoints(future_paths[i])

            print("WAYPOINT CHECK")
            print(future_paths[i])
            #print(waypoints_from_path)
                
            print("Waypoints are "+str(waypoints_from_path))

            count = 0
            pre_waypoint = None
            waypoint_hashmap = set()
            for waypoint in waypoints_from_path : 
                    
                    waypoint_name = "p_"+str(waypoint[0])+"_"+str(waypoint[1])
                    if waypoint_name in waypoint_hashmap : 
                        waypoint_name = "p"+str(waypoint_name)
                        waypoint_hashmap.add( "p"+str(waypoint_name) )

                    else : 
                        waypoint_hashmap.add( waypoint_name )


                    
                    ri_copy.create_point(proxy, waypoint_name) #HAS TO BE GPS COORDINATES
                    ri_copy.update_function( proxy, "altitude", [ KeyValue('waypoint3d', waypoint_name)], self.drones_altitude[i] , 0) 




                    if pre_waypoint!=None :
                        print(count)
                        distance = computeDistanceWaypoints(waypoint, pre_waypoint)
                        #ri_copy.update_function( proxy, "distance3D", [{"key": 'ecosub', "value": drone_name}], distance , 0) 
                        ri_copy.update_function( proxy, "distance3d", [ KeyValue('waypoint3d', pre_waypoint_name),  KeyValue('waypoint3d', waypoint_name)], distance , 0) 
                        ri_copy.update_predicate( proxy, "to-observe-line", [ KeyValue('waypoint3d', pre_waypoint_name),  KeyValue('waypoint3d', waypoint_name), KeyValue('Sensor', 'sensor_'+str(drone_name))],0  )
                        ri_copy.update_instance(proxy, 'water_current', "wc"+str(wc_count), 0 )
                        ri_copy.update_predicate( proxy, "current-profile", [ KeyValue('water_current', "wc"+str(wc_count)), KeyValue('waypoint3d', pre_waypoint_name),  KeyValue('waypoint3d', waypoint_name)],0  )
                        ri_copy.update_function( proxy, "current-force", [ KeyValue('water_current', "wc"+str(wc_count))], 0 , 0) 

                        wc_count = wc_count +1 

                    else : 
                        #old_waypoint_name = 
                        #ri_copy.update_predicate( proxy, "current-location", [KeyValue('ecosub', drone_name), KeyValue('waypoint3d', old_waypoint_name)], 2) #[drone_name, point_name] 
                        ri_copy.update_predicate( proxy, "current-location", [KeyValue('ecosub', drone_name), KeyValue('waypoint3d', waypoint_name)], 0) #[drone_name, point_name] 



                    pre_waypoint = waypoint
                    pre_waypoint_name = waypoint_name
                    count = count +1 

            #previous_goal_location = 
            #ri_copy.update_predicate( proxy, "current-location", [KeyValue('ecosub', drone_name), KeyValue('waypoint3d', previous_goal_location)], 3)  
            ri_copy.update_predicate( proxy, "current-location", [KeyValue('ecosub', drone_name), KeyValue('waypoint3d', waypoint_name)], 1)  




    def callRosPlanExecution(self) : 

        mission_success = self.ROSplan_obj.problem_to_plan()


        return


def computeDistanceWaypoints(waypoint_1, waypoint_2) : 
    return math.sqrt( (waypoint_1[0] - waypoint_2[0]) **2 + (waypoint_1[1] - waypoint_2[1]) **2   )

def extract_waypoints(path) : 
    waypoint_list = []
    current_waypoint = ( path[0][0], path[0][1] )
    waypoint_list.append( current_waypoint ) 
    direction = None
    for double_waypoint in path : 

        current_waypoint = ( double_waypoint[0], double_waypoint[1] )
        next_waypoint = ( double_waypoint[2], double_waypoint[3] )

        if direction == None : 
            direction = ( current_waypoint[0] - next_waypoint[0], current_waypoint[1] - next_waypoint[1] )

        else : 
             new_direction = ( current_waypoint[0] - next_waypoint[0], current_waypoint[1] - next_waypoint[1] )

             if direction != new_direction : 
                 
                 waypoint_list.append(current_waypoint)
                 direction = new_direction

        #current_waypoint = next_waypoint
        #print(direction)
    return waypoint_list

def gridtoGPS(x,y): 

    return

def GPStogrid(x,y) : 
    return

def transformCoordinates(x,y) : 
     new_x = int( x/ 2)
     new_y = int (y / 2 )
     return new_x, new_y


def extract_waypoint(path) : 

    waypoints = [] 




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


def generateObstacles( robots_start_pos_list,nb_obstacles, rows, cols) : 
    
    obstacle_list = []
    x = np.random.randint(0,rows)
    y = np.random.randint(0, cols)
    count = nb_obstacles
    test_value = True
    while (test_value and count > 0)  :
        print(x,y)
        print(count)
        x = np.random.randint(0,rows)
        y = np.random.randint(0, cols)

        if not ( PositionTransformer(x,y, rows, cols) in (robots_start_pos_list + obstacle_list) ) : 
            obstacle_list.append( PositionTransformer(x,y,rows, cols))
            count = count -1

        test_value = PositionTransformer(x,y, rows, cols) in (robots_start_pos_list + obstacle_list)

    return obstacle_list




if __name__ == '__main__':
    nx, ny, dronesNo, initial_positions, obs_pos, drones_energy = generate_instance_initial_random(10,10,3)

    nx, ny = 8, 4 
    initial_positions =  [12, 3, 7]
    obs_pos =  [23, 0, 4, 29]
    print("Instance is "+str(drones_energy))
    print ("Energy \t", " Initial Positions \t", " Obstacles Positions \t")
    print( (drones_energy, initial_positions, obs_pos) )
    DARP_instance_obj = DARP_instance(nx, ny, initial_positions, obs_pos, 1 )
    DARP_instance_obj.energy_level = drones_energy
    DARP_instance_obj.createInitialStep()
    DARP_instance_obj.finalize_step(vis_real_time=True)

    # DARP_instance_obj.drones_name = ["ecosub-1", "ecosub-2", "ecosub-3"]
    # DARP_instance_obj.drones_altitude=[10,10,25]
    # DARP_instance_obj.InsertinDB()
    # DARP_instance_obj.callRosPlanExecution()
    exit(1)
    while True : 
        input()
        performed_paths = DARP_instance_obj.generateNewStep()

        DARP_instance_obj.PreProcessSolveStep()
    
        DARP_instance_obj.finalize_step()



    
                

#path = [(22, 24, 23, 24), (23, 24, 24, 24), (24, 24, 25, 24), (25, 24, 26, 24), (26, 24, 27, 24), (27, 24, 28, 24), (28, 24, 29, 24), (29, 24, 30, 24), (30, 24, 31, 24), (31, 24, 32, 24), (32, 24, 33, 24), (33, 24, 34, 24), (34, 24, 35, 24), (35, 24, 36, 24), (36, 24, 37, 24), (37, 24, 38, 24), (38, 24, 39, 24), (39, 24, 40, 24), (40, 24, 41, 24), (41, 24, 42, 24), (42, 24, 43, 24), (43, 24, 44, 24), (44, 24, 45, 24), (45, 24, 46, 24), (46, 24, 47, 24), (47, 24, 48, 24), (48, 24, 49, 24), (49, 24, 50, 24), (50, 24, 51, 24), (51, 24, 52, 24), (52, 24, 53, 24), (53, 24, 54, 24), (54, 24, 55, 24), (55, 24, 56, 24), (56, 24, 57, 24), (57, 24, 58, 24), (58, 24, 59, 24), (59, 24, 60, 24), (60, 24, 61, 24), (61, 24, 62, 24), (62, 24, 62, 23), (62, 23, 61, 23), (61, 23, 60, 23), (60, 23, 60, 22), (60, 22, 61, 22), (61, 22, 62, 22), (62, 22, 62, 21), (62, 21, 61, 21), (61, 21, 60, 21), (60, 21, 60, 20), (60, 20, 61, 20), (61, 20, 62, 20), (62, 20, 62, 19), (62, 19, 61, 19), (61, 19, 60, 19), (60, 19, 60, 18), (60, 18, 61, 18), (61, 18, 62, 18), (62, 18, 62, 17), (62, 17, 61, 17), (61, 17, 60, 17), (60, 17, 60, 16), (60, 16, 61, 16), (61, 16, 62, 16), (62, 16, 62, 15), (62, 15, 61, 15), (61, 15, 60, 15), (60, 15, 60, 14), (60, 14, 61, 14), (61, 14, 62, 14), (62, 14, 62, 13), (62, 13, 61, 13), (61, 13, 60, 13), (60, 13, 60, 12), (60, 12, 61, 12), (61, 12, 62, 12), (62, 12, 62, 11), (62, 11, 61, 11), (61, 11, 60, 11), (60, 11, 60, 10), (60, 10, 61, 10), (61, 10, 62, 10), (62, 10, 62, 9), (62, 9, 61, 9), (61, 9, 60, 9), (60, 9, 60, 8), (60, 8, 61, 8), (61, 8, 62, 8), (62, 8, 62, 7), (62, 7, 62, 6), (62, 6, 63, 6), (63, 6, 63, 7), (63, 7, 63, 8), (63, 8, 63, 9), (63, 9, 63, 10), (63, 10, 63, 11), (63, 11, 63, 12), (63, 12, 63, 13), (63, 13, 63, 14), (63, 14, 63, 15), (63, 15, 63, 16), (63, 16, 63, 17), (63, 17, 63, 18), (63, 18, 63, 19), (63, 19, 63, 20), (63, 20, 63, 21), (63, 21, 63, 22), (63, 22, 63, 23), (63, 23, 63, 24), (63, 24, 63, 25), (63, 25, 63, 26), (63, 26, 63, 27), (63, 27, 63, 28), (63, 28, 63, 29), (63, 29, 63, 30), (63, 30, 63, 31), (63, 31, 62, 31), (62, 31, 61, 31), (61, 31, 60, 31), (60, 31, 60, 30), (60, 30, 61, 30), (61, 30, 62, 30), (62, 30, 62, 29), (62, 29, 61, 29), (61, 29, 60, 29), (60, 29, 60, 28), (60, 28, 61, 28), (61, 28, 62, 28), (62, 28, 62, 27), (62, 27, 61, 27), (61, 27, 60, 27), (60, 27, 60, 26), (60, 26, 61, 26), (61, 26, 62, 26), (62, 26, 62, 25), (62, 25, 61, 25), (61, 25, 60, 25), (60, 25, 59, 25), (59, 25, 58, 25), (58, 25, 57, 25), (57, 25, 56, 25), (56, 25, 55, 25), (55, 25, 54, 25), (54, 25, 53, 25), (53, 25, 52, 25), (52, 25, 51, 25), (51, 25, 50, 25), (50, 25, 49, 25), (49, 25, 48, 25), (48, 25, 47, 25), (47, 25, 46, 25), (46, 25, 45, 25), (45, 25, 44, 25), (44, 25, 43, 25), (43, 25, 42, 25), (42, 25, 41, 25), (41, 25, 40, 25), (40, 25, 39, 25), (39, 25, 38, 25), (38, 25, 37, 25), (37, 25, 36, 25), (36, 25, 35, 25), (35, 25, 34, 25), (34, 25, 33, 25), (33, 25, 32, 25), (32, 25, 31, 25), (31, 25, 30, 25), (30, 25, 29, 25), (29, 25, 28, 25), (28, 25, 27, 25), (27, 25, 26, 25), (26, 25, 25, 25), (25, 25, 24, 25), (24, 25, 23, 25), (23, 25, 22, 25), (22, 25, 22, 24)]
#[(22, 24), (62, 23), (61, 23), (60, 22), (61, 22), (62, 21), (61, 21), (60, 20), (61, 20), (62, 19), (61, 19), (60, 18), (61, 18), (62, 17), (61, 17), (60, 16), (61, 16), (62, 15), (61, 15), (60, 14), (61, 14), (62, 13), (61, 13), (60, 12), (61, 12), (62, 11), (61, 11), (60, 10), (61, 10), (62, 9), (61, 9), (60, 8), (61, 8), (62, 7), (63, 6), (63, 7), (62, 31), (60, 30), (61, 30), (62, 29), (61, 29), (60, 28), (61, 28), (62, 27), (61, 27), (60, 26), (61, 26), (62, 25), (61, 25), (22, 24)]