import rospy
from rosplan_knowledge_msgs.msg import KnowledgeItem
from std_srvs.srv import Empty
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest, GetDomainPredicateDetailsService
from diagnostic_msgs.msg import KeyValue
import os 


def update_function( proxy, function_name, params, value, update_type) :
    req = KnowledgeUpdateServiceRequest()
    req.knowledge.knowledge_type = KnowledgeItem.FUNCTION
    req.knowledge.values = params
    req.knowledge.attribute_name = function_name
    req.knowledge.function_value = value
    req.update_type = update_type

    success = proxy(req).success

    return success

    
def update_predicate( proxy, pred_name, params, update_type) :
    req = KnowledgeUpdateServiceRequest()
    req.knowledge.knowledge_type = KnowledgeItem.FACT
    req.knowledge.values.extend(params)
    req.knowledge.attribute_name = pred_name
    req.update_type = update_type

    success = proxy(req).success

    return success
        
def update_instance(proxy, ins_type, ins_name ,update_type) : 
    req = KnowledgeUpdateServiceRequest()
    req.knowledge.knowledge_type = KnowledgeItem.INSTANCE
    req.knowledge.instance_type = ins_type
    req.knowledge.instance_name = ins_name
    req.update_type = update_type

    success = proxy(req).success
    return success



def get_proxy_update() : 
    rospy.loginfo('Waiting for /rosplan_knowledge_base/update ...')
    rospy.wait_for_service('/rosplan_knowledge_base/update')
    knowledge_update_proxy = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
    return knowledge_update_proxy

def get_proxy_details():
    rospy.loginfo('Waiting for /rosplan_knowledge_base/domain/predicate_details ...')
    rospy.wait_for_service('/rosplan_knowledge_base/domain/predicate_details')
    predicate_detail_proxy = rospy.ServiceProxy('/rosplan_knowledge_base/domain/predicate_details', GetDomainPredicateDetailsService)
    return predicate_detail_proxy

def create_point(proxy, point_name) : 
    print("Creating a waypoint3D of name "+point_name)
    update_instance(proxy, 'waypoint3d', str(point_name), 0 )

class Rosplan_object :

    def __init__(self) -> None:
        #rospy.init_node("rosplan_interface_soar")
        rospy.loginfo('Executive started')

        print("HEREEEEEE")
        self.rosplan_services()
        return
    

    def rosplan_services(self):

        # Service proxies: Problem Generation, Planning, Parsing, Dispatching
        rospy.loginfo('Waiting for rosplan services...')
        rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
        rospy.loginfo('Rosplan services initialized...')

        self._problem_proxy = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
        rospy.wait_for_service('/rosplan_planner_interface/planning_server')
        rospy.loginfo('Rosplan services initialized...')

        self._planner_proxy = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
        rospy.loginfo('Rosplan services initialized...')


        rospy.wait_for_service('/rosplan_knowledge_base/clear')

        self._clear_KB_proxy = rospy.ServiceProxy('/rosplan_knowledge_base/clear', Empty)
        rospy.loginfo('Rosplan services initialized...')



    def problem_to_plan(self):

        mission_success = True 
        rospy.loginfo('Generating problem file ...')

        rospy.sleep(0.5)

        self._problem_proxy()

        rospy.sleep(0.5)

        VerifyDoubleLines("/home/benjies/Workspace_c/SoaR_pipeline/generated_problem.pddl","/home/benjies/Workspace_c/SoaR_pipeline/generated_corrected_problem.txt")


        rospy.loginfo('Planning ...')

        try:

            self._planner_proxy()
        except:

            rospy.logwarn('Planning attempt failed')

            mission_success=False

        return mission_success


def VerifyDoubleLines(path, path_corrected) :

    rospy.loginfo("Cleaning problem file")

    core_name_path = path.split('.')[0]
    new_name = str(core_name_path)+'.txt'
    os.rename(path, new_name)
    stored_lines = []
    interval = False
    with open(new_name, "r") as original_problem_reader : 

        with open(path_corrected, "w") as corrected_problem_writer :

            for line in original_problem_reader.readlines() : 

                if "(:objects" in line: 
                    interval = True 
                elif "(:init" in line :
                    interval = False
                    

                if interval == True :
                    line_stripped = line.strip()
                    if (not (line_stripped in stored_lines)) : 
                        #print(line)
                        corrected_problem_writer.write(line)
                        stored_lines.append(line_stripped)
                    else : 
                        #print(line)
                        print("ALREADY IN FILE")
                else : 
                    corrected_problem_writer.write(line)

    core_path_corrected = path_corrected.split('.')[0]
    os.rename(path_corrected, str(core_path_corrected)+'.pddl')
    os.rename(new_name, path)

    rospy.loginfo("Corrected problem file generated")

    
    return

