import rospy
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest, GetDomainPredicateDetailsService
from diagnostic_msgs.msg import KeyValue


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
    print("Creating a waypoint3d of name "+point_name)
    update_instance(proxy, 'waypoint3D', point_name, 0 )