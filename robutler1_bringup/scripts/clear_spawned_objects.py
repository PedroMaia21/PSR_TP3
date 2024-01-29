#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import DeleteModel
import rosservice

def clear_objects(models_to_delete):
    rospy.init_node('clear_objects', anonymous=True)

    # Get the list of available models from the gazebo/get_world_properties service
    try:
        models = rospy.ServiceProxy('/gazebo/get_world_properties', rosservice.get_service_class_by_name('/gazebo/get_world_properties'))().model_names
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))
        return

    # Iterate through each model and delete it if its base name is in the models_to_delete dictionary
    for model_name in models:
        base_name = model_name.split('-')[0]  # Extract the base name before the UUID
        if base_name in models_to_delete:
            try:
                delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
                delete_model(model_name)
                rospy.loginfo("Deleted model: %s", model_name)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", str(e))

    rospy.loginfo("Selected objects cleared.")

if __name__ == '__main__':
    # Specify the base names of the models you want to delete in the set
    base_names_to_delete = {
        'sphere_r',
        'bottle_red_wine',
        'coca_cola',
        'cube_b',
        'human_female_4',
        'human_male_4',
        'laptop_pc_1'
    }

    clear_objects(base_names_to_delete)
