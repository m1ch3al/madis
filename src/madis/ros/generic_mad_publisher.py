import rospy
import yaml
import sys
from madis.utils.conf_utils import read_sensors_configuration
from madis.utils.introspection import instantiate_class
from std_msgs.msg import String


def mad_generic_node(role_name, yaml_mad_configuration):
    with open(yaml_mad_configuration, "r") as yaml_file_descriptor:
        initial_configuration = yaml.safe_load(yaml_file_descriptor)
    yaml_file_descriptor.close()
    ros_node_configuration = initial_configuration["ros-nodes"][role_name]
    sensor_name = ros_node_configuration["sensor-name"]
    sensors = read_sensors_configuration(yaml_mad_configuration, initial_configuration)
    selected_sensor_configuration = sensors[sensor_name]
    module_name = selected_sensor_configuration["driver-module"]
    class_name = selected_sensor_configuration["driver-class"]
    sensor_reader = instantiate_class(module_name, class_name)
    sensor_reader.initialize_sensor()
    mad_pubisher = rospy.Publisher(ros_node_configuration["ros-topic-name"], String, queue_size=10)
    rospy.init_node(ros_node_configuration["ros-node-name"])
    rate = rospy.Rate(ros_node_configuration["ros-rate"])
    while not rospy.is_shutdown():
        json_data_from_sensor = sensor_reader.read_from_sensor()
        mad_pubisher.publish(json_data_from_sensor)
        rate.sleep()


def main():
    role_name = sys.argv[1]
    mad_configuration_filepath = sys.argv[2]
    print(role_name)
    print(mad_configuration_filepath)

    try:
        mad_generic_node(role_name, mad_configuration_filepath)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
