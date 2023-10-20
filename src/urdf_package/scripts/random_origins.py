#!/usr/bin/env python3

import rospy
import random
import xml.etree.ElementTree as ET

def generate_random_coordinate():
    # Supondo que a lista de coordenadas é uma lista de tuplas (x, y)
    coord_list = [
        (0.0, 0.0),
        (0.0, 0.319),
        (0.0, 0.638),
        (0.0, 0.957),
        (0.0, -0.319),
        (0.0, -0.638),
        (0.319, 0.957),
        (0.319, 0.638),
        (0.319, -0.319),
        (0.319, -0.638),
        (0.638, 0.957),
        (0.638, 0.638),
        (0.638, -0.319),
        (0.638, -0.638),
        (0.957, 0.957),
        (0.957, 0.638),
        (0.957, 0.319),
        (0.957, 0.0),
        (0.957, -0.319),
        (0.957, -0.638),
        (-0.319, 0.957),
        (-0.319, 0.638),
        (-0.319, -0.319),
        (-0.319, -0.638),
        (-0.638, 0.957),
        (-0.638, 0.638),
        (-0.638, -0.319),
        (-0.638, -0.638),
        (-0.957, 0.957),
        (-0.957, 0.638),
        (-0.957, 0.319),
        (-0.957, 0.0),
        (-0.957, -0.319),
        (-0.957, -0.638),
    ]

    # Seleciona uma coordenada aleatoriamente da lista
    random_coordinate = random.choice(coord_list)

    return random_coordinate

def create_launch(file_path):

    random_coord = generate_random_coordinate()

    launch_content = f'''<launch>
    <arg name="x" default="{random_coord[0]}" />
    <arg name="y" default="{random_coord[1]}" />
    <arg name="yaw" default="0"/>
</launch>
    '''

    with open(file_path, 'w') as launch_file:
        launch_file.write(launch_content)

def main():
    rospy.init_node('random_coordinate_generator', anonymous=True)

    file_path = "/home/jetson/open_workspace/src/urdf_package/launch/origin.launch"
    create_launch(file_path)
    
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
