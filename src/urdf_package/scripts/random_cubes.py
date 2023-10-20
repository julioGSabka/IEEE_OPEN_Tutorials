#!/usr/bin/env python3

import rospy
import os
import random
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom

def create_cube_model(cubo_numero, origem):

    model = ET.Element('model', {'name': 'cube' + str(cubo_numero)})
    pose = ET.SubElement(model, 'pose')
    pose.text = f"{origem[0]} {origem[1]} 0.03 0 0 0"
    link = ET.SubElement(model, 'link', {'name': 'cube' + str(cubo_numero)})

    # Inertial element
    inertial = ET.SubElement(link, 'inertial')
    mass = ET.SubElement(inertial, 'mass')
    mass.text = '0.06'
    inertia = ET.SubElement(inertial, 'inertia')
    ixx = ET.SubElement(inertia, 'ixx')
    ixx.text = '0.000036'
    ixy = ET.SubElement(inertia, 'ixy')
    ixy.text = '0'
    ixz = ET.SubElement(inertia, 'ixz')
    ixz.text = '0'
    iyy = ET.SubElement(inertia, 'iyy')
    iyy.text = '0.000036'
    iyz = ET.SubElement(inertia, 'iyz')
    iyz.text = '0'
    izz = ET.SubElement(inertia, 'izz')
    izz.text = '0.000036'
    pose_inertial = ET.SubElement(inertial, 'pose')
    pose_inertial.text = '0 0 0 0 0 0'

    # Collision element
    collision = ET.SubElement(link, 'collision', {'name': 'collision'})
    geometry_collision = ET.SubElement(collision, 'geometry')
    mesh_collision = ET.SubElement(geometry_collision, 'mesh')
    uri_collision = ET.SubElement(mesh_collision, 'uri')
    uri_collision.text = f"/home/jetson/open_workspace/src/urdf_package/models/Cubo_{cubo_numero}.dae"
    max_contacts = ET.SubElement(collision, 'max_contacts')
    max_contacts.text = '10'
    surface = ET.SubElement(collision, 'surface')
    contact = ET.SubElement(surface, 'contact')
    ode_contact = ET.SubElement(contact, 'ode')

    # Visual element
    visual = ET.SubElement(link, 'visual', {'name': 'visual'})
    geometry_visual = ET.SubElement(visual, 'geometry')
    mesh_visual = ET.SubElement(geometry_visual, 'mesh')
    uri_visual = ET.SubElement(mesh_visual, 'uri')
    uri_visual.text = f"//home/jetson/open_workspace/src/urdf_package/models/Cubo_{cubo_numero}.dae"

    # Other elements
    self_collide = ET.SubElement(link, 'self_collide')
    self_collide.text = '0'
    enable_wind = ET.SubElement(link, 'enable_wind')
    enable_wind.text = '0'
    kinematic = ET.SubElement(link, 'kinematic')
    kinematic.text = '0'
    
    return model

def prettify(elem):
    """Retorna uma string formatada em XML para um elemento ou árvore XML."""
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return '\n'.join([line for line in reparsed.toprettyxml(indent="  ").split('\n') if line.strip()])

def random_cubes_node(original_file_path, new_file_path):
    # Número de cubos que você deseja selecionar
    num_cubos_selecionados = 20

    origens_selecionadas = [
        (0.244, 0.075),
        (0.244, -0.075),
        (0.244, 0.244),
        (0.244, 0.394),
        (0.394, 0.075),
        (0.394, -0.075),
        (0.394, 0.244),
        (0.394, 0.394),
        (0.563, 0.075),
        (0.563, -0.075),
        (0.563, 0.244),
        (0.563, 0.394),
        (0.713, 0.075),
        (0.713, -0.075),
        (0.713, 0.244),
        (0.713, 0.394),
        (-0.244, 0.075),
        (-0.244, -0.075),
        (-0.244, 0.244),
        (-0.244, 0.394),
        (-0.394, 0.075),
        (-0.394, -0.075),
        (-0.394, 0.244),
        (-0.394, 0.394),
        (-0.563, 0.075),
        (-0.563, -0.075),
        (-0.563, 0.244),
        (-0.563, 0.394),
        (-0.713, 0.075),
        (-0.713, -0.075),
        (-0.713, 0.244),
        (-0.713, 0.394),
    ]

    if len(origens_selecionadas) < num_cubos_selecionados:
        raise ValueError("Não há coordenadas de origens suficientes para selecionar o número de cubos desejados.")

    # Sorteando quais coordenadas de origens serão selecionadas
    origens_selecionadas = random.sample(origens_selecionadas, num_cubos_selecionados)

    # Lista de números de 1 a 22 representando os nomes dos arquivos xacro
    lista_de_cubos = list(range(1, 31))

    # Sorteando quais cubos serão selecionados
    cubos_selecionados = random.sample(lista_de_cubos, num_cubos_selecionados)

    # Lendo o conteúdo do arquivo original_file_path
    with open(original_file_path, 'r') as file:
        content = file.read()
        root = ET.fromstring(content)

    world_element = root.find(".//world")

    # Adicionando os modelos de cubo diretamente no nível raiz do arquivo
    for cubo_numero, origem in zip(cubos_selecionados, origens_selecionadas):
        cube_model = create_cube_model(cubo_numero, origem)
        world_element.append(cube_model)

    with open(new_file_path, 'w') as new_file:
        new_file.write(prettify(root))

if __name__ == '__main__':
    try:
        rospy.init_node('random_cubes_node', anonymous=True)
        original_file_path = "/home/jetson/open_workspace/src/urdf_package/worlds/default.world"
        new_file_path = "/home/jetson/open_workspace/src/urdf_package/worlds/arena_cubes.world"

        random_cubes_node(original_file_path, new_file_path)
        print(f"Arquivo {new_file_path} criado com sucesso.")
    except Exception as e:
        print(f"Erro ao criar o arquivo: {e}")
