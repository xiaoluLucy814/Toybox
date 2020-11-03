#!/usr/local/bin/python

import argparse
import glob
import os
import shutil
import xml.etree.ElementTree as ET

import numpy as np

import trimesh

dataset_root = '/Users/liuzeyi/Desktop/sapien/partnet-mobility-v0'
density = 20.
scale = 1.#.15/2 # convert 2 normalized units to .15m

def load_mesh(obj_path, scale=1.0):
    '''
    Convert a possible scene to a mesh.
    If conversion occurs, the returned mesh has only vertex and face data.
    Modified from: https://github.com/mikedh/trimesh/issues/507
    '''
    scene_or_mesh = trimesh.load(obj_path)

    mesh = None
    if isinstance(scene_or_mesh, trimesh.Scene):
        if len(scene_or_mesh.geometry) == 0:
            mesh = None  # empty scene
        else:
            # we lose texture information here
            mesh = trimesh.util.concatenate(
                tuple(trimesh.Trimesh(vertices=g.vertices * scale, faces=g.faces)
                    for g in scene_or_mesh.geometry.values()))
    else:
        assert(isinstance(scene_or_mesh, trimesh.Trimesh))
        mesh = trimesh.Trimesh(vertices=scene_or_mesh.vertices * scale, faces=scene_or_mesh.faces)

    return mesh

def urdf_link_scale(link, scale):
    inner_tags = ['collision', 'visual']
    for tag in inner_tags:
        for part in link.findall(tag):
            for geometry in part.findall('geometry'):
                for mesh in geometry.findall('mesh'):
                    mesh.set('scale', '{} {} {}'.format(scale, scale, scale))

def urdf_link_add_inertia(link, mass, inertia_tensor):
    inertial_tag = ET.SubElement(link, 'inertial')
    mass_tag = ET.SubElement(inertial_tag, 'mass')
    mass_tag.set('value', str(mass))
    inertia_tag = ET.SubElement(inertial_tag, 'inertia')
    inertia_tag.set('ixx', str(inertia_tensor[0, 0]))
    inertia_tag.set('ixy', str(inertia_tensor[0, 1]))
    inertia_tag.set('ixz', str(inertia_tensor[0, 2]))
    inertia_tag.set('iyy', str(inertia_tensor[1, 1]))
    inertia_tag.set('iyz', str(inertia_tensor[1, 2]))
    inertia_tag.set('izz', str(inertia_tensor[2, 2]))

def create_partnet_inertia_mobility(dataset_root, density, scale=1.0):
    # set of bad object instance in partnet_mobility
    # (1) missing mesh referenced in urdf
    # (2) could not create watertight mesh for all parts of the object
    bad_ids = set()

    # create an archive for bad objects
    object_archive_path = os.path.join(dataset_root, '!ARCHIVE')
    if not os.path.exists(object_archive_path):
        os.mkdir(object_archive_path)

    for object_id in os.listdir(dataset_root):
        # make sure we are looking at some object
        if not object_id.isdigit():
            continue

        # parse urdf into in memory xml
        tree = ET.parse(os.path.join(dataset_root, object_id, 'mobility.urdf'))

        # create a dir to dump the scaled link hulls
        link_hulls_dir = os.path.join(dataset_root, object_id, 'scaled_link_hulls')
        if not os.path.exists(link_hulls_dir):
            os.mkdir(link_hulls_dir)

        # create a dir to dump the scaled link hulls
        scaled_obj_dir = os.path.join(dataset_root, object_id, 'scaled_textured_objs')
        if not os.path.exists(scaled_obj_dir):
            os.mkdir(scaled_obj_dir)

        # copy over textures
        for file in glob.glob(os.path.join(dataset_root, object_id, 'textured_objs', '*.mtl')):
            shutil.copy(file, scaled_obj_dir)

        for link_index, link in enumerate(tree.findall('link')):
            urdf_link_add_inertia(link, 1, np.zeros([3, 3]))
            continue
            part_trimeshes = []
            # print(f'start, link_id = {link_index}')
            # associate all meshes with their link
            for part in link.findall('collision'):
                for geometry in part.findall('geometry'):
                    for mesh in geometry.findall('mesh'):
                        part_trimesh = None
                        try:
                            # try to open all objs associated with a link
                            obj_path = os.path.join(dataset_root, object_id, mesh.attrib['filename'])
                            part_trimesh = load_mesh(obj_path, scale)

                        except:
                            print('could not find file {0}'.format(obj_path))
                            bad_ids.add(object_id)
                        if part_trimesh:
                            part_trimeshes.append(part_trimesh)

            part_mesh = None

            if len(part_trimeshes) > 1:
                # aggregate mesh fragments into a part
                vertice_list = [m.vertices for m in part_trimeshes]
                faces_list = [m.faces for m in part_trimeshes]
                faces_offset = np.cumsum([v.shape[0] for v in vertice_list])
                faces_offset = np.insert(faces_offset, 0, 0)[:-1]
                vertices = np.vstack(vertice_list)
                faces = np.vstack([face + offset for face, offset in zip(faces_list, faces_offset)])
                part_mesh = trimesh.Trimesh(vertices, faces)
            elif len(part_trimeshes) == 1:
                part_mesh = part_trimeshes[0]
            else:
                urdf_link_add_inertia(link, 0, np.zeros([3, 3]))
                continue

            # print(f'here, link_id = {link_index}')
            # find convex hull, which is a coarse approximation for the object shape
            part_hull_mesh = part_mesh.convex_hull

            # if not part_hull_mesh.is_watertight:
            #     bad_ids.add(object_id)
            #     continue

            part_hull_mesh.export(os.path.join(link_hulls_dir, 'link_' + str(link_index) + '.obj'))

            # get an approximation for the inertia matrix and mass based on the convex hull
            moment = part_hull_mesh.moment_inertia
            mass = density * part_hull_mesh.volume

            urdf_link_scale(link, scale)
            urdf_link_add_inertia(link, mass, moment)

        # write modifed urdf to an inertial version
        tree.write(os.path.join(dataset_root, object_id, 'mobility_new.urdf'))

    # move the bad datasets to the !ARCHIVE
    for object_id in bad_ids:
        shutil.move(os.path.join(dataset_root, object_id), object_archive_path)

def copy_mobility(new_path):
    import utils
    import json
    utils.mkdir(new_path)
    old_path = os.environ['MOBILITY']
    with open('assets/mobility_meta.json', 'r') as f:
        mobility_meta = json.load(f)
    for id_list in mobility_meta.values():
        for id in id_list:
            shutil.copytree(
                os.path.join(old_path, id),
                os.path.join(new_path, id)
            )

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Create articulated object dataset.')
    parser.add_argument('--partnet', action='store', type=str, default=dataset_root)
    parser.add_argument('--density', action='store', type=float, default=20)
    parser.add_argument('--scale', action='store', type=float, default=1.0)

    args = parser.parse_args()
    print(args)

    # copy the dataset
    # copy_mobility(args.partnet)

    create_partnet_inertia_mobility(args.partnet, args.density, args.scale)
