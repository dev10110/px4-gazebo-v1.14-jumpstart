from lxml import etree
import random
import math
import copy
import argparse
from collections import namedtuple

class Tree:
  def __init__(self, id_in, pose_in, scale_in, mesh_num_in):
    self.id = 'tree' + str(id_in)
    self.pose = pose_in
    self.scale = scale_in
    self.mesh_num = mesh_num_in

class Cylinder:
    def __init__(self, id_, pose_, radius_, height_):
        self.id = 'cylinder' + str(id_)
        self.pose = pose_
        self.radius = radius_
        self.height = height_

class Cube:
    def __init__(self, id_, pose_, size_):
        self.id = 'cube' + str(id_)
        self.pose = pose_
        self.size = size_

class GenXML:
  def __init__(self):
    self.root = etree.Element('world',name='default')

    #add gui
    gui_el = etree.Element("gui")
    camera_el = etree.Element("camera", name="user_camera")
    pose_el = etree.Element("pose")
    pose_el.text = "-20 -30 40 0 0.66 0.83"
    camera_el.append(pose_el)
    gui_el.append(camera_el)
    self.root.append(gui_el)

    #add ground
    include_el = etree.Element('include')
    ground_el = etree.Element('uri')
    ground_el.text = 'model://ground_plane'
    include_el.append(ground_el)
    self.root.append(include_el)

    #add sun
    include_el = etree.Element('include')
    sun_el = etree.Element('uri')
    sun_el.text = 'model://sun'
    include_el.append(sun_el)
    self.root.append(include_el)

    #disable shadows
    scene_el= etree.Element('scene')
    shadows_el = etree.Element('shadows')
    shadows_el.text = '0'
    scene_el.append(shadows_el)
    self.root.append(scene_el)

    #create the forest base model
    model_el = etree.Element('model', name = 'forest')
    static_el = etree.Element('static')
    static_el.text = 'true'
    model_el.append(static_el)
    self.root.append(model_el)

    #plugins
    self.add_plugins()

    #physics
    self.add_physics()



  def add_tree_model(self, tree, models_type):

    name_in = tree.id
    pose_in = tree.pose
    scale_in = tree.scale
    mesh_num_in = tree.mesh_num

    link_el = etree.Element('link', name = name_in)

    pose_el = etree.Element('pose')
    pose_el.text = ''.join(str(e) + ' ' for e in pose_in)
    link_el.append(pose_el)

    visual_el = etree.Element('visual', name = 'visual')
    geometry_el = etree.Element('geometry')
    mesh_el = etree.Element('mesh')
    uri_el = etree.Element('uri')
    uri_el.text = 'file://' + models_type + '/Tree' + str(mesh_num_in) + '.dae'
    mesh_el.append(uri_el)
    scale_el = etree.Element('scale')
    scale_el.text = str(scale_in) + ' ' + str(scale_in) + ' ' + str(scale_in)
    mesh_el.append(scale_el)
    geometry_el.append(mesh_el)
    visual_el.append(geometry_el)
    link_el.append(visual_el)

    collision_el = etree.Element('collision', name = 'collision')
    collision_el.append(copy.deepcopy(geometry_el))
    contacts_el = etree.Element('max_contacts')
    contacts_el.text = '0'
    collision_el.append(contacts_el)
    link_el.append(collision_el)

    self.root.find('model').append(link_el)
  
  def add_cylinder_model(self, cylinder):

    name_in = cylinder.id
    pose_in = cylinder.pose

    link_el = etree.Element('link', name = name_in)

    pose_el = etree.Element('pose')
    pose_el.text = ''.join(str(e) + ' ' for e in pose_in)
    link_el.append(pose_el)

    visual_el = etree.Element('visual', name = 'visual')
    geometry_el = etree.Element('geometry')
    cylinder_el = etree.Element('cylinder')
    radius_el = etree.Element('radius')
    radius_el.text = str(cylinder.radius)
    length_el = etree.Element("length")
    length_el.text = str(cylinder.height)
    cylinder_el.append(radius_el)
    cylinder_el.append(length_el)
    geometry_el.append(cylinder_el)
    visual_el.append(geometry_el)
    link_el.append(visual_el)

    collision_el = etree.Element('collision', name = 'collision')
    collision_el.append(copy.deepcopy(geometry_el))
    contacts_el = etree.Element('max_contacts')
    contacts_el.text = '0'
    collision_el.append(contacts_el)
    link_el.append(collision_el)

    self.root.find('model').append(link_el)
  
  def add_cube_model(self, cube):

    name_in = cube.id
    pose_in = cube.pose

    link_el = etree.Element('link', name = name_in)

    pose_el = etree.Element('pose')
    pose_el.text = ''.join(str(e) + ' ' for e in pose_in)
    link_el.append(pose_el)

    visual_el = etree.Element('visual', name = 'visual')
    geometry_el = etree.Element('geometry')
    cube_el = etree.Element('box')
    size_el = etree.Element("size")
    size_el.text = f"{cube.size[0]} {cube.size[1]} {cube.size[2]}"
    cube_el.append(size_el)
    geometry_el.append(cube_el)
    visual_el.append(geometry_el)
    link_el.append(visual_el)

    collision_el = etree.Element('collision', name = 'collision')
    collision_el.append(copy.deepcopy(geometry_el))
    contacts_el = etree.Element('max_contacts')
    contacts_el.text = '0'
    collision_el.append(contacts_el)
    link_el.append(collision_el)

    self.root.find('model').append(link_el)

  def add_plugins(self):
    #start plugin section
    plugins_comment_el = etree.Comment(' Plugins ')
    self.root.append(plugins_comment_el)

    #add octomap plugin
    oct_plug_el = etree.Element('plugin', name = 'gazebo_octomap',
                                filename = 'librotors_gazebo_octomap_plugin.so')
    self.root.append(oct_plug_el)

    #add rotors interface plugin
    rotors_plug_el = etree.Element('plugin', name = 'ros_interface_plugin',
                                   filename =
                                   'librotors_gazebo_ros_interface_plugin.so')
    self.root.append(rotors_plug_el)

  def add_physics(self):
    #start physics section
    physics_comment_el = etree.Comment(' Physics ')
    self.root.append(physics_comment_el)

    #physics
    physics_el = etree.Element('physics', name = 'default_physics',
                               default = '0', type = 'ode')
    max_step_size_el = etree.SubElement(physics_el, 'max_step_size')
    max_step_size_el.text = '0.004'
    real_time_factor_el = etree.SubElement(physics_el, 'real_time_factor')
    real_time_factor_el.text = '1'
    real_time_update_rate_el = etree.SubElement(physics_el,
                                                'real_time_update_rate')
    real_time_update_rate_el.text = '250'
    gravity_el = etree.SubElement(physics_el, 'gravity')
    gravity_el.text = '0 0 -9.8'
    magnetic_field_el = etree.SubElement(physics_el, 'magnetic_field')
    magnetic_field_el.text = '6e-06 2.3e-05 -4.2e-05'

    #ode
    ode_el = etree.SubElement(physics_el, 'ode')
    #solver
    solver_el = etree.SubElement(ode_el, 'solver')
    type_el = etree.SubElement(solver_el, 'type')
    type_el.text = 'quick'
    iters_el = etree.SubElement(solver_el, 'iters')
    iters_el.text = '1000'
    sor_el = etree.SubElement(solver_el, 'sor')
    sor_el.text = '1.3'
    use_dynamic_moi_rescaling = etree.SubElement(solver_el,
                                                 'use_dynamic_moi_rescaling')
    use_dynamic_moi_rescaling.text = '0'
    #constraints
    constraints_el = etree.SubElement(ode_el, 'constraints')
    cfm_el = etree.SubElement(constraints_el, 'cfm')
    cfm_el.text = '0'
    erp_el = etree.SubElement(constraints_el, 'erp')
    erp_el.text = '0.2'
    contact_max_correcting_vel_el = etree.SubElement(constraints_el,
                                                  'contact_max_correcting_vel')
    contact_max_correcting_vel_el.text = '100'
    contact_surface_layer_el = etree.SubElement(constraints_el,
                                                  'contact_surface_layer')
    contact_surface_layer_el.text = '0.001'
    self.root.append(physics_el)

  def output_xml(self):
    sdf = etree.Element('sdf',version='1.4')
    sdf.append(self.root)

    return '<?xml version="1.0"?>\n' + etree.tostring(sdf, pretty_print=True, encoding="unicode")

class World:
  TOTAL_NUM_MESHES = 6

  def __init__(self, world_length_x, world_length_y, use_high_res):
    self.world_length_x = world_length_x
    self.world_length_y = world_length_y
    self.trees = []
    self.cylinders = []
    self.cubes = []

    if(use_high_res):
      self.models_type = 'models_high_res'
    else:
      self.models_type = 'models_low_res'

  def _gen_random_tree(self):
    id_num = len(self.trees)
    x = random.uniform(-self.world_length_x/2, self.world_length_x/2)
    y = random.uniform(-self.world_length_y/2, self.world_length_y/2)
    angle = random.uniform(0, 2*math.pi)
    scale = random.uniform(0.3, 1)
    mesh_num = random.randint(1, self.TOTAL_NUM_MESHES)

    return Tree(id_num, [x,y,0,0,0,angle], scale, mesh_num)

  def _gen_random_cylinder(self):
    id_num = len(self.cylinders)
    x = random.uniform(-self.world_length_x/2, self.world_length_x/2)
    y = random.uniform(-self.world_length_y/2, self.world_length_y/2)
    radius = random.uniform(0.2, 0.5)
    height = random.uniform(2*radius, 4)
    z = height/2

    return Cylinder(id_num, [x,y,z,0,0,0], radius, height)

  def add_trees(self, num_trees):
    for i in range(num_trees):
      self.trees.append(self._gen_random_tree())

  def add_cylinders(self, num_cylinders):
      for i in range(num_cylinders):
          self.cylinders.append(self._gen_random_cylinder())

  def add_bounding_cubes(self):

      height = 4.0
      width = 0.25

      # block off along the x-axis
      cube_p = Cube(0, [0,  self.world_length_y/2, height/2, 0, 0, 0], [self.world_length_x, width, height])
      cube_n = Cube(1, [0, -self.world_length_y/2, height/2, 0, 0, 0], [self.world_length_x, width, height])

      self.cubes.append(cube_p)
      self.cubes.append(cube_n)

      # add a cube for the floor
      cube_f = Cube(2, [0, 0, width/2, 0, 0, 0], [self.world_length_x, self.world_length_y, width])
      self.cubes.append(cube_f)

  def save_world(self, filename):

    xml = GenXML()
    for tree in self.trees:
      xml.add_tree_model(tree, self.models_type)

    for cylinder in self.cylinders:
      xml.add_cylinder_model(cylinder)

    for cube in self.cubes:
      xml.add_cube_model(cube)

    text_file = open(filename, "w")
    text_file.write(xml.output_xml())
    text_file.close()

def gen_worlds(save_path, num_worlds, world_length_x, world_length_y, num_trees, num_cylinders, use_high_res):
  for i in range(num_worlds):
    world = World(world_length_x, world_length_y, use_high_res)
    world.add_trees(num_trees)
    world.add_cylinders(num_cylinders)
    world.add_bounding_cubes()
    world.save_world(save_path + '/forest' + str(i) + '.world')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate a random gazebo forest.')
    parser.add_argument('--num_worlds', type=int, help='Number of worlds to generate')
    parser.add_argument('--world_length_x', type=int, help='Length of world in m')
    parser.add_argument('--world_length_y', type=int, help='Width of world in m')
    parser.add_argument('--tree_density', type=float, help='Number of trees per m^2')
    parser.add_argument('--cylinder_density', type=float, help='Number of cylinders per m^2')
    parser.add_argument('--high_res', type=int, help='Use high res tree models')
    parser.add_argument('--save_dir', type=str, help='save folder')
    args = parser.parse_args()

    gen_worlds('./' + args.save_dir , args.num_worlds, args.world_length_x, args.world_length_y,
               int(args.world_length_x*args.world_length_y*args.tree_density),
               int(args.world_length_x*args.world_length_y*args.cylinder_density),
               bool(args.high_res))
