#!/usr/bin/env python2

import numpy as np
import math
import yaml
import pprint

import rospy
from std_msgs.msg import Int32MultiArray
from amsl_navigation_msgs.msg import Node, Edge, NodeEdgeMap

class GetYaml:
    def __init__(self):
        rospy.init_node('get_yaml')
        self.map = NodeEdgeMap()
        self.map_pub = rospy.Publisher('map', NodeEdgeMap, queue_size=1, latch=True)
        self.checkpoint_list = Int32MultiArray()
        self.checkpoint_list_pub =rospy.Publisher('checkpoint', Int32MultiArray, queue_size=1, latch=True)

        # rospy.set_param('self.MAP_PATH', '$(find amsl_navigation_managers)/amsl_navigation_managers/sample/map/R_rwrc21_d_kan_map.yaml')
        # rospy.set_param('self.CHECKPOINT_PATH', '$(find amsl_navigation_managers)/amsl_navigation_managers/sample/checkpoint/rwrc21_d_kan_checkpoint.yaml')

        self.HZ = 1
        if rospy.has_param('~HZ'):
            self.HZ = rospy.get_param('~HZ')
        if rospy.has_param('~MAP_PATH'):
            self.MAP_PATH = rospy.get_param('~MAP_PATH')
        if rospy.has_param('~CHECKPOINT_PATH'):
            self.CHECKPOINT_PATH = rospy.get_param('~CHECKPOINT_PATH')

    def process(self):
        r = rospy.Rate(self.HZ)
        while not rospy.is_shutdown():
            self.map_data = self.load_map_from_yaml()
            self.cp_data = self.load_cp_from_yaml()
            # pprint.pprint(self.map_data)
            # pprint.pprint(self.cp_data)
            self.make_and_publish()
            r.sleep()

    def load_map_from_yaml(self):
        with open(self.MAP_PATH) as file:
            map_data = yaml.safe_load(file)
        return map_data

    def load_cp_from_yaml(self):
        with open(self.CHECKPOINT_PATH) as file:
            cp_data = yaml.safe_load(file)
        return cp_data

    def make_map(self):
        self.map.nodes = []
        for node in self.map_data['NODE']:
            n = Node()
            n.id = node['id']
            n.type = node['type']
            n.label = node['label']
            n.point.x = node['point']['x']
            n.point.y = node['point']['y']
            self.map.nodes.append(n)

    def make_checkpoint(self):
        for cp in self.cp_data['checkpoints']:
            self.checkpoint_list.data.append(cp)

    def make_and_publish(self):
        self.make_map()
        self.make_checkpoint()
        self.map_pub.publish(self.map)
        self.checkpoint_list_pub.publish(self.checkpoint_list)

if __name__ == '__main__':
    get_yaml = GetYaml()
    get_yaml.process()
