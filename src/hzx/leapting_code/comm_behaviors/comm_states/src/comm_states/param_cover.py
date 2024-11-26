#!/usr/bin/env python
from flexbe_core import EventState, Logger
import xml.etree.ElementTree as ET
from rospkg import RosPack
import os

'''
Created on 2024-10-28
@author: lei.zeng@leapting.com
'''


class ParamCover(EventState):
    def __init__(self, behavior="TrailerPVMDetect",
                 include=[],
                 exclude=[]):
        super(ParamCover, self).__init__(outcomes=['done'])
        self._behavior = behavior
        self._include = include
        self._exclude = exclude

    def on_enter(self, userdata):
        manifest_path = RosPack().get_path('comm_behaviors') + "/manifest/"
        base_xml_path = RosPack().get_path('comm_behaviors') + \
            "/manifest/" + self._behavior.lower()+".xml"
        base_tree = ET.parse(base_xml_path)
        base_root = base_tree.getroot()
        base_params = base_root.findall('params')
        if len(base_params) > 0:
            base_param_list = base_params[0].findall('param')
        else:
            return
        base_param_dict = {}
        for p in base_param_list:
            if self._include == []:
                if not (p.attrib['name'] in self._exclude):
                    base_param_dict[p.attrib['name']] = p
            else:
                if p.attrib['name'] in self._include:
                    base_param_dict[p.attrib['name']] = p

        for b in os.listdir(manifest_path):
            f = manifest_path + b
            tree = ET.parse(f)
            root = tree.getroot()
            params = root.findall('params')
            if len(params) == 0:
                continue
            param_list = params[0].findall('param')
            for p in param_list:
                if p.attrib['name'] in base_param_dict:
                    base_default = base_param_dict[p.attrib['name']
                                                   ].attrib['default']
                    p.set("default", str(base_default))
            try:
                tree.write(f, encoding='utf-8', xml_declaration=True)
                Logger.loginfo("%s updated" % f)
            except Exception as e:
                Logger.logwarn('failed to update %s, exception: %s' %
                               (self.name, str(e)))

    def execute(self, userdata):
        return "done"
