#!/usr/bin/env python
from flexbe_core import EventState, Logger
import xml.etree.ElementTree as ET
import getpass


class CalibrationSite(EventState):
    
    '''
    read arm site in srdf file
    -- srdf_path                string              path of srdf file
    -- mirror                   bool                get mirror site
    -- mirror_joint             list                joint is set negative

    <= done                     read succeed
    '''

    def __init__(self, srdf_path='', mirror=False, mirror_joint=[1,4,6]):
        super(CalibrationSite, self).__init__(outcomes = ['done'], output_keys=['arm_site'])
        self._srdf_path = self.relative_to_absolute_path(srdf_path)
        self._mirror = mirror
        self._mirror_joint = mirror_joint
    
    def on_enter(self, userdata):
        site_dict = {}
        try:
            tree = ET.parse(self._srdf_path)
            root = tree.getroot()
            site_list = root.findall('group_state')
            site = [x for x in site_list if x.attrib['name'][:11]=='calibration']
            if len(site) == 0:
                Logger.logwarn('No calibration site in srdf')
                return
            else:
                for c in site:
                    joint_list = c.findall('joint')
                    joint_value = []
                    for i in range(len(joint_list)):
                        if self._mirror and self._mirror_joint.__contains__(i+1):
                            joint_value.append(-1 * float(joint_list[i].attrib['value']))
                        else:
                            joint_value.append(float(joint_list[i].attrib['value']))
                    site_dict[c.attrib['name']] = joint_value
                userdata.arm_site = site_dict
        except Exception as e:
            Logger.logwarn('%s: exception : %s' % (self.name, str(e)))
            return
    
    def execute(self, userdata):
        return 'done'

    def relative_to_absolute_path(self, relative_path):
        if relative_path[0] == '~':
            return '/home/' + getpass.getuser() + relative_path[1:]
        else:
            return relative_path