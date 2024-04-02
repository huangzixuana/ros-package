#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from sensor_msgs.msg import JointState
import xml.etree.ElementTree as ET
import tf
from tf import TransformListener
import moveit_commander
import sys
import getpass
import yaml
import time

class ArmSiteCdsrSrdfYaml(EventState):
    
    '''
    operate arm site in srdf and yaml file
    -- srdf_path                string              path of srdf file
    -- yaml_path                string              path of yaml file
    -- source_frame             string              source frame
    -- target_frame             string              target frame

    <= done                     operation succeed
    <= failed                   operation fail
    <= srdf_failed              only srdf fails while operating together
    <= yaml_failed              only yaml fails while operating together
    '''

    def __init__(self, srdf_path='',
                 yaml_path='',
                 source_frame='base_arm',
                 target_frame='tool0',
                 operation = '',
                 operation_file = '',
                 site_name = ''):
        super(ArmSiteCdsrSrdfYaml, self).__init__(outcomes = ['done', 'failed', 'srdf_failed', 'yaml_failed'])
        self._srdf_path = self.relative_to_absolute_path(srdf_path)
        self._yaml_path = self.relative_to_absolute_path(yaml_path)
        self._operation = operation
        self._operation_file = operation_file
        self._site_name = site_name
        self.flag_yaml = True
        self.flag_srdf = True
        self.info_yaml = ''
        self.info_srdf = ''

        # method-1
        # self.tf1_lis = TransformListener()
        # method-2
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_group = moveit_commander.MoveGroupCommander('leapting_arm')

        self._source_frame = source_frame
        self._target_frame = target_frame
        self.site_positions = None
        self.joint_name = None
        self.pos = None
        self.quat = None
        self.joint_state_sub = ProxySubscriberCached({'joint_states': JointState})
        self.joint_state_sub.set_callback('joint_states', self.get_position_from_joint_states)
    
    def on_enter(self, userdata):
        self.site_positions = None
        self.pos = None
        if self._operation == 'add' or self._operation == 'update':
            if self._operation_file == 'yaml' or self._operation_file == 'together':
                while True:
                    self.get_pos_and_quat()
                    if self.pos != None:
                        break
            elif self._operation_file == 'srdf' or self._operation_file == 'together':
                while True:
                    if self.site_positions != None:
                        break
        
        if self._operation_file == 'yaml':
            self.operate_yaml()
        elif self._operation_file == 'srdf':
            self.operate_srdf()
        elif self._operation_file == 'together':
            self.operate_yaml()
            self.operate_srdf()
        
        # self.start_time = time.time()
        # pass
    
    def execute(self, userdata):
        # self.end_time1 = time.time()
        # Logger.loginfo('on_enter to execute time: %.2fs' % (self.end_time1-self.start_time))
        # if self.site_positions is None or self.pos is None:
        #     Logger.loginfo('site_positions: '+str(self.site_positions is None))
        #     Logger.loginfo('pos: '+str(self.pos is None))
        #     return
        # self.end_time2 = time.time()
        # Logger.loginfo('get msg time: %.2fs' % (self.end_time2-self.end_time1))

        if self._operation_file == 'together':
            if self.flag_srdf and self.flag_yaml:
                Logger.loginfo(self.info_srdf)
                Logger.loginfo(self.info_yaml)
                return 'done'
            elif not self.flag_srdf and self.flag_yaml:
                Logger.logwarn(self.info_srdf)
                Logger.loginfo(self.info_yaml)
                return 'srdf_failed'
            elif not self.flag_yaml and self.flag_srdf:
                Logger.loginfo(self.info_srdf)
                Logger.logwarn(self.info_yaml)
                return 'yaml_failed'
            else:
                Logger.logwarn(self.info_srdf)
                Logger.logwarn(self.info_yaml)
                return 'failed'
        elif self._operation_file == 'srdf':
            if self.flag_srdf:
                Logger.loginfo(self.info_srdf)
                return 'done'
            else:
                Logger.logwarn(self.info_srdf)
                return 'failed'
        elif self._operation_file == 'yaml':
            if self.flag_yaml:
                Logger.loginfo(self.info_yaml)
                return 'done'
            else:
                Logger.logwarn(self.info_yaml)
                return 'failed'
    
    def get_position_from_joint_states(self, msg):
        self.site_positions = msg.position
        self.joint_name = msg.name
    
    def get_pos_and_quat(self):
        # get TF between source_frame and target_frame
        # method-1
        # while True:
        #     try:
        #         t = self.tf1_lis.getLatestCommonTime(self._source_frame,  self._target_frame)
        #         p, q = self.tf1_lis.lookupTransform(self._source_frame, self._target_frame, t)
        #         self.pos = [round(x,3) for x in p]
        #         self.quat = [round(x,3) for x in q]
        #         break
        #     except Exception as e:
        #         Logger.logwarn('exception tf1: %s' %(str(e)))
        # method-2
        self.move_group.set_pose_reference_frame(self._source_frame)
        self.move_group.set_end_effector_link(self._target_frame)
        pose = self.move_group.get_current_pose().pose
        self.pos = [round(pose.position.x,3), round(pose.position.y,3), round(pose.position.z,3)]
        self.quat = [round(pose.orientation.x,3), round(pose.orientation.y,3), round(pose.orientation.z,3), round(pose.orientation.w,3)]

    def operate_yaml(self):
        if self._operation == 'add':
            self.create_site_yaml(self._site_name)
        elif self._operation == 'delete':
            self.delete_site_yaml(self._site_name)
        elif self._operation == 'search':
            self.search_site_yaml(self._site_name)
        elif self._operation == 'update':
            self.revise_site_yaml(self._site_name)
    
    def operate_srdf(self):
        if self._operation == 'add':
            self.create_site_srdf(self._site_name)
        elif self._operation == 'delete':
            self.delete_site_srdf(self._site_name)
        elif self._operation == 'search':
            self.search_site_srdf(self._site_name)
        elif self._operation == 'update':
            self.revise_site_srdf(self._site_name)

    def create_site_yaml(self, name):
        fr = open(self._yaml_path, 'r')
        dat = yaml.safe_load(fr)
        fr.close()
        if name in dat.keys():
            self.info_yaml = '[YAML] Failed. Already has %s site, please update it' %name
            self.flag_yaml = False
        elif len(self.pos) == 0 or len(self.quat) == 0:
            self.info_yaml = '[YAML] Failed. Check arm'
            self.flag_yaml = False
        else:
            pot = {}
            pot['position'] = self.pos
            pot['orientation'] = self.quat
            pot['type'] = 'pose'
            dat[name] = pot
            fw = open(self._yaml_path, 'w')
            yaml.dump(dat, fw)
            fw.close()
            self.info_yaml = '[YAML] Add success'

    def search_site_yaml(self, name):
        fr = open(self._yaml_path, 'r')
        dat = yaml.safe_load(fr)
        fr.close()
        if name not in dat.keys():
            self.info_yaml = '[YAML] Failed. No %s site' %name
            self.flag_yaml = False
        else:
            search_str = "\r\n[YAML] arm_site_name: "+str(name)+"\r\n"
            pot = dat[name]
            for k in pot:
                search_str += str(k)+": "+str(pot[k])+"\r\n"
            self.info_yaml = search_str

    def revise_site_yaml(self, name):
        fr = open(self._yaml_path, 'r')
        dat = yaml.safe_load(fr)
        fr.close()
        if name not in dat.keys():
            self.info_yaml = '[YAML] Failed. No %s site' %name
            self.flag_yaml = False
        else:
            dat[name]['position'] = self.pos
            dat[name]['orientation'] = self.quat
            fw = open(self._yaml_path, 'w')
            yaml.dump(dat, fw)
            fw.close()
            self.info_yaml = '[YAML] Update success'

    def delete_site_yaml(self, name):
        fr = open(self._yaml_path, 'r')
        dat = yaml.safe_load(fr)
        fr.close()
        if name not in dat.keys():
            self.info_yaml = '[YAML] Failed. No %s site' %name
            self.flag_yaml = False
        else:
            del dat[name]
            fw = open(self._yaml_path, 'w')
            yaml.dump(dat, fw)
            fw.close()
            self.info_yaml = '[YAML] Delete success'

    def create_site_srdf(self, name):
        tree = ET.parse(self._srdf_path)
        root = tree.getroot()
        site_list = root.findall('group_state')
        site = [x for x in site_list if x.attrib['name']==name]
        if len(site) != 0:
            self.info_srdf = '[SRDF] Failed. Already has %s site, please update it' %name
            self.flag_srdf = False
        else:
            group_state_p = ET.Element('group_state')
            group_state_p.set('name', name)
            group_state_p.set('group', "leapting_arm")
            for i in range(len(self.site_positions)):
                joint_p = ET.Element('joint')
                joint_p.set('name', self.joint_name[i])
                joint_p.set('value', str(round(self.site_positions[i],6)))
                group_state_p.append(joint_p)
            root.append(group_state_p)
            self.pretty_xml(root)
            tree.write(self._srdf_path, encoding='utf-8', xml_declaration=True)
            self.info_srdf = '[SRDF] Add success'

    def search_site_srdf(self, name):
        tree = ET.parse(self._srdf_path)
        root = tree.getroot()
        site_list = root.findall('group_state')
        site = [x for x in site_list if x.attrib['name']==name]
        if len(site) == 0:
            self.info_srdf = '[SRDF] Failed. No %s site' %name
            self.flag_srdf = False
        else:
            search_str = "\r\n[SRDF] arm_site_name: "+str(name)+"\r\n"
            joint_list = site[0].findall('joint')
            for i in range(len(joint_list)):
                search_str += str(joint_list[i].attrib['name'])+": "+str(joint_list[i].attrib['value'])+"\r\n"
            self.info_srdf = search_str

    def revise_site_srdf(self, name):
        tree = ET.parse(self._srdf_path)
        root = tree.getroot()
        site_list = root.findall('group_state')
        site = [x for x in site_list if x.attrib['name']==name]
        if len(site) == 0:
            self.info_srdf = '[SRDF] Failed. No %s site' %name
            self.flag_srdf = False
        else:
            js = site[0].findall('joint')
            if len(js) != len(self.joint_name):
                self.info_srdf = '[SRDF] Failed. Please check joint number'
                self.flag_srdf = False
            else:
                for j in range(len(js)):
                    name_temp = str(js[j].attrib['name'])
                    if name_temp not in self.joint_name:
                        self.info_srdf = '[SRDF] Failed. Please check joint name'
                        self.flag_srdf = False
                        break
                    else:
                        index_temp = self.joint_name.index(name_temp)
                        js[j].set('value',str(round(self.site_positions[index_temp],6)))
                        if j == len(js)-1:
                            self.pretty_xml(root)
                            tree.write(self._srdf_path, encoding='utf-8', xml_declaration=True)
                            self.info_srdf = '[SRDF] Update success'

    def delete_site_srdf(self, name):
        tree = ET.parse(self._srdf_path)
        root = tree.getroot()
        site_list = root.findall('group_state')
        site = [x for x in site_list if x.attrib['name']==name]
        if len(site) == 0:
            self.info_srdf = '[SRDF] Failed. No %s site' %name
            self.flag_srdf = False
        else:
            root.remove(site[0])
            self.pretty_xml(root)
            tree.write(self._srdf_path, encoding='utf-8', xml_declaration=True)
            self.info_srdf = '[SRDF] Delete success'

    def pretty_xml(self, elem, level=0):
        i ="\n" + level*"\t"
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i +"\t"
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for elem in elem:
                self.pretty_xml(elem, level+1)
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i

    def relative_to_absolute_path(self, relative_path):
        if relative_path[0] == '~':
            return '/home/' + getpass.getuser() + relative_path[1:]
        else:
            return relative_path