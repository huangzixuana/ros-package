#!/usr/bin/env python3
"""
Author: lei.zeng@leapting.com
"""
import rospy
import re
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import json
import time
from std_msgs.msg import Header
import pywifi
import speedtest


class AggData(object):
    def __init__(self):
        self.diagnostics_pub = rospy.Publisher('/diagnostics',
                                               DiagnosticArray,
                                               queue_size=10)
        self._startup_diag = diag_array_gen('startup', 1, 'init')
        self._startup_diag.status[0].values.append(
            key_val_gen('robot system', 'OK'))
        self._startup_diag.status[0].values.append(
            key_val_gen('pdu release', 'waiting'))
        self._startup_diag.status[0].values.append(
            key_val_gen('robot status', 'waiting'))
        self._startup_diag.status[0].values.append(
            key_val_gen('arm', 'waiting'))
        self._startup_diag.status[0].values.append(
            key_val_gen('devices', 'waiting'))

        self.agg_json_pub = rospy.Publisher('json_agg', Header, queue_size=1)
        rospy.Subscriber('/diagnostics_agg',
                         DiagnosticArray,
                         self.agg_cb,
                         queue_size=5)
        self.wifi_dict = {'ssid': '',
                          'signal(dbm)': 0,
                          'upspeed(Mb/s)': 0,
                          'downspeed(Mb/s)': 0}
        self.wifi_level = 0

        try:
            self.pywifi = pywifi.PyWiFi()
            self.spd = speedtest.Speedtest()
        except Exception as e:
            rospy.logwarn(" %s " % str(e))

        rospy.Timer(rospy.Duration(1), self.time_cb_unit, oneshot=False)

    # def trig_cb(self, msg):
    #     if msg.frame_id == 'launch_arm':
    #         if msg.seq == 0:
    #             self._startup_diag.status[0].values[2].value = 'waiting'
    #         if msg.seq == 1:
    #             self._startup_diag.status[0].values[2].value = 'OK'

    def time_cb_unit(self, event):
        msg = 'ssid:%s, signal:%s, upspeed:%s, downspeed:%s' % (
            self.wifi_dict['ssid'],  str(self.wifi_dict['signal(dbm)']),
            str(self.wifi_dict['upspeed(Mb/s)']), str(self.wifi_dict['downspeed(Mb/s)']))
        wifi_diag_msg = diag_array_gen(
            'net:wifi', self.wifi_level, msg)
        wifi_diag_msg.status.append(self._startup_diag.status[0])
        self.diagnostics_pub.publish(wifi_diag_msg)
        print(wifi_diag_msg)
        print('---')

    def get_ssid(self):
        try:
            iface = self.pywifi.interfaces()[0]
            profile = iface.scan_results()[0]

            self.wifi_dict['ssid'] = profile.ssid
            self.wifi_dict['signal(dbm)'] = profile.signal
            self.wifi_level = 0

        except Exception as e:
            rospy.logwarn("get wifi-ssid exception: %s " % str(e))
            self.wifi_dict['ssid'] = ''
            self.wifi_dict['signal(dbm)'] = 0
            self.wifi_level = 1

    def get_speed(self):
        try:
            self.wifi_dict['downspeed(Mb/s)'] = round(
                (round(self.spd.download()) / 1048576), 2)  # Mb/s
            self.wifi_dict['upspeed(Mb/s)'] = round(
                (round(self.spd.upload()) / 1048576), 2)
        except Exception as e:
            rospy.logwarn("get wifi-speed exception: %s " % str(e))
            self.wifi_dict['downspeed(Mb/s)'] = 0
            self.wifi_dict['upspeed(Mb/s)'] = 0

    def agg_cb(self, msg):
        self.agg_dict = {'root': {}}
        status = list(msg.status)
        status = sorted(status, key=lambda x: len(
            [i.start() for i in re.finditer("/", x.name)]), reverse=True)
        for s in status:
            n = s.name
            if s.name.find('/') != 0:
                rospy.logwarn('state name must start with slash')
                continue

            list_slash_index = [i.start() for i in re.finditer("/", n)]
            list_slash_index.append(len(n) + 1)

            n_unit = n[list_slash_index[-2]: list_slash_index[-1]]
            n_unit = n_unit[1:]
            if n_unit not in self.agg_dict:
                self.agg_dict[n_unit] = {}
            self.agg_dict[n_unit].update({
                'level': s.level,
                'message': s.message,
                'hardware_id': s.hardware_id,
                'name': n_unit

            })

            if len(list_slash_index) >= 3:
                n_last = n[list_slash_index[-3]: list_slash_index[-2]]
                n_last = n_last[1:]
                if n_last not in self.agg_dict:
                    self.agg_dict[n_last] = {'underling': {}}
                self.agg_dict[n_last]['underling'][n_unit] = self.agg_dict[n_unit]
            else:
                n_last = 'root'
                self.agg_dict[n_last][n_unit] = self.agg_dict[n_unit]

            if self._startup_diag.status[0].level != 0:
                if s.name == '/DEVICES/PDU/rosbridge_pdu: Hardware status':
                    if s.values[-1].key == "robot_status":
                        self._startup_diag.status[0].values[2].value = ('OK' if (
                            s.values[-1].value == '1') else 'waiting')
                        # print('*'*20)
                    if s.values[-3].key == "estop_status":
                        self._startup_diag.status[0].values[1].value = ('OK' if (
                            s.values[-1].value == '0') else 'waiting')
                elif s.name == '/DEVICES/ARM/arm_joint':
                    if s.level == 0:
                        self._startup_diag.status[0].values[3].value = "OK"
                    else:
                        self._startup_diag.status[0].values[3].value = "waiting"
                elif s.name == '/DEVICES':
                    if s.level == 0:
                        self._startup_diag.status[0].values[4].value = "OK"
                        self._startup_diag.status[0].level = 0
                        self._startup_diag.status[0].message = "done"
                    else:
                        self._startup_diag.status[0].values[4].value = "waiting"

        pub_msg = Header()
        pub_msg.stamp = rospy.Time.now()
        pub_msg.frame_id = json.dumps(self.agg_dict['root'], sort_keys=True)
        self.agg_json_pub.publish(pub_msg)
        # print(json.dumps(self.agg_dict['root'], sort_keys=True))
        print('*'*20)
        print(self._startup_diag)


def diag_array_gen(d_name, d_level, d_msg):
    da_msg = DiagnosticArray()
    da_msg.header.stamp = rospy.Time.now()
    status_msg = DiagnosticStatus()
    status_msg.name = d_name
    status_msg.level = d_level
    status_msg.message = d_msg
    da_msg.status.append(status_msg)
    return da_msg


def key_val_gen(k, v):
    kv_msg = KeyValue()
    kv_msg.key = k
    kv_msg.value = v
    return kv_msg


def main():
    rospy.init_node('agg_data')
    wifi_access_period = rospy.get_param("~wifi_access_period", 10)
    wifi_monitor = rospy.get_param("~wifi_monitor", True)
    agg_obj = AggData()

    while not rospy.is_shutdown() and wifi_monitor:
        agg_obj.get_ssid()
        agg_obj.get_speed()
        rospy.sleep(wifi_access_period)

    try:
        rospy.spin()
    except:
        print("Shutting down...")


if __name__ == '__main__':
    main()
