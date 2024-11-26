from flexbe_core import EventState, Logger
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from flexbe_core.proxy import ProxySubscriberCached
from collections import deque
import time

class VacuumPressure(EventState):
    """
    FlexBE 状态，用于处理真空压力数据。

    -- action               string          "cupon" 或 "cupoff"。
    -- threshold            float           压力阈值
    -- frame                int             用于判断的帧数
    """

    def __init__(self, action="cupon", threshold=510.0, frame=5):
        super(VacuumPressure, self).__init__(outcomes=['done', 'timeout'])
        self._action = action
        self._threshold = threshold
        self._frame = frame
        self._pressure_values = deque(maxlen=frame)
        self._start_time = time.time()  # 记录状态开始时间

    def on_enter(self, userdata):
        self._start_time = time.time()  # 记录状态开始时间
        self._solar_sub = ProxySubscriberCached(
            {'plc24_message': DiagnosticArray})
        self._solar_sub.subscribe("plc24_message",
            DiagnosticArray,
            self._diagnostic_callback)

    def execute(self, userdata):

        current_time = time.time()
        # 检查是否超过 10 秒
        if (current_time - self._start_time) > 10:
            Logger.logwarn("pressure dont meet threshold more than 10s")
            return 'timeout'

        # 如果帧数据不够，继续等待
        if len(self._pressure_values) < self._frame:
            return   # 继续等待

        # 检查帧数据是否满足条件
        if self._action == "cupon":
            if all(value > self._threshold for value in self._pressure_values):
                Logger.loginfo("Vacuum pressure values: {0}".format(list(self._pressure_values)))
                return 'done'
        
        else:
            if all(value < self._threshold for value in self._pressure_values):
                Logger.loginfo("Vacuum pressure values: {0}".format(list(self._pressure_values)))
                return 'done'

        return  # 继续等待

    def _diagnostic_callback(self, data):
        for status in data.status:
            for value in status.values:
                if value.key == "vacuum_pressure":
                    vacuum_pressure = float(value.value)
                    self._pressure_values.append(vacuum_pressure)
                    break