#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <unordered_map>

class MessageToWebNode 
{
public:
    MessageToWebNode() : nh_("~") 
    {
        // 订阅诊断消息
        diagnostics_sub_ = nh_.subscribe("/diagnostics_agg", 10, &MessageToWebNode::diagnosticsCallback, this);
        // 发布诊断消息
        diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/message_to_web_diagnostics", 10);
    }

    void diagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) 
    {
        diagnostic_msgs::DiagnosticArray output_msg;
        output_msg.header.stamp = ros::Time::now();

        for (const auto& status : msg->status) 
        {
            if (status.name == "/DEVICES") 
            {
                std::unordered_map<std::string, std::string> values1_map;
                // 遍历，将所有需要的值存储到map1中
                for (const auto& value : status.values) 
                {
                    values1_map[value.key] = value.value;
                }
                addStatusIfExists4(output_msg, values1_map, "DRIVE", 1, 1);
                addStatusIfExists4(output_msg, values1_map, "PLC", 1, 1);
                addStatusIfExists4(output_msg, values1_map, "CAMERA", 1, 1);
                addStatusIfExists4(output_msg, values1_map, "IMU", 1, 1);
                addStatusIfExists4(output_msg, values1_map, "ARM", 1, 1);
            }
            else if (status.name == "/DEVICES/PLC/rosbridge_plc24: Hardware status") 
            {
                std::unordered_map<std::string, std::string> values2_map;
                // 遍历，将所有需要的值存储到map2中
                for (const auto& value : status.values) 
                {
                    values2_map[value.key] = value.value;
                }
                addStatusIfExists(output_msg, values2_map, "battery_voltage", 2, 2);
                addStatusIfExists(output_msg, values2_map, "battery_current", 2, 2);
                addStatusIfExists(output_msg, values2_map, "hydraulic_voltage", 2, 2);
                addStatusIfExists(output_msg, values2_map, "chassis_voltage", 2, 2);
                addStatusIfExists(output_msg, values2_map, "inverter_voltage", 2, 2);
                addStatusIfExists1(output_msg, values2_map, "hydraulic_status", 2, 2);
                addStatusIfExists1(output_msg, values2_map, "chassis_status", 2, 2);
                addStatusIfExists1(output_msg, values2_map, "inverter_status", 2, 2);
                addStatusIfExists2(output_msg, values2_map, "comm_status", 1, 1);
                addStatusIfExists2(output_msg, values2_map, "estop_status", 1, 1);
                addStatusIfExists3(output_msg, values2_map, "robot_status", 1, 1);
                addStatusIfExists1(output_msg, values2_map, "charger_status", 2, 2);
                addStatusIfExists(output_msg, values2_map, "battery_voltage_alarm", 1, 1);
                addStatusIfExists(output_msg, values2_map, "overcurrent_alarm", 1, 1);
                addStatusIfExists2(output_msg, values2_map, "sensor_status", 1, 1);
                addStatusIfExists2(output_msg, values2_map, "joy_estop", 1, 1);
                addStatusIfExists2(output_msg, values2_map, "whisker_status", 1, 1);
                addStatusIfExists(output_msg, values2_map, "vacuum1_pressure", 2, 2);
                addStatusIfExists(output_msg, values2_map, "vacuum_pressure", 2, 2);
                addStatusIfExists(output_msg, values2_map, "temperature", 2, 2);
            }
        }

        // 发布消息
        diagnostics_pub_.publish(output_msg);
    }

private:
    void addDiagnosticStatus(diagnostic_msgs::DiagnosticArray& output_msg, const std::string& name, uint8_t level, 
        int hardware_id, const std::string& message, const std::vector<diagnostic_msgs::KeyValue>& values) 
    {
        diagnostic_msgs::DiagnosticStatus status;
        status.name = name;
        status.level = level;
        status.hardware_id = std::to_string(hardware_id);
        status.message = message;
        status.values = values;
        output_msg.status.push_back(status);
    }

    //Output original value
    void addStatusIfExists(diagnostic_msgs::DiagnosticArray& output_msg, const std::unordered_map<std::string, std::string>& values_map,
        const std::string& key, int lev = 1, int har = 1)
    {
        if (values_map.find(key) != values_map.end()) 
        {
            addDiagnosticStatus(output_msg, key, lev, har, values_map.at(key), {});
        }
    }

    //0 is "off" , 1 is "on", 2 is "input_error", 3 is "output_error"
    void addStatusIfExists1(diagnostic_msgs::DiagnosticArray& output_msg, std::unordered_map<std::string, std::string>& values_map,
        const std::string& key, int lev, int har)
    {
        if (values_map.find(key) != values_map.end()) 
        {
            values_map.at(key) = getStatus(values_map.at(key));
            addDiagnosticStatus(output_msg, key, lev, har, values_map.at(key), {});
        }
    }

    //!0 is error
    void addStatusIfExists2(diagnostic_msgs::DiagnosticArray& output_msg, std::unordered_map<std::string, std::string>& values_map,
        const std::string& key, int lev, int har)
    {
        if (values_map.find(key) != values_map.end()) 
        {
            if (values_map.at(key) != "0")
                values_map.at(key) = "2";
            addDiagnosticStatus(output_msg, key, lev, har, values_map.at(key), {});
        }
    }

    // change 0 and 1
    void addStatusIfExists3(diagnostic_msgs::DiagnosticArray& output_msg, std::unordered_map<std::string, std::string>& values_map,
        const std::string& key, int lev, int har)
    {
        if (values_map.find(key) != values_map.end()) 
        {
            if (values_map.at(key) == "1")
                values_map.at(key) = "0";
            else
                values_map.at(key) = "2";
            addDiagnosticStatus(output_msg, key, lev, har, values_map.at(key), {});
        }
    }

    //ok is "0" , warn is "1", error is "2", stale is "3"
    void addStatusIfExists4(diagnostic_msgs::DiagnosticArray& output_msg, std::unordered_map<std::string, std::string>& values_map,
        const std::string& key, int lev, int har)
    {
        if (values_map.find(key) != values_map.end()) 
        {
            values_map.at(key) = getLevel(values_map.at(key));
            addDiagnosticStatus(output_msg, key, lev, har, values_map.at(key), {});
        }
    }

    std::string getLevel(const std::string& value) 
    {
        if (value == "OK" || value == "Ok") return "0";
        else if (value == "WARN" || value == "Warn") return "1";
        else if (value == "ERROR" || value == "Error") return "2";
        else if (value == "STALE" || value == "Stale") return "3";
        return "2"; // 默认值
    }

    std::string getStatus(const std::string& value) 
    {
        if (value == "0") return "off";
        else if (value == "1") return "on";
        else if (value == "2") return "input_error";
        else if (value == "3") return "output_error";
        return "off"; // 默认值
    }

    ros::NodeHandle nh_;
    ros::Subscriber diagnostics_sub_;
    ros::Publisher diagnostics_pub_;
};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "message_to_web");
    MessageToWebNode node;
    ros::spin();
    return 0;
}