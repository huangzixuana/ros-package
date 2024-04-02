底盘电池话题 /battery_status 说明 

- 消息类型sensor_msgs/BatteryState

- 消息中各字段含义

  - voltage:电压 ，单位 ：v

  - temperature：4个电芯的平均温度，单位 : ^o^C

  - current：电流 ,单位：A

  - charge:当前电池容量，单位Ah

  - percentage:电量 ，在0-1之间

  - power_supply_status:  1代表充电，2代表放电 

  - power_supply_health:0代表未知，1代表健康，2代表过热 6代表过冷

  - power_supply_technology:4代表锂铁电池

  - persent：true代表有电池

  - cell_voltage:代表电池组电压，单位V

  - cell_temperature:代表4个电芯温度，单位 ^o^C