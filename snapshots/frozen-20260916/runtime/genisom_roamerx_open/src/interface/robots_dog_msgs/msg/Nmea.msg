std_msgs/Header header

# 核心定位信息
sensor_msgs/NavSatFix nav_sat_fix

# GPGGA 特定字段
string  utc_time                # utc时间   <2>
uint8   qual                    # 质量指示符 <7>
uint8   satellites_used         # 使用卫星数 <8>
float64 hdop                    # 水平精度因子 <9>
float64 undulation              # 大地水准面高度 <12>
string  undulation_units        # 大地水准面高度单位 <13>

# GPTRA 字段
float64 heading                 # 航向角 <3>
float64 pitch                   # 俯仰角 <4>
float64 roll                    # 横滚角 <5>

# 原始NMEA
string raw_sentence_gpgga
string raw_sentence_gptra