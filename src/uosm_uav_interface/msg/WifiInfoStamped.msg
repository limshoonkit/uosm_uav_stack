# uosm_msgs/msgs/WifiInfoStamped.msg
builtin_interfaces/Time time

# Interface name
string interface_name

# MAC address
string mac_address

# SSID
string ssid

# Frequency channel
uint16 channel
float32 frequency_mhz
float32 center_frequency_mhz

# Transmission power (in dBm)
float32 tx_power_dbm

# TX bitrate in Mbps
float32 tx_bitrate_mbps

# RX bitrate in Mbps
float32 rx_bitrate_mbps

# Signal strength in dBm
int8 signal_dbm