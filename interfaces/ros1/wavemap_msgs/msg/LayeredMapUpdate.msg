# A sensor-independent batch of spatial layer observations.
# Every layer contains one value per position, in the same order.
# Producers may include any subset of the configured map layers.
std_msgs/Header header
geometry_msgs/Point32[] positions
Layer[] layers
