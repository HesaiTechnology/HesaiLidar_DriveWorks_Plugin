# Licensed under the Apache License: Version 2.0
# This file is used to simulate the sending of lidar UDP packet from the pcap file 
# Refer to https://gist.github.com/ninedraft/7c47282f8b53ac015c1e326fffb664b5 for more information
# Copyright (C) Hesai Technology Co., Ltd

import socket
import time
from scapy.utils import PcapReader

server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

# enable broadcasting mode
server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# support three types of lidar, pcap_sample_p128 pcap_sample_qt128 pcap_sample_qt128
filename = r'../data/pcap_sample_p128.pcap'

# @note sending packets could be slow down while computing too large bunch of points, e.g. Pandar128
# This phenomenent could be witnessed by PandarView & sample_lidar_replay
while (True):
    start_time  = time.time()
    with PcapReader(filename) as packets:
        for packet in packets:
            server.sendto(packet["Raw"].load[42:], ('<broadcast>', 2368))
            # ensure the point cloud display, 1ms
            # time.sleep(0.001)
    print("-----Done, playing the pcap, exec time = %s seconds-----" % (time.time() - start_time))