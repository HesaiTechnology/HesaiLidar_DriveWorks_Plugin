# -*- coding: utf-8 -*-

import socket
import time
from scapy.utils import PcapReader

# https://gist.github.com/ninedraft/7c47282f8b53ac015c1e326fffb664b5

server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

# Enable broadcasting mode
server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# pcap_sample_p128 pcap_sample_qt128
filename = r'../data/pcap_sample_p128.pcap'

while (True):
    with PcapReader(filename) as packets:
        for packet in packets:
            # print(p)
            server.sendto(packet["Raw"].load[42:], ('<broadcast>', 2368))
            # break
            # ensure the point cloud display clearly 1ms
            # time.sleep(0.001)
    # break
    print("----------")

