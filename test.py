#!/usr/bin/env python
from DracaenaPLC import DracaenaPLC, DracaenaCodes
import struct


# Main workflow
def main():
    plc = DracaenaPLC(host='192.168.0.50', port=2000)
    plc.connect_to_plc_socket()
    while True:
        received_data = plc.receive(2)
        print(received_data)
        # if received_data == DracaenaCodes.VisionStart.value:
        #     print('Sent {}'.format(DracaenaCodes.GripperClose.value))
        #     data = DracaenaCodes.GripperClose.value
        #     plc.send(data, wait_for_ack=False)


if __name__ == '__main__':
    main()
