#!/usr/bin/env python
import json
import rospy
import stomp
from include.constants import ACTIVEMQ_QUEUE


if __name__ == "__main__":
    conn = stomp.Connection([('localhost', 61613)])
    conn.start()
    conn.connect()
    conn.send(ACTIVEMQ_QUEUE, 'Simples Assim')
    conn.disconnect()
