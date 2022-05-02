#!/usr/bin/env python3
# _run.py
"""Run node for ah_ros_barrier.
"""
######################
# Imports & Globals
######################

import signal

from autopsy.node import Node

from aclpy.client.client_pkcs12 import ArrowheadClient
from aclpy.interface import ArrowheadInterface
from aclpy.server import ArrowheadServer
from aclpy.service import ArrowheadService

import websocket, json, sys, os

from pathlib import Path
from PIL import Image
# https://stackoverflow.com/questions/68957686/pillow-how-to-binarize-an-image-with-threshold
arrowhead_logo = Image.open(Path(__file__).parents[2].joinpath("logo").joinpath("arrowhead_logo.bmp")) \
                      .convert("L") \
                      .point( lambda p: 255 if p > 20 else 0 ) \
                      .convert("1") \
                      .resize((16, 9))

from typing import List


from std_msgs.msg import Time


######################
# Logo
######################

def show_arrowhead_logo():
    for y in range(arrowhead_logo.height):
        for x in range(arrowhead_logo.width):
            if arrowhead_logo.getpixel((x, y)):
                sys.stdout.write("#")
            else:
                sys.stdout.write(" ")
        print("")


######################
# UserInput handling
######################

def select_provider(providers: List[str]) -> int:
    if len(providers) == 1:
        print ("Auto selecting provider 1.\n")
        return 0

    print ("Multiple providers found.\n")

    while True:
        selection = raw_input("Use provider:")

        try:
            provider = int(selection)
        except:
            print ("Selection must be a number.")
            continue

        if provider > len(providers) or provider == 0:
            print ("There is not a provider with this number.")
        else:
            continue

    return provider-1


######################
# RunNode
######################

class RunNode(Node):

    def __init__(self):
        super(Node, self).__init__("ah_ros_barrier")

        show_arrowhead_logo()
        print ("Arrowhead ROS barrier starting up...\n")

        from . import configuration
        """Expected contents:
        Server = ArrowheadServer(
            address = AH_CORE_IP_ADDRESS,
        )

        Interface = ArrowheadInterface(
            name = NAME_OF_THE_INTERFACE,
        )

        Service = ArrowheadService(
            name = NAME_OF_THE_SERVICE,
        )

        Client = ArrowheadClient(
            name = NAME_OF_THE_SYSTEM,
            address = SYSTEM_IP_ADDRESS,
            port = SYSTEM_PORT,
            pubfile = PATH_TO_PUB_FILE,
            p12file = PATH_TO_P12_FILE,
            p12pass = PASS_TO_P12_FILE,
            cafile = PATH_TO_CA_FILE,
            server = Server,
            interfaces = [Interface],
        )
        """
        self.server = configuration.Server
        self.client = configuration.Client
        self.service = configuration.Service
        self.interface = configuration.Interface

        # Communicate with AHCore
        print ("Obtaining client ID from the Arrowhead Core...")
        self.client.obtain_id()

        if self.client.id < 0:
            raise Exception ("Unable to communicate with AHCore.")

        print ("> Client ID: %d\n" % self.client.id)

        print ("Looking for available providers...")
        success, matches = self.client.orchestrate(self.service)

        if not success:
            print (self.client.last_error)
            raise Exception ("Unable to communicate with Orchestrator.")

        if len(matches) == 0:
            raise Exception ("No matched providers. Is the target system running?")

        print ("> Found %d provider(s):" % len(matches))
        for _i, match in enumerate(matches):
            print ("\t%d: %s:%d" % (_i + 1, match.get("provider").address, match.get("provider").port))


        print ()
        provider = select_provider(matches)


        self.pub_time = self.Publisher("lap_time", Time, queue_size = 1)
        self.last_time = None


        self.provider_address = matches[provider].get("service").metadata.get("address", matches[provider].get("provider").address)
        self.provider_port = matches[provider].get("provider").port

        self.websocket = websocket.WebSocketApp("ws://%s:%d/" % (self.provider_address, self.provider_port), on_message = self.on_message, on_error = self.on_error)

        # Attach signal handler
        signal.signal(signal.SIGINT, self.signal_handler)

        self.websocket.run_forever()


    def on_message(self, ws, message):

        m = json.loads(message)

        if self.last_time is not None:
            us = m["timestamp"] - self.last_time

            msg = Time()
            msg.data.secs = int(us / 1e6)
            msg.data.nsecs = int((us % 1e6) * 1e3)

            self.pub_time.publish(msg)

        self.last_time = m["timestamp"]


    def on_error(self, ws, error):
        print ("Received error: ", error)


    def signal_handler(self, sig, frame):
        exit(0)
