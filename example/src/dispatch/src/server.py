#!/usr/bin/python3

import sys
import threading
import signal
from http.server import BaseHTTPRequestHandler, HTTPServer

import rospy
from std_srvs.srv import EmptyRequest

# FINAL CONSTANTS
ROBOT_IP='0.0.0.0'
NODE_NAME = 'server'
CMD_PARAM_NAME = 'cmd'

START_SERVICE = 'start'
STOP_SERVICE = 'stop'
CHANGE_ALGORITHM = 'change'
CHANGE_ALG_ALGORITHM_KEY = 'algorithm'

HTTP_OK_CODE = 200
HTTP_ERROR_CODE = 404

# GLOBAL PARAMS
running = True

def stop_program(code, arg):
    global running
    running = False

def parseRoute(route: str) -> dict:
    params: dict = {}
    paramsStr: list[str] = route.split('?')
    if(len(paramsStr) <= 1):
        return params
    paramsStr = paramsStr[1].split('&') 
    for i in range(len(paramsStr)):
        values: list[str] = paramsStr[i].split('=')
        params[values[0]] = values[1]
    return params

def getEndpoint(route: str):
    return route.split('?')[0]

class DispatcherServer(BaseHTTPRequestHandler):
    """
    A class representing the server for receiving command requests and dispatch them.

    Inherits from BaseHTTPRequestHandler and handles GET requests.
    """

    def __init__(self, request, client_address, server):
        super().__init__(request, client_address, server)

    def do_GET(self):
        self.send_header("Content-type", "json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        params: dict = parseRoute(self.path)
        if not params.get(CMD_PARAM_NAME):
            self.send_response(HTTP_ERROR_CODE)
            return

        # DISPATCH TO CORRECT SERVICE
        dispatched = True

        if params[CMD_PARAM_NAME] == START_SERVICE:
            client = rospy.ServiceProxy(START_SERVICE)
            client.call(EmptyRequest())
        elif params[CMD_PARAM_NAME] == STOP_SERVICE:
            client = rospy.ServiceProxy(STOP_SERVICE)
            client.call(EmptyRequest())
        elif params[CMD_PARAM_NAME] == CHANGE_ALGORITHM:
            pass
            # TODO, implement
        else:
            dispatched = False

        if dispatched: self.send_response(HTTP_OK_CODE)
        else: self.send_response(HTTP_ERROR_CODE)

if __name__ == "__main__":
    print(sys.argv)
    if len(sys.argv) < 2:
        print("Usage:\npython server.py <port number>")
        exit()

    server_port = int(sys.argv[1])
    rospy.init_node(NODE_NAME)

    webServer = HTTPServer((ROBOT_IP, server_port), DispatcherServer)
    print("Server started http://%s:%s" % (ROBOT_IP, server_port))

    def server_listen():
        while running:
            webServer.handle_request()

    thread = threading.Thread(target=server_listen, daemon=True)
    thread.start()
    
    signal.signal(signal.SIGINT, stop_program)

    while running:
        pass

    webServer.server_close()
    print("Server stopped.")