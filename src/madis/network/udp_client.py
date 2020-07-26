#######################################################
#                                                     #
# NATO STO CMRE - La Spezia                           #
#                                                     #
# Utility function for geographic operations          #
# Refactoring into another project                    #
#                                                     #
#     Author: Renato Sirola (CTR)                     #
#     E-mail: renato.sirola@cmre.nato.int             #
#       Date: January 2018                            #
# Department: E.T.D.                                  #
#                                                     #
#######################################################

import threading
import asyncio
import time


class InternalUPDClient(object):
    def __init__(self, receiving_handle_data=None, logger=None):
        self._receiving_handle_data = receiving_handle_data
        self._logger = logger
        self._transport = None
        self._use_logger = True
        if self._logger is None:
            self._use_logger = False
        if self._use_logger is True:
            self._logger.debug("InternalUDPClient will use logger")

    def connection_made(self, transport):
        self._transport = transport

    def send_data(self, data_to_send):
        if self._transport is not None:
            self._transport.sendto(data_to_send.encode())

    def datagram_received(self, data, addr):
        if self._receiving_handle_data is not None:
            self._receiving_handle_data(data)

    def error_received(self, exc):
        if self._use_logger is True:
            self._logger.warning("The transport system return an error: {}".format(exc))

    def connection_lost(self, exc):
        if self._use_logger is True:
            self._logger.warning("Socket closed, stop the event loop...")
        loop = asyncio.get_event_loop()
        loop.stop()


class UDPClient(object):
    def __init__(self, server, port, receiving_handle_data=None, logger=None):
        self._logger = logger
        self._server = server
        self._port = port
        self._loop = None
        self._internal_udp_client = None
        self._internal_loop_thread = None
        self._stop_loop_thread = None
        self._transport = None
        self._protocol = None
        self._receiving_handle_data = receiving_handle_data
        self._use_logger = True
        if self._logger is None:
            self._use_logger = False
        if self._use_logger is True:
            self._logger.debug("UDPClient will use logger")
            self._logger.info("UDPClient -> try to connect to : (" + self._server + ":" + str(self._port) + ")")
            if self._receiving_handle_data is not None:
                self._logger.info("_receiving_handle_data() method is defined: {}".format(self._receiving_handle_data))
            else:
                self._logger.warning("_receiving_handle_data() method is UNDEFINED (NONE) - the udp/ip client will can only send data to server")

    def initialize_connection(self):
        if self._use_logger is True:
            self._logger.debug("Initializing UDPClient")
        asyncio.set_event_loop(asyncio.new_event_loop())
        self._loop = asyncio.get_event_loop()
        if self._use_logger is True:
            self._logger.debug("AsyncIO loop created")
        self._internal_udp_client = InternalUPDClient(self._receiving_handle_data, self._logger)
        connection = self._loop.create_datagram_endpoint(lambda: self._internal_udp_client, remote_addr=(self._server, self._port))
        self._transport, self._protocol = self._loop.run_until_complete(connection)
        self._internal_loop_thread = threading.Thread(target=self._start_server_loop, args=())
        self._internal_loop_thread.setDaemon(True)
        self._internal_loop_thread.start()
        self.send("\n")

    def _start_server_loop(self):
        if self._use_logger is True:
            self._logger.debug("UDP/IP Client: starting loop/connection with {} on port {}".format(self._server, self._port))
        self._loop.run_forever()

    def send(self, data_to_send):
        if self._internal_udp_client is not None:
            self._internal_udp_client.send_data(data_to_send)

    def receive(self, args=None):
        pass

    def stop(self):
        if self._use_logger is True:
            self._logger.debug("Request stop UDP/IP")
        self._stop_loop_thread = threading.Thread(target=self._stop_loop, args=())
        self._stop_loop_thread.start()

    def _stop_loop(self):
        if self._use_logger is True:
            self._logger.debug("Stopping UDP/IP Client")
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._internal_udp_client = None
        time.sleep(1)
        try:
            self._loop.close()
            if self._use_logger is True:
                self._logger.debug("UDP/IP Client : Loop closed correctly")
        except Exception as ex:
            if self._use_logger is True:
                self._logger.warning("UDP/IP Client : can't close the loop cause: {} ".format(ex))

    def close_connection(self):
        self.stop()


import sys

def main():
    port = int(sys.argv[1])
    server = UDPClient("127.0.0.1", port, test)
    server.initialize_connection()
    count = 0
    stop = False
    while not stop:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            stop = True
    print("Closing transport...")



def test(data):
    print("Data from UDPServer : {}".format(data))


if __name__ == '__main__':
    main()
