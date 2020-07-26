#######################################################
#                                                     #
# NATO STO CMRE - La Spezia                           #
#                                                     #
# Common Python Utilities                             #
# Networking utilities                                #
#                                                     #
#     Author: Renato Sirola (CTR)                     #
#     E-mail: renato.sirola@cmre_.nato.int            #
#       Date: January 2018                            #
# Department: E.D.                                    #
#                                                     #
#######################################################

import threading
import time
import asyncio
import logging


class InternalUDPServer(asyncio.DatagramProtocol):
    def __init__(self, receiving_handle_data=None, logger=None):
        self._receiving_handle_data = receiving_handle_data
        self._stop = False
        self._logger = logger
        self._use_logger = True
        if self._logger is None:
            self._use_logger = False
        self._transport = None
        self._clients = set()

    def connection_made(self, transport: asyncio.DatagramTransport):
        self._transport = transport

    def datagram_received(self, data: bytes, addr: tuple):
        self._clients.add(addr)
        if self._receiving_handle_data is not None:
            self._receiving_handle_data(data)
        else:
            if self._use_logger is not None:
                self._logger.info("Some data are incoming but the _receiving_handle_data() is undefined")

    def send_data_to_clients(self, data, destination_address=None, destination_port=None):
        if destination_address is not None and destination_port is not None:
            if self._use_logger is not None:
                self._logger.info("Sending data to {}:{}".format(destination_address, destination_port))
            try:
                self._transport.sendto(data.encode(), (destination_address, destination_port))
            except Exception as ex:
                if self._use_logger is not None:
                    self._logger.warning("Can't send data, cause: {}".format(ex))
        clients = list(self._clients)
        for client in clients:
            try:
                self._transport.sendto(data.encode(), (client))
            except Exception as ex:
                if self._use_logger is not None:
                    self._logger.warning("Can't send data to {}, cause: {}".format(client, ex))

    def close_active_client_connections(self):
        if self._use_logger is not None:
            self._logger.info("Close connection(s) with clients")
        try:
            self._transport.close()
            self._logger.info("Close connections")
        except Exception as ex:
            self._logger.warning("Can't close connections cause: {}".format(ex))
            self._transport = None


class UDPServer(object):
    def __init__(self, bind_ip_address, bind_port, receiving_handle_data=None, logger=None):
        self._bind_ip_address = bind_ip_address
        self._bind_port = bind_port
        self._receiving_handle_data = receiving_handle_data
        self._loop = None
        self._internal_udp_server = None
        self._internal_loop_thread = None
        self._stop_loop_thread = None
        self._stop = False
        self._bind_ok = False
        self._logger = logger
        self._use_logger = True
        if self._logger is None:
            self._use_logger = False
        if self._use_logger is True:
            self._logger.debug("UDPServer will use logger")
            if self._receiving_handle_data is not None:
                self._logger.info("_receiving_handle_data() method is defined: {}".format(self._receiving_handle_data))
            else:
                self._logger.warning("_receiving_handle_data() method is UNDEFINED (NONE) - the udp/ip server will can only send data to clients")

    def initialize_connection(self):
        if self._use_logger is True:
            self._logger.debug("Initializing UDPServer")
        asyncio.set_event_loop(asyncio.new_event_loop())
        self._loop = asyncio.get_event_loop()
        if self._use_logger is True:
            self._logger.debug("AsyncIO loop created")
        self._internal_udp_server = InternalUDPServer(self._receiving_handle_data, self._logger)
        if self._use_logger is True:
            self._logger.debug("InternalUDPServer created")
        while not self._bind_ok:
            try:
                if self._use_logger is True:
                    self._logger.debug("Try to bind UDPServer at port: {} on this address: {}".format(self._bind_port, self._bind_ip_address))
                coro = self._loop.create_datagram_endpoint(lambda: self._internal_udp_server, local_addr=(self._bind_ip_address, self._bind_port))
                self._bind_ok = True
                server = self._loop.run_until_complete(coro)
                self._internal_loop_thread = threading.Thread(target=self._start_server_loop, args=())
                self._internal_loop_thread.setDaemon(True)
                self._internal_loop_thread.start()

                time.sleep(1)
            except Exception as ex:
                if self._use_logger is True:
                    self._logger.warning("Can't bind at UDP/IP Server at port {} cause : {}".format(self._bind_port, ex))
                self._internal_udp_server.close_active_client_connections()
                time.sleep(1)

    def stop(self):
        if self._use_logger is True:
            self._logger.debug("Request stop UDP/IP")
        self._stop_loop_thread = threading.Thread(target=self._stop_loop, args=())
        self._stop_loop_thread.start()

    def get_loop(self):
        return self._loop

    def _start_server_loop(self):
        if self._use_logger is True:
            self._logger.debug("UDP/IP Server: starting loop/listening on port: {}".format(self._bind_port))
        self._loop.run_forever()

    def send(self, data):
        if self._internal_udp_server is not None:
            self._internal_udp_server.send_data_to_clients(data)

    def _stop_loop(self):
        if self._use_logger is True:
            self._logger.debug("Stopping UDP/IP Server at port {}".format(self._bind_port))
        self._bind_ok = False
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._internal_udp_server.close_active_client_connections()
        self._internal_udp_server = None
        time.sleep(1)
        try:
            self._loop.close()
            if self._use_logger is True:
                self._logger.debug("UDP/IP Server : Loop closed correctly")
        except Exception as ex:
            if self._use_logger is True:
                self._logger.warning("UDP/IP Server : can't close the loop cause: {} ".format(ex))


def test(data):
    try:
        print(data.decode())
    except Exception as ex:
        print(ex)


def main():
    #logging.config.fileConfig(fname='file.conf', disable_existing_loggers=False)
    logging.basicConfig(format='%(asctime)s - %(levelname)s - %(message)s', level=logging.INFO)
    server = UDPServer("0.0.0.0", 5400, test, logger=logging.getLogger("UDPServer"))
    server.initialize_connection()
    count = 0
    stop = False
    while not stop:
        try:
            time.sleep(0.01)
            server.send("{}\r\n".format(randomString(100)))
        except KeyboardInterrupt:
            stop = True
    print("Closing transport...")


import random
import string


def randomString(stringLength=10):
    """Generate a random string of fixed length """
    letters = string.ascii_lowercase
    return ''.join(random.choice(letters) for i in range(stringLength))


if __name__ == '__main__':
    main()