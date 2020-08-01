import time
import zmq
import threading


class MADSubscriber(object):
    def __init__(self, address, port, topic, method_to_call):
        self._port = port
        self._address = address
        self._topic = topic.encode()
        self._subscriber = None
        self._context = None
        self._method_to_call = method_to_call

    def initialize_socket(self):
        self._context = zmq.Context()
        self._subscriber = self._context.socket(zmq.SUB)
        self._subscriber.connect("tcp://{}:{}".format(self._address, self._port))
        self._subscriber.setsockopt(zmq.SUBSCRIBE, self._topic)

    def receive_data(self):
        [address, contents] = self._subscriber.recv_multipart()
        self._method_to_call(contents.decode())

    def _receive_data_continuously_inner(self):
        while True:
            self.receive_data()

    def start_receive_data_continuously(self):
        thread_receiver = threading.Thread(target=self._receive_data_continuously_inner, args=())
        thread_receiver.setDaemon(True)
        thread_receiver.start()

    def close_connection(self):
        self._subscriber.close()
        self._context.term()


def print_on_screen(data):
    print(data)

def main():
    m = MADSubscriber("127.0.0.1", 10002, "inclinometer", print_on_screen)
    m.initialize_socket()
    m.start_receive_data_continuously()
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()