import time
import zmq


class MADPublisher(object):
    def __init__(self, bind_port, topic):
        self._bind_port = bind_port
        self._topic = topic.encode()
        self._publisher = None
        self._context = None

    def initialize_socket(self):
        self._context = zmq.Context()
        self._publisher = self._context.socket(zmq.PUB)
        self._publisher.bind("tcp://*:{}".format(self._bind_port))

    def send_data(self, data_to_send):
        self._publisher.send_multipart([self._topic, data_to_send.encode()])

    def close_connection(self):
        self._publisher.close()
        self._context.term()


def main():
    p = MADPublisher(12000, "pippo")
    p.initialize_socket()
    counter = 0
    while True:
        p.send_data("{}".format(counter))
        #time.sleep(1)
        counter += 1


if __name__ == "__main__":
    main()