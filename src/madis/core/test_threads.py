import time
import threading
import os


def access_to_dictionary(stop, dictionary):
    while not stop:
        value = dictionary["value"]
        os.system("clear")
        print("value : {}".format(value))
        #time.sleep(0.01)


def write_on_dictionary(stop, dictionary):
    counter = 0
    while not stop:
        counter += 1
        dictionary["value"] = counter
        #time.sleep(0.01)


def main():
    dictionary = dict()
    dictionary["value"] = None
    stop = False
    thread_writer = threading.Thread(target=write_on_dictionary, args=(stop, dictionary))
    thread_writer.setDaemon(True)

    thread_reader = threading.Thread(target=access_to_dictionary, args=(stop, dictionary))
    thread_writer.setDaemon(True)

    thread_writer.start()
    time.sleep(1)
    thread_reader.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop = True
        print("stopping threads...")
        exit()


if __name__ == "__main__":
    main()

