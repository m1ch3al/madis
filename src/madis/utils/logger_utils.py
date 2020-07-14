import os
import yaml
import logging.config
import logging
import time


def create_logger(logger_name, logger_config_file):
    if os.path.exists(logger_config_file):
        with open(logger_config_file, "rt") as f:
            try:
                config = yaml.safe_load(f.read())
                config["handlers"]["file_handler"]["filename"] = time.strftime(config["handlers"]["file_handler"]["filename"])
                prepare_loggin_directory(config["handlers"]["file_handler"]["filename"])
                logging.config.dictConfig(config)
                logger = logging.getLogger(logger_name)
                logger.debug("Logger initialized with this CONFIG-FILE:" + str(logger_config_file))
                return logger
            except Exception as ex:
                print(ex)
                print('Error in Logging Configuration. Using default configs')
                return create_default_file_logger()
    else:
        raise Exception("The logger configuration file, '{}' does not exist".format(logger_config_file))


def get_directory_from_filepath(file_path):
    absolute_path = os.path.abspath(file_path)
    return os.path.dirname(absolute_path)


def directory_exist(path):
    return os.path.exists(path) and os.path.isdir(path)


def create_folder(folder_to_create):
    if not os.path.exists(folder_to_create):
        os.makedirs(folder_to_create)


def file_exist(file_path):
    return os.path.exists(file_path) and os.path.isfile(file_path)


def prepare_loggin_directory(filepath):
    logging_directory = get_directory_from_filepath(filepath)
    if not directory_exist(logging_directory):
        create_folder(logging_directory)


def get_logger(logger_name):
    return logging.getLogger(logger_name)


def create_default_file_logger(filename):
    import time
    import logging
    from logging import FileHandler
    log_filename = "/tmp/" + time.strftime("%Y%m%dT%H%M%S_") + "_" + filename
    file_handler = FileHandler(log_filename)
    formatter = logging.Formatter("%(asctime)s - [%(levelname)s] : %(message)s", "%Y-%m-%d %H:%M:%S")
    file_handler.setFormatter(formatter)
    logger = logging.getLogger("default_file_logger")
    logger.addHandler(file_handler)
    logger.setLevel(logging.DEBUG)



def main():
    logger = create_logger("test", "logging.yaml")
    logger.info("TEST")
    logger.info("TEST2")
    logger.info("TEST3")

if __name__ == "__main__":
    main()


