import importlib


def instantiate_class(module_name, class_name, constructor_parameters=[]):
    try:
        module = importlib.import_module(module_name)
        class_ = getattr(module, class_name)
        class_instantiated = class_(*constructor_parameters)
        return class_instantiated
    except Exception as ex:
        message = "Introspection Exception : try to create this class [{}] in this module : [{}]".format(class_name, module_name)
        message += "Cannot instantiate the requested object class, cause: {}".format(str(ex))
        raise Exception(message)
