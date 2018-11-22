import dynamic_reconfigure._dynamic_reconfigure_ as dy
import uuid

def reconfigure_global(configs):
    global callback
    callback(configs)
    return 1

def dy_wrapper(service_name, python_handler, cfg_list):
    global callback
    callback = python_handler
    _name = service_name
    _cfg_list = cfg_list
    dy.params_service_init(_name, reconfigure_global, _cfg_list)

def Server(service_name, python_handler, cfg_list):
    _uuid = str(uuid.uuid4()).replace('-', '')
    _name = "_DynamicReconfigure_" + service_name + _uuid
    _cfg_list = cfg_list
    dy_wrapper(_name, python_handler, _cfg_list)

