import dynamic_reconfigure._dynamic_reconfigure_ as dy

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
    _name = "DynamicReconfigure_" + service_name
    _cfg_list = cfg_list
    dy_wrapper(_name, python_handler, _cfg_list)

