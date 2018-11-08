import dynamic_reconfigure._dynamic_reconfigure_ as dy

def reconfigure_global(configs):
    global callback
    callback(configs)
    return 1

def dy_wrapper(service_name, python_handler, cfg_list):
    dy.params_service_init(service_name, reconfigure_global, cfg_list)

    global callback
    callback = python_handler

def Server(service_name, python_handler, cfg_list):
    dy_wrapper(service_name, python_handler, cfg_list)

