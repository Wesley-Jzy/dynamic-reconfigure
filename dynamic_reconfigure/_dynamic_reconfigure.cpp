#include <Python.h>
#include "rclcpp/rclcpp.hpp"
#include "rcl/rcl.h"
#include "dynamic_reconfigure_server_py.hpp" 
#include "rqt_support.hpp"
#include "rclcpp/parameter.hpp"
#include <thread>

static rclcpp::Parameter _make_param(PyObject* dict, PyObject* key)
{
    std::string key_string= PyUnicode_AsUTF8(key);
    PyObject* value=PyDict_GetItem(dict, key);

    if(PyUnicode_Check(value)){
        return rclcpp::Parameter(key_string, PyUnicode_AsUTF8(value));
    }

    if(PyBool_Check(value)) {
        PyObject_Print(value, stdout, Py_PRINT_RAW);
        if (value==Py_True) {
            return rclcpp::Parameter(key_string, true);
        } else {
            return rclcpp::Parameter(key_string, false);
        }
    }

    if( PyLong_Check(value)) {
        return rclcpp::Parameter(key_string, PyLong_AsLong(value));
    }

    if( PyFloat_Check(value)) {
        return rclcpp::Parameter(key_string, PyFloat_AsDouble(value));
    }
    return rclcpp::Parameter("", 0);
}

static void _set_dict(PyObject* dict, rclcpp::Parameter parameter)
{
    switch(parameter.get_type()){
        case rclcpp::ParameterType::PARAMETER_BOOL:
            if(parameter.as_bool()) {
                PyDict_SetItemString(dict, parameter.get_name().data(), Py_True);
            } else {
                PyDict_SetItemString(dict, parameter.get_name().data(), Py_False);
            }
            return;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            PyDict_SetItemString(dict, parameter.get_name().data(), PyLong_FromLong(parameter.as_int()));
            return;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            PyDict_SetItemString(dict, parameter.get_name().data(), PyFloat_FromDouble(parameter.as_double()));
            return;
        case rclcpp::ParameterType::PARAMETER_STRING:
            PyDict_SetItemString(dict, parameter.get_name().data(), PyUnicode_FromString(parameter.as_string().data()));
            return;
        default:
            ;
    }
}

static PyObject *get_description(PyObject * Py_UNUSED(self), PyObject * args)
{

    char* remote_name;
    if (!PyArg_ParseTuple(args, "z", &remote_name)) {
        return NULL;
    }

    //auto node = rclcpp::Node::make_shared("get_parameters_try_client");
    std:: shared_ptr<rqt_reconfigure::Client> test_client = rqt_reconfigure::client_map.get_client(remote_name);
    //rqt_reconfigure::Client test_client = rqt_reconfigure::Client(node, remote_name);
    std::vector<rclcpp::Parameter> descritions = test_client->get_description();
    PyObject* dict=PyDict_New();
    for (auto & parameter : descritions) {
        _set_dict(dict, parameter);
    }
    //rclcpp::shutdown();
    return dict;
}
static PyObject *get_values(PyObject * Py_UNUSED(self), PyObject * args)
{
    char* remote_name;
    if (!PyArg_ParseTuple(args, "z", &remote_name)) {
        return NULL;
    }

    //auto node = rclcpp::Node::make_shared("get_parameters_try_client");

    std:: shared_ptr<rqt_reconfigure::Client> test_client = rqt_reconfigure::client_map.get_client(remote_name);
    std::vector<rclcpp::Parameter> values = test_client->get_values();
    PyObject* dict=PyDict_New();
    for (auto & parameter : values) {
        _set_dict(dict, parameter);
    }
    //rclcpp::shutdown();
    return dict;
}

static  PyObject *update_parameters(PyObject * Py_UNUSED(self), PyObject *args)
{
    //rcl_shutdown();
    //auto parameters_client=_init_client("set_parameters_client");

    PyObject* dict;
    char* remote_name;
    PyArg_ParseTuple(args, "zO", &remote_name, &dict);
    //printf("remote_name = %s\n", remote_name);
    PyObject* l=PyDict_Keys(dict);
    Py_ssize_t i=0;
    std::vector<rclcpp::Parameter> para;
    for(i=0;i<PyList_GET_SIZE(l);++i){
        PyObject* key=PyList_GetItem(l,i);
        if(PyUnicode_Check(key)){
            para.push_back(_make_param(dict, key));
        }
    }

    //auto node = rclcpp::Node::make_shared("get_parameters_try_client");
    std:: shared_ptr<rqt_reconfigure::Client> test_client = rqt_reconfigure::client_map.get_client(remote_name);
    test_client->update_params(para);

    std::vector<rclcpp::Parameter> values = test_client->get_values();

    PyObject* ans=PyDict_New();
    for (auto & parameter : values) {
        _set_dict(ans, parameter);
    }

    return ans;

}



static rclcpp::Parameter _make_param_add_name(PyObject* dict, PyObject* key, std::string name, std::string short_form)
{
    std::string key_string= short_form + '.' + name;
    PyObject* value=PyDict_GetItem(dict, key);

    if(PyUnicode_Check(value)){
        return rclcpp::Parameter(key_string, PyUnicode_AsUTF8(value));
    }

    if(PyBool_Check(value)) {
        //PyObject_Print(value, stdout, Py_PRINT_RAW);
        if (value==Py_True) {
            return rclcpp::Parameter(key_string, true);
        } else {
            return rclcpp::Parameter(key_string, false);
        }
    }

    if( PyLong_Check(value)) {
        return rclcpp::Parameter(key_string, PyLong_AsLong(value));
    }

    if( PyFloat_Check(value)) {
        return rclcpp::Parameter(key_string, PyFloat_AsDouble(value));
    }
    return rclcpp::Parameter("", 0);
}



static void workThread(char* service_name, PyObject *python_handle, std::vector<rclcpp::Parameter> cfg_all) {
    if (!PyEval_ThreadsInitialized()) {
        PyEval_InitThreads();
    }
    rqt_reconfigure::Server_py(service_name, python_handle, cfg_all);
}


static  PyObject *params_service_init(PyObject * Py_UNUSED(self), PyObject *args)
{
    char* service_name;
    PyObject *python_handle;
    PyObject *cfg_list;
    PyArg_ParseTuple(args, "zOO", &service_name, &python_handle, &cfg_list);

    std::vector<rclcpp::Parameter> cfg_all;

    for(int i = 0; i < PyList_GET_SIZE(cfg_list); ++i) {
        PyObject* param = PyList_GetItem(cfg_list, i);
        PyObject* name = PyDict_GetItem(param, Py_BuildValue("s","name"));
        std::string name_c = PyUnicode_AsUTF8(name);
        cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","name"), name_c, "name"));
        cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","description"), name_c, "des"));
        cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","edit_method"), name_c, "edit"));
        cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","type"), name_c, "type"));
        cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","level"), name_c, "lev"));
        cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","min"), name_c, "min"));
        cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","max"), name_c, "max"));
        cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","default"), name_c, "default"));
        cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","default"), name_c, "value"));
    }
    std::thread t(workThread, service_name, python_handle, cfg_all);
    t.detach();
    //rqt_reconfigure::Server_py(service_name, python_handle, cfg_all);
    Py_RETURN_NONE;
}

static PyMethodDef module_methods[] =
{
    {
        "params_service_init", params_service_init, METH_VARARGS,
        "params_service_init"
    },
    {
        "get_description", get_description, METH_VARARGS,
        "get_description good"
    },
    {
        "get_values", get_values, METH_VARARGS,
        "get_values good"
    },
    {
        "update_parameters", update_parameters, METH_VARARGS,
        "update_parameters good"
    },
    {NULL, NULL, 0, NULL}  /* sentinel */
};
PyDoc_STRVAR(dynamic_reconfigure__doc__,
            "dynamic_reconfig module: params_service_init");
static struct PyModuleDef dynamic_reconfigure_module =
{
    PyModuleDef_HEAD_INIT,
    "dynamic_reconfigure",
    dynamic_reconfigure__doc__,
    -1, /* -1 means that the module keeps state in global variables */
    module_methods,
    NULL,
    NULL,
    NULL,
    NULL
};
PyMODINIT_FUNC PyInit__dynamic_reconfigure_(void)
{

    PyObject *m;
    m = PyModule_Create(&dynamic_reconfigure_module);
    if( m == NULL)
    {
        return NULL;
    }


    return m;
}

