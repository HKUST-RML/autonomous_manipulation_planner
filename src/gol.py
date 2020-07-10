#!/usr/bin/env python
global _global_dict
_global_dict = {}
 
 
def set_value(key,value):
    global _global_dict
    _global_dict[key] = value
 
 
def get_value(key,defValue=None):
    global _global_dict
    try:
        return _global_dict[key]
    except KeyError:
        return defValue
