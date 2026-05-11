#!/usr/bin/env python3


"""
When comparing multiple planner / robots etc. it is advisable to join the data into one large dataset
"""

import json
import pandas as pd
import numpy as np
import glob  # usefull for listing all files of a type in a directory
import os, time
import yaml, math
import sys, re
from pyarrow import feather

def list_files():
    """finds all individual datasets and returns list of datasets
    """
    dir_path = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.dirname(dir_path) + "/02_evaluation/ftr_data"
    # now = time.strftime("%y-%m-%d_%H-%M-%S")
    files = [_temp.split('/')[-1] for _temp in glob.glob(f"{data_dir}/*.ftr")]
    return files

def prepare_dataset():
    return pd.DataFrame({'Planner':[],'Robot':[], 'World':[], 'Dyn_obs':[]})

def extend_dataset(data, file:str):
    _new_data = pd.read_feather(os.path.dirname(os.path.abspath(__file__)) + '/ftr_data/' + file)
    _file_meta_infos = file.split('_')
    if 'turtlebot3' in _file_meta_infos:
        _file_meta_infos[2] = f'{_file_meta_infos[2]}_{_file_meta_infos[3]}'
        _file_meta_infos = _file_meta_infos[:3] + _file_meta_infos[4:]
    if 'warehouse' in _file_meta_infos:
        _file_meta_infos[3] = f'{_file_meta_infos[3]}_{_file_meta_infos[4]}'
        _file_meta_infos = _file_meta_infos[:4] + _file_meta_infos[5:]

    _new_data['Planner'], _new_data['Robot'], _new_data['World'] = _file_meta_infos[1:-1]
    _new_data['Dyn_obs'] = int(re.search(r'\d+', _file_meta_infos[-1]).group())
    return data.append(_new_data)

def simplify_data(_data):
    for e in ['Planner', 'Robot', 'World', 'Dyn_obs']:
        _data[e] = _data[e].astype("category")
    return _data

def save_data(data):
    dir_path = os.path.dirname(os.path.abspath(__file__))
    data.reset_index().to_feather(dir_path+"/data.ftr")


if __name__ == "__main__":
    files = list_files()
    data = prepare_dataset()

    for file in files:
        data = extend_dataset(data, file)

    data = simplify_data(data)

    save_data(data)