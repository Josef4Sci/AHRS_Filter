import time
from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt
import vqf
import numpy as np
import pandas as pd
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSInv, JustaAHRSPure, JustaAHRSv2, JustaAHRSv3
from utils import angle_error, eval_filter_on_dataset

plot_result = False

dl = DatasetLoader()
# dataset_name = 'slow_v4.mat'
# dat = dl.load_sassari_dataset(dataset_name, 2)

dat = dl.load_dataset('Justa')

j_filter = JustaAHRSInvFast( w_acc=0.0, w_mag=0.0)

quaternion_result = eval_filter_on_dataset(j_filter, dat)

