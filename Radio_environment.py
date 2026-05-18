# -*- coding: utf-8 -*-
######## A Radio environment for the LPWAN-Sim Simulator #####################################
######## https://github.com/Guillaumegaillard/LPWAN-Sim ######################################
######## 2026-05-15 ##########################################################################
######## License MIT: ########################################################################
######## https://github.com/Guillaumegaillard/LPWAN-Sim/blob/main/LICENSE ####################
######## Inspired from LoRaSim https://mcbor.github.io/lorasim/ ##############################


import numpy as np
from scipy.ndimage import gaussian_filter as scipy_ndimage_gaussian_filter

import constants

rng = np.random.default_rng()

## This class periodically updates a noise map with spatial correlations, using a Gaussian filter.
class Radio_environment:
    def __init__(self, simu):
        self.simu = simu
        self.noise_map = np.zeros((simu.radio_params["corr_noise_map_xsize"],simu.radio_params["corr_noise_map_ysize"]))
        # self.noise_map = np.zeros((5000,5000))

    def refresh_correlated_noise_map(self):
        
        # white_noise = rng.normal(loc=constants.noise_mu_dB, scale=32, size=self.noise_map.shape)
        white_noise = rng.normal(loc=constants.noise_mu_dB, scale=self.simu.radio_params["corr_noise_white_scale"], size=self.noise_map.shape)

        self.noise_map = np.clip(scipy_ndimage_gaussian_filter(white_noise, sigma=self.simu.radio_params["corr_noise_gauss_filter_sigma"]),0,2*constants.noise_mu_dB)    
        # self.noise_map = np.clip(scipy_ndimage_gaussian_filter(white_noise, sigma=12),0,2*constants.noise_mu_dB)    

    def correlated_noise_map(self, update_period):
        # while nrScheduled < targetSchedPacket:
        while not self.simu.check_all_networks_finished():
            # print("noise updated",file=sys.stderr)
            self.refresh_correlated_noise_map()
            self.simu.update_noise(self.noise_map)
            yield self.simu.env.timeout(update_period)


