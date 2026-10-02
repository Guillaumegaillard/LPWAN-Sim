# -*- coding: utf-8 -*-
######## Build a reference scenario, specific network parameters and several examples ########
######## https://github.com/Guillaumegaillard/LPWAN-Sim ######################################
######## 2026-04-28 ##########################################################################
######## License MIT: ########################################################################
######## https://github.com/Guillaumegaillard/LPWAN-Sim/blob/main/LICENSE ####################
######## Inspired from LoRaSim https://mcbor.github.io/lorasim/ ##############################

import pickle
import constants
import os


# build a folder where will be stored simulation setups and results
if not os.path.exists('results'):
    os.makedirs('results')

# Prepares default network parameters, meant to serve as a reference scenario. 
# called in lpwan_simulator.py
def prepare_default_network(nrNodes=10, nrGWs=2, avgSendTime=10000, targetSchedPacket=10, modulation="LoRa-CSS"):
    net = {
        "maxDist_dev_gw":100, # improperly named, corresponds to the radius of the topology for this net (100m)
        "nrNodes":nrNodes,
        "nrGWs":nrGWs,
        "avgSendTime":avgSendTime,
        "targetSchedPacket":targetSchedPacket,
        "modulation":modulation,
        "LR-FHSS-CR":"1/3",

                    ######### Channel properties ################
        "CAD_prob": 50,
        "var_CAD_prob": False,
        "full_distances": False,
        "lora24GHz": False,
        "full_collision": True,

        "capture_interf_lock_coef": .1, #6# dB
        "gamma_ED": constants.gamma,
        "gamma_GW": constants.gamma_GW,

        "normal_gamma_ED": True,
        "sigma_gamma_ED": constants.sigma_gamma_ED,

        "CAD_gamma_ED_delta": constants.CAD_gamma_ED_delta,

        "CAD_normal_gamma_ED": True,
        "CAD_sigma_gamma_ED": constants.CAD_sigma_gamma_ED,

        "intra_cluster_gamma_gain": 0.,

        "GW_sensitivity_gain": 5, #dB

                    ######### Distribution&Traffic properties ################
        "externals_prop": 0.,

        "radio_config_time": 5, #ms,
        "radio_wake_up_time": 5, #ms,

        # default behavior: random choice of channel and blacklist until all but one are blacklisted 
        "tx_blacklist_memory": len(constants.Channels[modulation]["Channel_list"]), #in order to follow predefined CH sequences, must be changed to 0
        "tx_blacklist_update_strategy": "empty_but_last", #"shift",




        "maxBSReceives": 8,
        "packetLength": 104,
        "variablePayloadSize": False,
        "normalPayloadSize": False,
        "dist_min_payload_size": 40,
        "dist_max_payload_size": 100,
        "normaldist_mean_payload_size": 60,
        "normaldist_sigma_payload_size": 15,

        "max_payload_size": 150,
        "targetSentPacket": 2000,



        # distribType=uniformDistribType
        # distribType=expoDistribType
        # distribType=perioDistribType
        "distribType": constants.perioDistribType,
        "shuffle_start": False,

                    ######### Simulation properties ################
        # "exp4SF": 12,
        "net_SF": 12,
        "net_BW": 125,
        "net_CR": 1,


        "Ptx": constants.Ptx_subGHz,
        "minsensi": constants.sensi_subGHz[6,2],     # 6th row is SF12, 2nd column is BW125
        "transmit_at_adjusted_power": False,
        "max_queue_size": 8,

        "nCadSym": 4,
        "detection_min_preamb_symb": 3,

        "LoRa_PHY_HDR": 0,             # explicit header (H=0) or implicit header (H=1),


    }

    return net


# compose a specific/parametric network 
def prepare_network(nrNodes, nrGWs, avgSendTime, targetSchedPacket, modulation, proto, maxDist_dev_gw =2500, experiment = 4, CAD_always_true = False):

    net = {     
        "maxDist_dev_gw":maxDist_dev_gw,

                ######### Channel properties ################
        # "use_CADs":False,
        "use_CADs":True,
        "CAD_prob": 100 if CAD_always_true else 50,
        "var_CAD_prob": False if CAD_always_true else True, #has precedence
        "full_distances": True,
        "full_collision": True,

        "capture_interf_lock_coef": .1,#6,# dB,
        "gamma_ED": constants.gamma,
        "gamma_GW": constants.gamma_GW,

        "normal_gamma_ED": True,
        "sigma_gamma_ED": constants.sigma_gamma_ED,
        "intra_cluster_gamma_gain": 0.2,

        "CAD_gamma_ED_delta": constants.CAD_gamma_ED_delta,
        "CAD_normal_gamma_ED": True,
        "CAD_sigma_gamma_ED": constants.CAD_sigma_gamma_ED,

        "GW_sensitivity_gain": .5, #dB,

                ######### Distribution&Traffic properties ################
        "nrNodes":nrNodes,
        "nrGWs":nrGWs,
        "externals_prop":0.,#3,

        "radio_config_time":5, #ms
        "radio_wake_up_time":5, #ms


        "tx_blacklist_memory": len(constants.Channels[modulation]["Channel_list"]), #in order to work with node CH sequences, must be 0
        "tx_blacklist_update_strategy": "empty_but_last", #"shift",


        "avgSendTime":avgSendTime,
        "variablePayloadSize":True,
        "normalPayloadSize":True,
        "dist_min_payload_size":40, # proba. distrib of sizes is bounded at this min
        "dist_max_payload_size":100,
        "normaldist_mean_payload_size":60,
        "normaldist_sigma_payload_size":15,

        "max_payload_size":150, # max_expected_payload_size, a protocol parameter
        "targetSchedPacket":targetSchedPacket,

        "distrib":"expo", #traffic distrib, can be exponential (expo), periodic (periodic, default), uniform (unif), specific (spe100, spe500)
        
        "shuffle_start":False,

        "experiment": experiment, # somehow obsolete

        "max_queue_size":2,
        "nCadSym":4, 

        "detection_min_preamb_symb":6,

        "transmit_at_adjusted_power":False,

                ######### Modulation/Techno properties ################
        "LoRa_PHY_HDR":0,
        "modulation":modulation,
        "LR-FHSS-CR":"1/3",


        ## in the following, non-null proportions in [0,1] must represent at least one device (TODO: improve cluster definitions)
        ## watch it with externals_prop value
        "node_profiles" : {
            "clustered":{ # distribution of nodes inside clusters
                "distribType":"full_cluster", # "node-wise"
                "distrib":{
                    proto:1,
                    # "ideal_FIFO":.5,
                    # "ALOHA":0,
                }
            },
            "unclustered":{ # distribution of nodes outside clusters
                "distrib":{
                    proto:1,
                }
            },
            #TODO: adapt node stats variable updates for non aloha externals
            "externals":{ # distribution of interfering nodes
                "distrib":{
                    "ALOHA":1,
                }
            }
        },

        # specific parameters for each node profile (protocol)
        "profiles":{
            "LoRa_CSMA":{
                "protocol":"CAD+Backoff",
                "n_retry":6,
                "abort_after_retries":False,
                "Wultim_min_sym": 0,#66,
                "Wultim_max_sym": 0,#132,     
                "Wclear_min_CAD":2,           
                "Wclear_max_CAD":22,           
                "nCAD_DIFS":2,      
                "DIFS_inter_CAD_sym":0,
                "BO_inter_CAD_sym":0,
                "CH_DIFS":False,
                "active_BO_when_busy":False,
                "passive_BO_when_busy":False,
                "BO_when_clear":True,
                "active_clear_BO":True,
                "CA_Change_Channel":True,
            },            
            

            "xCANL_CAD":{


                "protocol":"CANL22",
                "n_retry":6,
                "abort_after_retries":False,
                "Wcanl_min_sym":0,
                "Wcanl_max_sym":50,

                "nCAD_DIFS":2,      
                "DIFS_inter_CAD_sym":5,    
                "CH_DIFS":True,

                "CANL22_P": 0,
                "CANL22_L1_min": 6,
                "CANL22_L1_MAX": 16,
                "CANL22_L2": 6,
                "CANL22_check_busy": True,
                "CANL22_RTS_min_payload_size": 100000000000, # no RTS
                "CANL22_fair_factor": 1, # 0 means no reduction
                "CANL22_softer_fair": False,

                "CANL22_RTS_hdr_size": 4, 
                "CANL22_RTS_PHY_HDR": 1,
                "CANL22_data_hdr_size": 0,

                "CANL22_MAX_NAV_x_chan_threshold": 2800, # ms         

                "Wclear_min_sym":0,
                "Wclear_max_sym":20,
                "BO_when_clear":True,
                "active_clear_BO":False,


                "Interrupts_on_header_valid":True,

            },
            "ideal_FIFO":{
                "protocol":"ideal_FIFO",
            },
            "ALOHA":{
                "protocol":"ALOHA",
            },
            "CANL_2022":{
                "protocol":"CANL22",
                "n_retry":5,
                "abort_after_retries":True,#True,
                # "Wcanl_min_sym":0,
                # "Wcanl_max_sym":6,
                "nCAD_DIFS":0,      
                "DIFS_inter_CAD_sym":0,    

                "CANL22_P": 0,
                "CANL22_L1_min": 5,
                "CANL22_L1_MAX": 120,
                # "CANL22_L2": 6,
                "CANL22_check_busy": False,
                "CANL22_RTS_min_payload_size": 120000000,
                # "CANL22_fair_factor": 4, # 0 means no reduction
                "CANL22_fair_factor": 1, # 0 means no reduction
                "CANL22_softer_fair": False,

                "CANL22_RTS_hdr_size": 4, 
                "CANL22_RTS_PHY_HDR": False, #(implicit header for rts (header is in upper layer))
                "CANL22_data_hdr_size": 4,

                "CANL22_MAX_NAV_x_chan_threshold": 22222200, # ms       

                "BO_when_clear":False,  

                "Interrupts_on_header_valid":True,
            },

        }

    }

    return net


# change parameters in a previously defined net
# meant to have a more complex validation process than default Python update() 
def update_network(params, new_params):
    params.update(new_params)

# parametric radio model definitions
def prepare_radio_params(correlated_noise = False, rayleigh_fading = False, locally_obstructed = False, buildings_attenuated = False):
    return {
        "gaussian_noise": False,
        "correlated_noise": correlated_noise,
        "corr_noise_white_scale": 32,
        "corr_noise_gauss_filter_sigma": 12,
        "correlated_noise_update_period": 5000, #ms,
        "corr_noise_map_xsize":1000,
        "corr_noise_map_ysize":1000,

        "rayleigh_fading": rayleigh_fading,
        "rayleigh_mean_dB":1,
        # "rayleigh_mean_dB": 0,

        "locally_obstructed": locally_obstructed,
        "buildings_attenuated": buildings_attenuated,        
        "buildings_km": 0.5,
    }



#
# "main" program as used as "python config_networks.py"
# builds and dumps various example scenarios:
#   - LoRa-CSS
#   - LR-FHSS
#   - Mioty
#   - simple "coexistent"
#   - "complex_coexistent"



if __name__ == '__main__':

    import json
    JSON_EXPORT = True

    ###################################################
    # basic LoRa CSS example
    start_time = "LoRa-CSS_example"
    networks = []
    networks.append(prepare_network(
        nrNodes = 50,
        nrGWs = 3, 
        avgSendTime = 10000, 
        targetSchedPacket = 10, 
        modulation = "LoRa-CSS", 
        proto = "ALOHA"
        )
    )
    params = {
        "start_time":start_time,
        "pickle_each":False,
        "run_index":0,
        "log_events":False,
        "keep_chan_log":False,
        "keep_Global_TT_IGTs":False,
        "topo_scale":1,
        "topo":0#repe
    }
    params["networks"] = networks
    params["radio_params"] = prepare_radio_params(correlated_noise = False)

    pickle.dump(params, open('results/{0}_params.dat'.format(start_time), 'wb'))

    if JSON_EXPORT:
        json_dict = json.dumps(params, indent=4)
        with open('results/{0}_params.json'.format(start_time), "w") as outfile:
            outfile.write(json_dict)



    ###################################################
    # basic LR-FHSS example
    start_time = "LR-FHSS_example"
    networks = []
    networks.append(prepare_network(
        nrNodes = 50,
        nrGWs = 3, 
        avgSendTime = 10000, 
        targetSchedPacket = 10, 
        modulation = "LR-FHSS", 
        proto = "ALOHA"
        )
    ) 


    params = {
        "start_time":start_time,
        "pickle_each":False,
        "run_index":0,
        "log_events":False,
        "keep_chan_log":False,
        "keep_Global_TT_IGTs":False,
        "topo_scale":1,
        "topo":0#repe
    }
    params["networks"] = networks
    params["radio_params"] = prepare_radio_params(correlated_noise = True)
    # params["radio_params"] = prepare_radio_params(correlated_noise = True, rayleigh_fading = True, locally_obstructed = True, buildings_attenuated = True)


    pickle.dump(params, open('results/{0}_params.dat'.format(start_time), 'wb'))

    if JSON_EXPORT:
        json_dict = json.dumps(params, indent=4)
        with open('results/{0}_params.json'.format(start_time), "w") as outfile:
            outfile.write(json_dict)


    ###################################################
    # basic Mioty example
    start_time = "Mioty_example"
    networks = []
    networks.append(prepare_network(
        nrNodes = 50,
        nrGWs = 3, 
        avgSendTime = 10000, 
        targetSchedPacket = 10, 
        modulation = "Mioty", 
        proto = "ALOHA"
        )
    )

    params = {
        "start_time":start_time,
        "pickle_each":False,
        "run_index":0,
        "log_events":False,
        "keep_chan_log":False,
        "keep_Global_TT_IGTs":False,
        "topo_scale":1,
        "topo":0#repe
    }
    params["networks"] = networks
    params["radio_params"] = prepare_radio_params(correlated_noise = True)
    # params["radio_params"] = prepare_radio_params(correlated_noise = True, rayleigh_fading = True, locally_obstructed = True, buildings_attenuated = True)

    pickle.dump(params, open('results/{0}_params.dat'.format(start_time), 'wb'))

    if JSON_EXPORT:
        json_dict = json.dumps(params, indent=4)
        with open('results/{0}_params.json'.format(start_time), "w") as outfile:
            outfile.write(json_dict)


    ###################################################
    ###################################################
    # Simple coexistent LoRa-CSS + LR-FHSS + Mioty example
    start_time = "coexistent_example"
    networks = []
    networks.append(prepare_network(
        nrNodes = 10,
        nrGWs = 2, 
        avgSendTime = 20000, 
        targetSchedPacket = 10, 
        modulation = "LoRa-CSS", 
        proto = "ALOHA"
        )
    ) 
    networks.append(prepare_network(
        nrNodes = 10,
        nrGWs = 2, 
        avgSendTime = 20000, 
        targetSchedPacket = 10, 
        modulation = "LR-FHSS", 
        proto = "ALOHA"
        )
    )
    networks.append(prepare_network(
        nrNodes = 10,
        nrGWs = 2, 
        avgSendTime = 20000, 
        targetSchedPacket = 10, 
        maxDist_dev_gw = 2500,
        modulation = "Mioty", 
        proto = "ALOHA"
        )
    )

    params = {
        "start_time":start_time,
        "pickle_each":False,
        "run_index":0,
        "log_events":False,
        "keep_chan_log":True,
        "keep_Global_TT_IGTs":False,
        "topo_scale":1,
        "topo":0#repe
    }
    params["networks"] = networks
    params["radio_params"] = prepare_radio_params(correlated_noise = False)
    # params["radio_params"] = prepare_radio_params(correlated_noise = True, rayleigh_fading = True, locally_obstructed = True, buildings_attenuated = True)


    pickle.dump(params, open('results/{0}_params.dat'.format(start_time), 'wb'))

    if JSON_EXPORT:
        json_dict = json.dumps(params, indent=4)
        with open('results/{0}_params.json'.format(start_time), "w") as outfile:
            outfile.write(json_dict)



    ###################################################
    ###################################################
    # complex coexistent 3*LoRa-CSS + 2*LR-FHSS + Mioty example
    start_time = "complex_coexistent_example"
    networks = []
    networks.append(prepare_network(
        nrNodes = 10,
        nrGWs = 2, 
        avgSendTime = 20000, 
        targetSchedPacket = 10, 
        modulation = "LoRa-CSS", 
        proto = "ALOHA"
        )
    )
    networks.append(prepare_network(
        nrNodes = 10,
        nrGWs = 2, 
        avgSendTime = 20000, 
        targetSchedPacket = 10, 
        modulation = "LoRa-CSS", 
        proto = "xCANL_CAD",
        CAD_always_true = True
        )
    )
    networks.append(prepare_network(
        nrNodes = 10,
        nrGWs = 2, 
        avgSendTime = 20000, 
        targetSchedPacket = 10, 
        modulation = "LoRa-CSS", 
        proto = "CANL_2022",
        CAD_always_true = True
        )
    )    
    networks.append(prepare_network(
        nrNodes = 10,
        nrGWs = 2, 
        avgSendTime = 20000, 
        targetSchedPacket = 10, 
        modulation = "LR-FHSS", 
        proto = "ALOHA"
        )
    )
    networks.append(prepare_network(
        nrNodes = 10,
        nrGWs = 2, 
        avgSendTime = 20000, 
        targetSchedPacket = 10, 
        maxDist_dev_gw = 2500,
        modulation = "LR-FHSS", 
        proto = "ALOHA"
        )
    )
    update_network(networks[-1], {"LR-FHSS-CR":"2/3"})

    networks.append(prepare_network(
        nrNodes = 10,
        nrGWs = 2, 
        avgSendTime = 20000, 
        targetSchedPacket = 10, 
        maxDist_dev_gw = 2500,
        modulation = "Mioty", 
        proto = "ALOHA"
        )
    )


    params = {
        "start_time":start_time,
        "pickle_each":False,
        "run_index":0,
        "log_events":False,
        "keep_chan_log":False,
        "keep_Global_TT_IGTs":False,
        "topo_scale":1,
        "topo":0#repe
    }
    params["networks"] = networks
    # params["radio_params"] = prepare_radio_params(correlated_noise = False)
    params["radio_params"] = prepare_radio_params(correlated_noise = True, rayleigh_fading = True, locally_obstructed = True, buildings_attenuated = True)

    pickle.dump(params, open('results/{0}_params.dat'.format(start_time), 'wb'))

    if JSON_EXPORT:
        json_dict = json.dumps(params, indent=4)
        with open('results/{0}_params.json'.format(start_time), "w") as outfile:
            outfile.write(json_dict)

