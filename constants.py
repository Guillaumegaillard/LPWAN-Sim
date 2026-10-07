# -*- coding: utf-8 -*-
######## Some constants for the for the LPWAN-Sim Simulator ##################################
######## https://github.com/Guillaumegaillard/LPWAN-Sim ######################################
######## 2026-04-30 ##########################################################################
######## License MIT: ########################################################################
######## https://github.com/Guillaumegaillard/LPWAN-Sim/blob/main/LICENSE ####################
######## Inspired from LoRaSim https://mcbor.github.io/lorasim/ ##############################

import numpy as np

## End Device (ED) definitions (constant definitions may be by-passed in scenario's parameters) 
Ptx_2dot4GHz = 10                   # transmission power for 2.4GHz scenarios 
Ptx_subGHz = 14                     # transmission power for 868MHz scenarios

## radio conditions
gamma = 3                           # Path loss exponent for ED-ED links
gamma_GW=2.95                       # Path loss exponent for ED-GW links

sigma_gamma_ED=.25                  # standard deviation for path loss exponent of ED-ED links

d0 = 40.0                           # Path loss ref. dist
var = 0                             # variance ignored for now
Lpld0 = 83                          # Path loss at ref. dist
GL = 0                              # ED antenna gain
GL_GW = 1.5                         # GW antenna gain (dB)


################ A different path-loss model for CAD ||| but in current version (2025-04-04), CAD == RX
CAD_gamma_ED_delta = 0#gamma
CAD_sigma_gamma_ED=.25              # standard deviation for path loss exponent of ED-ED CAD links
CAD_d0 = 40.0
CAD_var = 0                         # variance ignored for now
CAD_Lpld0 = Lpld0
CAD_GL = 0
###############


########################## ENERGY CONSTANTS
# Transmit consumption in mA from -2 to +17 dBm
# TODO: same for lora24GHz
# not in use in current version (2025-04-04)
TX = [
    22, 22, 22, 23,                                             # RFO/PA0: -2..1
    24, 24, 24, 25, 25, 25, 25, 26, 31, 32, 34, 35, 44,         # PA_BOOST/PA1: 2..14
    82, 85, 90,                                                 # PA_BOOST/PA1: 15..17
    105, 115, 125]                                              # PA_BOOST/PA1+PA2: 18..20

# SX1262_dsV2_1
# static not in use in current version (2025-04-04)
TX_868_14dBm=[45]*23                                            # mA


# feb 2025: interpretation from SX1262_dsV2_1 PA HIGH
power_levels = [i for i in range(-9,23)] #dBm
TX_currents = [22,23,24,25,30,31.5,33,36,38.5,40.1,42,46,48,53,54,58,60,62,66,70,74,78.5,82,90,94,95,96,98,100,104,109,119] # mA

#### PA optimal settings are used to maximize the PA efficiency when the requested output power is lower than the nominal
# => 90 -> 45 ===> PA LOW
power_levels = [i for i in range(-17,15)] #dBm
TX_currents = [9.0, 9.1, 9.2, 9.4, 9.6, 9.9, 10.2, 10.6, 11.0, 11.5, 12.0, 12.5, 15.0, 15.75, 16.5, 18.0, 19.25, 20.05, 21.0, 23.0, 24.0, 26.5, 27.0, 29.0, 30.0, 31.0, 33.0, 35.0, 37.0, 39.25, 41.0, 45.0] # mA

#use 5mA for receive consumption in mA. This can be achieved by SX126X LoRa chip
RX = 5
RX_boosted_LDRO_mode=10.1                                       # mA, max value
RX_boosted_DCDC_mode=5.3                                        # mA, considered energy efficient value
V = 3.3                                                         # voltage 

MCU_on = 9                                                      # mA
MCU_sleep = .005                                                # mA

### LR-FHSS
##### in semtech an 1200.64 v1.3 SX1626 for lr-fhss, I assume for 30dBm tx power
#130 mA Tx current, 10 mA Rx current, 20 μA sleep current
### October 25 => no RX, ETSI = 14 dBm => assume the same as for LoRa-CSS


### LoRa-CSS
#only for BW125/BW500, in nAh for SF7/SF12
#based on SX1262 Semtech's AN on CAD performance 
    # AN1200.48
    # Rev 2.1
    # November 2019
cad_consumption = {
    "SF7":{
        "BW125":{
            "1S":1.73,
            "2S":2.84,
            "4S":5.03,
            "8S":9.41,
            "16S":18.16,
        },
        "BW500":{
            "1S":0.502,
            "2S":0.81,
            "4S":1.43,
            "8S":2.62,
            "16S":4.97,
        },
    },
    "SF9":{
        "BW125":{
            "1S":7.33,
            "2S":11.70,
            "4S":20.44,
            "8S":37.94,
            "16S":72.91,
        },
        "BW500":{
            "1S":2.06,#!!!20.6,
            "2S":3.27,
            "4S":5.69,
            "8S":10.38,
            "16S":20.05,
        },
    },
    "SF12":{
        "BW125":{
            "1S":64.59,
            "2S":99.57,
            "4S":169.54,
            "8S":309.50,
            "16S":589.39,
        },
        "BW500":{
            "1S":16.15,
            "2S":24.89,
            "4S":42.39,
            "8S":77.38,
            "16S":147.35,
        },        
    },
}


########################## SENSITIVITIES AS ANNOUNCED BY SEMTECH

# this is an array with values for sensitivity
# see SX128X Semtech doc
# BW in 203, 406, 812, 1625 kHz
sf5_2dot4GHz = np.array([5,-109.0,-107.0,-105.0,-99.0])
sf6_2dot4GHz = np.array([6,-111.0,-110.0,-118.0,-103.0])
sf7_2dot4GHz = np.array([7,-115.0,-113.0,-112.0,-106.0])
sf8_2dot4GHz = np.array([8,-118.0,-116.0,-115.0,-109.0])
sf9_2dot4GHz = np.array([9,-121.0,-119.0,-117.0,-111.0])
sf10_2dot4GHz = np.array([10,-124.0,-122.0,-120.0,-114.0])
sf11_2dot4GHz = np.array([11,-127.0,-125.0,-123.0,-117.0])
sf12_2dot4GHz = np.array([12,-130.0,-128.0,-126.0,-120.0])

#taken from spec
# this is an array with measured values for sensitivity
# see Table 1 in Bor, M., Roedig, U., Voigt, T., Alonso, J. M. (2016). Do LoRa low-power wide-area networks scale? MSWiM 2016, 59–67. https://doi.org/10.1145/2988287.2989163
# BW in 125, 250, 500 kHz
sf6_subGHz = np.array([6,-118.0,-115.0,-111.0])
sf7_subGHz = np.array([7,-126.5,-124.25,-120.75])
sf8_subGHz = np.array([8,-127.25,-126.75,-124.0])
sf9_subGHz = np.array([9,-131.25,-128.25,-127.5])
sf10_subGHz = np.array([10,-132.75,-130.25,-128.75])
sf11_subGHz = np.array([11,-134.5,-132.75,-128.75])
sf12_subGHz = np.array([12,-133.25,-132.25,-132.25])

sensi_2dot4GHz = np.array([sf5_2dot4GHz,sf6_2dot4GHz,sf7_2dot4GHz,sf8_2dot4GHz,sf9_2dot4GHz,sf10_2dot4GHz,sf11_2dot4GHz,sf12_2dot4GHz])
sensi_subGHz = np.array([sf6_subGHz,sf7_subGHz,sf8_subGHz,sf9_subGHz,sf10_subGHz,sf11_subGHz,sf12_subGHz])

# GW will have boosted rx 
# see SX1261-2_V1.1.pdf p19
sensi_GW_boosted = {
    125:{7:-124,9:-129,12:-137}
}

# determine impact of interference on sensitivity, based on an interpolation of observations in:
# doi={10.1109/PIMRC.2018.8581011} fig4 - right: when SIR increases, the gap sensitivity/snr decreases
# Elshabrawy, Tallal, and Joerg Robert. "Analysis of BER and coverage performance of LoRa modulation under same spreading factor interference." 2018 IEEE 29th Annual International Symposium on Personal, Indoor and Mobile Radio Communications (PIMRC). IEEE, 2018.
def interf_impact_sensi(sir):
    b = 30.610479032143783
    c = 0.23980237740207408
    e = -38.26952794647392

    SNR_default_sensi_12_125 = -20.55

    if(sir>0):
        return(min(0,SNR_default_sensi_12_125 - (b*sir**-c+e)))
    return(-1000)

        

##########################  NOISE CHARACTERISTICS 
noise_mu_dB, noise_sigma_dB = 3, 3                                      # mean and standard deviation, dB
noise_mu, noise_sigma = 10**(.1*noise_mu_dB), 10**(.1*noise_sigma_dB)   # mean and standard deviation, mW

noise_floor_ST = 10*np.log10(125000)-174-6 # -129.03089986991944        # typical noise floor and figure used for sensitivities by Semtech

########################## SHADOWING CHARACTERISTICS
obstruction_dB_GW = .4 # mu/sigma of normally distributed loss due to device's local obstruction (dB)
obstruction_dB = .5
obstruction_dB_CAD = .5

loss_per_building_GW = .3 # for each building on the path, the signal loses this amount of dBm
loss_per_building = .4
loss_per_building_CAD = .4


## MIOTY
Mioty_symbol_rate = 2380.371 # symbols per second
Mioty_split_duration = 36 # symbols per split


## Channel definitions
Channels = {
  "LoRa-CSS":{
        #taken in 2024 from https://github.com/TheThingsNetwork/lorawan-frequency-plans/blob/master/EU_863_870.yml
        #TODO: gen for other bw (250, 500)
        "Channel_list":[
            (868100000,125000),
            (868300000,125000),
            (868500000,125000),
            (867100000,125000),
            (867300000,125000),
            (867500000,125000),
            (867700000,125000),
            (867900000,125000)
        ]
  },
  "LR-FHSS":{
        # AN1200.64
        # https://arxiv.org/html/2312.13981v1
        # The system bandwidth supports 280 channels, where each channel is 488 Hz. Channels k, k+8, k+16, …, k+272 are in group k where 1≤k≤8. To transmit a packet, a node randomly selects a group and hops only among channels belonging to the same group. Fig. 4 shows the medium usage of a synthesized trace containing 400 DR8 packets with payload size randomly distributed between 8 and 16 bytes. It can be seen that the medium is fairly busy in this trace; still, our receiver was able to decode over 90% of the packets.
        #TODO: adjust with correct channels
        "Channel_list":[
            (868100000,136700),
            (868300000,136700),
            (868500000,136700),
            (867100000,136700),
            (867300000,136700),
            (867500000,136700),
            (867700000,136700),
            (867900000,136700)
        ],
        "subchan_bw":488,
        "subchan_inter_bw":0,


  },
  "Mioty":{
        # 24/25 +-5
        #TODO: add downlink channels
        "Channel_list":[
            (868080000,100000), #EU1 up
            (868180000,100000), #EU0 up
            # (869475000,100000), #EU1 down
            # (869575000,100000), #EU0 down
            # (866911000,750000), #EU2
            # (867625000,750000), #EU2
        ],
        "subchan_bw":2380.371,
        "subchan_inter_bw":0,

  }
}



################################
##############
# Local Constants      # TODO: re-dispatch & make documentation
##############


#packet type
dataPacketType=1
rtsPacketType=2

#node's states to control the CANL22 mechanism
schedule_tx=0
want_transmit=1

CANL_listen1=10
CANL_NAV_state=11
CANL_send_RTS=12
CANL_listen2=13
CANL_send_DATA=14
CANL_CAD=15

#node's states to control the ideal mechanism
Ideal_send_DATA=16

#node type, you can add others and customize the behavior on configuration
endDeviceType=1
relayDeviceType=2

#packet transmit interval distribution type
expoDistribType=1
uniformDistribType=2
perioDistribType=3

