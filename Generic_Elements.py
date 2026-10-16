# -*- coding: utf-8 -*-
######## Generic components of the LPWAN-Sim Simulator #######################################
######## https://github.com/Guillaumegaillard/LPWAN-Sim ######################################
######## 2026-05-14 ##########################################################################
######## License MIT: ########################################################################
######## https://github.com/Guillaumegaillard/LPWAN-Sim/blob/main/LICENSE ####################
######## Inspired from LoRaSim https://mcbor.github.io/lorasim/ ##############################


import numpy as np
# rng = np.random.default_rng()
import sys

import constants

#
# this class creates a packet/frame (associated with a node)
# it also initiates many parameters related to transmission
# TODO: could maybe be integrated into Node() directly, or partially
class Packet():
    ## a packet belong to a simu, has net and node ID, default packet length, distances to the GWs, and a node profile
    def __init__(self, simu, net_id, node_id, plen, distance_GWs, node_profile):

        self.net_id = net_id
        self.simu = simu # TODO: could inherit necessary items only, instead of full simu? 
        self.node_id = node_id
        self.profile = node_profile

        self.dist_GWs=distance_GWs

        # GWs of my net
        self.myGWs = self.simu.networks[self.net_id]["nrGWs"]
        self.myGWs_ids = simu.networks[net_id]["gateways"]["gw_ids"]
        
        # TODO: should be a Node attribute            
        self.txpow = self.simu.networks[self.net_id]["Ptx"]

        # initial values, will be modified after
        self.pl = plen
        self.rss_GWs = self.txpow # *np.ones(distance_GWs.shape)
        
        # fast rayleigh multipath fading
        self.rayleigh_dB = np.zeros((self.dist_GWs.shape)) # to GWs
        self.rayleigh_dB_arr = np.zeros((self.simu.nrNodes))

        ## fast local noise (at receiver, this packet is impaired by this noise value)
        self.noise_dB = np.zeros((self.dist_GWs.shape)) # to GWs
        self.noise_dB_arr = np.zeros((self.simu.nrNodes)) # to all neighs


        #init interf caused and receiver lock variable for rx in devs and gws
        # all nodes, all gws suffer 0 interference from this packet
        self.interf_mw = np.zeros((self.simu.nrNodes))
        self.interf_mw_GWs = np.zeros((self.dist_GWs.shape))

        # there is no node, no gw, that has a focus (rx lock) on this packet (could only be in LoRa-CSS, actually)
        self.receiving = np.full((self.simu.nrNodes), False)
        self.receiving_GWs = np.full((self.dist_GWs.shape), False) 

        # RX lock has started nowhere yet
        self.receiving_start = np.zeros((self.simu.nrNodes))
        self.receiving_start_GWs = np.zeros((self.dist_GWs.shape))

        # type of packet, kind of obsolete variable
        self.ptype = constants.dataPacketType
        
        #self.data_len keeps the payload length of a data packet
        #self.pl will be used in presence of RTS packets (cf. CANL) to discriminate the current packet length which can either be 
        #data_len or 5 (the size of an RTS packet)
        self.data_len=plen
        self.dataPayloadSize=self.data_len

        # denote if packet is collided (init of vars)
        self.collided = 0
        self.collided_at_GWs = [0 for _ in range(self.myGWs)]
        self.lost_at_GWs = [0 for _ in range(self.myGWs)]
        self.processed = 0 # not in use in the current version (2026-04-08)

    #
    # give payload size according to the specified probability distribution
    # completed (overridden) in tech-specific modules
    def setDataPayloadSize(self):
        if self.simu.networks[self.net_id]["variablePayloadSize"]:
            if self.simu.networks[self.net_id]["normalPayloadSize"]:
                self.dataPayloadSize=self.simu.rng.normal(self.simu.networks[self.net_id]["normaldist_mean_payload_size"], self.simu.networks[self.net_id]["normaldist_sigma_payload_size"],1).astype('int').clip(self.simu.networks[self.net_id]["dist_min_payload_size"],self.simu.networks[self.net_id]["dist_max_payload_size"])[0]
            else: # uniform
                self.dataPayloadSize=self.simu.rng.integers(self.simu.networks[self.net_id]["dist_min_payload_size"], self.simu.networks[self.net_id]["dist_max_payload_size"]+1)


    #
    ## change packet type and size accordingly
    def setPacketType(self,ptype):
        self.ptype = ptype
        
        if ptype == constants.rtsPacketType:
            self.pl=5
        else:
            self.pl=self.dataPayloadSize # self.data_len

    # rx noise is updated after each transmission (if "gaussian_noise") 
    # warning: "gaussian_noise" is clipped to a (min, max) window (not a full "gaussian_noise")
    def update_noise(self):
        ## fast local noise
        if self.simu.radio_params["gaussian_noise"]:
            self.noise_dB = np.clip(self.simu.rng.normal(constants.noise_mu_dB,constants.noise_sigma_dB,self.dist_GWs.shape),0,2*constants.noise_mu_dB)
            self.noise_dB_arr = np.clip(self.simu.rng.normal(constants.noise_mu_dB,constants.noise_sigma_dB,self.noise_dB_arr.shape),0,2*constants.noise_mu_dB)

    # multipath/rayleigh/fast is updated after each transmission
    def update_multi_path_fading(self):

        self.rx_array += self.rayleigh_dB_arr
        self.rss_GWs += self.rayleigh_dB
        
        if self.simu.radio_params["rayleigh_fading"]: # gain is exponential (normally, of mean 1, i.e. 0 dB)
            # older version
            # rayleigh_dB = rng.rayleigh(scale=np.sqrt(2 / np.pi)*rayleigh_mean_dB,size=self.dist_GWs.shape) - rayleigh_mean_dB
            # rayleigh_dB_arr = rng.rayleigh(scale=np.sqrt(2 / np.pi)*rayleigh_mean_dB,size=rayleigh_dB_arr.shape) - rayleigh_mean_dB
            self.rayleigh_dB = -10*np.log10(self.simu.rng.exponential(scale=10**(.1*self.simu.radio_params["rayleigh_mean_dB"]), size=self.dist_GWs.shape))
            self.rayleigh_dB_arr = -10*np.log10(self.simu.rng.exponential(scale=10**(.1*self.simu.radio_params["rayleigh_mean_dB"]), size=self.rayleigh_dB_arr.shape))

        self.rx_array -= self.rayleigh_dB_arr
        self.rss_GWs -= self.rayleigh_dB

    # set/update a random rx array to all neighbors
    def repropagate(self, first_time=False):  

        if first_time: # prepare coherent things only the first propagate

            # local shadowing
            self.local_obstruction_dB = np.zeros((self.dist_GWs.shape)) # to GWs
            self.local_obstruction_dB_arr = np.zeros((self.simu.nrNodes))

            if self.simu.radio_params["locally_obstructed"]:
                self.local_obstruction_dB = np.clip(self.simu.rng.normal(constants.obstruction_dB_GW,constants.obstruction_dB_GW,self.dist_GWs.shape),0,2*constants.obstruction_dB_GW)
                self.local_obstruction_dB_arr = np.clip(self.simu.rng.normal(constants.obstruction_dB,constants.obstruction_dB,self.local_obstruction_dB_arr.shape),0,2*constants.obstruction_dB)

            # link shadowing accounting for buildings
            self.buildings_atten_dB = np.zeros((self.dist_GWs.shape)) # to GWs
            self.buildings_atten_dB_arr = np.zeros((self.simu.nrNodes))

            if self.simu.radio_params["buildings_attenuated"]:
                self.buildings_atten_dB = self.simu.radio_params["buildings_km"]*self.dist_GWs/1000*constants.loss_per_building_GW
                self.buildings_atten_dB_arr = self.simu.radio_params["buildings_km"]*self.simu.get_dist_vector(self.net_id,self.node_id)/1000*constants.loss_per_building      


            # matrix of every received signal strength accounting for path loss, coherent shadowing and fast mp fading 
            self.rx_array=np.clip(-1000,self.txpow,self.txpow + constants.GL - constants.Lpld0 - 10*self.gamma_array*np.log10(self.simu.get_dist_vector(self.net_id,self.node_id)/constants.d0) - self.rayleigh_dB_arr - self.local_obstruction_dB_arr - self.buildings_atten_dB_arr)
            
            # to GWs
            self.rss_GWs=np.minimum(self.txpow,self.txpow + constants.GL_GW - constants.Lpld0 - 10*self.simu.networks[self.net_id]["gamma_GW"]*np.log10(self.dist_GWs/constants.d0) - self.rayleigh_dB - self.local_obstruction_dB - self.buildings_atten_dB)

        # this is done at each repropagate (end of tx) => in future, could be taken away (multipath fading alongside correlated noise) 
        self.update_multi_path_fading()
        self.update_noise() # only effective if gaussian_noise parameter is True


        # reset interference caused by packet, and who is locked on it
        self.interf_mw = np.zeros((self.simu.nrNodes)) # interf suff by this packet on each listening dev
        self.interf_mw_GWs = np.zeros((self.dist_GWs.shape)) # # interf suff by this packet on each gw

        self.receiving = np.full((self.simu.nrNodes), False)
        self.receiving_GWs = np.full((self.dist_GWs.shape), False) 

        

#
# the GW object definition
#
class Gateway(): 
    def __init__(self, simu, net_id, gw_id, gw_x, gw_y):
        self.simu = simu
        self.net_id = net_id
        self.gw_id = gw_id
        self.pid = (net_id, gw_id) # pair_id

        self.x = gw_x
        self.y = gw_y


#
# the generic node (device) object definition
#
class Node():
    # a node in a simu has an id, coordinates, a type (relay, endDevice...), a cluster, a profile, a traffic period, distribution, and default payload size
    def __init__(self, simu, net_id, node_id, nodex, nodey, node_cluster, node_profile, nodeType, period, distrib, packetlen):
        self.simu = simu
        self.net_id = net_id
        self.node_id = node_id
        self.pid = (net_id, node_id) # pair_id

        self.x = nodex
        self.y = nodey
        self.cluster=node_cluster
        self.profile=node_profile       # node profile

        self.nodeType = nodeType        # relay, enddev... not used currently
        self.period = period            # average generate (send) time period
        self.distrib = distrib          # generate (send) prob distrib
        
        self.packetlen = packetlen

        self.listening = False
        self.uses_sub_channels = False


        # distance to center of topology
        self.dist_to_center = np.sqrt((self.x-simu.networks[net_id]["center_x"])**2+(self.y-simu.networks[net_id]["center_y"])**2)
        
        # distance to all GWs (not only the GWs of network net_id)
        # self.dist_GWs = np.sqrt((self.x-simu.networks[net_id]["gateways"]["xs"])**2+(self.y-simu.networks[net_id]["gateways"]["ys"])**2)
        self.dist_GWs = np.sqrt((self.x-simu.gateways["xs"])**2+(self.y-simu.gateways["ys"])**2)

        # GWs of my net
        self.myGWs = self.simu.networks[self.net_id]["nrGWs"]
        self.myGWs_ids = simu.networks[net_id]["gateways"]["gw_ids"]

        # node has a packet, but will be init at tech specific heir classes (lora node, fhss node, etc.) 
        
        # available channels for channel hopping
        self.avail_chans = [i for i in range(len(constants.Channels[simu.networks[net_id]["modulation"]]["Channel_list"]))]

        # channel is randomly initialized as a sequence per device        
        self.channel_sequence = self.avail_chans[:]
        self.simu.rng.shuffle(self.channel_sequence)

        # self.ch = rng.choice(self.avail_chans)
        self.ch = self.channel_sequence[0]
        self.channel_just_changed = True
        self.last_tx_ch_seq_id = -1 # the id in sequence of the channel that was last transmitted on

        # channels tried (listened to in a Carrier Sense mechanism) and found busy
        # self.failed_chans = []
        self.failed_chans = 0

        # channels previously used for transmission (max size of this list is defined by setting "tx_memory")
        self.transmitted_black_list = []

        # time required for configuring the radio to a given channel
        self.radio_config_time = simu.networks[net_id]["radio_config_time"] # e.g. 5 #ms
        # time required to wake up the radio
        self.radio_wake_up_time = simu.networks[net_id]["radio_wake_up_time"] # e.g. 5 #ms

        # statistics
        self.radio_config_number = 0
        self.radio_wake_up_number = 0

        # last moment radio was on
        self.radio_on_last = -1000000


        ################## NODE VARIABLES ##########################
        self.n_data_sent = 0
        self.n_rts_sent = 0
        self.n_data_success = 0 # successfully transmitted frames
        self.n_payload_gen = 0 # amount of generated bytes
        self.n_payload_sent = 0 # amount of sent bytes
        self.n_payload_success = 0 # amount of received bytes


        self.want_transmit_time=0
        self.queued_time=0

        #### Variables for collision avoidance /listen
        self.MAC_state = constants.schedule_tx # initial state, will change in communicate() function

        ## stats for node during whole simulation                    
        self.n_aborted=0
        self.n_collided=0
        self.n_captured = 0
        self.n_lost=0
        self.n_dropped=0
        self.cycle=0
        self.kamikaze=0

        self.n_fragment_sent = 0
        self.n_fragment_collided = 0
        self.n_fragment_lost = 0

        self.n_CAD=0 # number of cads done 


        ## latencies
        self.latency=0 # of current packet 
        self.success_latency=0 # of all packets, summed and then averaged
        self.min_success_latency=self.period*1000 # init at very large value
        self.attempt_latency = 0 # of all CS attempt, summed and then averaged
        self.min_attempt_latency = self.period*1000 # init at very large value
        self.max_attempt_latency = 0

        ### variables for packet generation and queue
        self.gen_times=[0]
        self.queue = []
        self.next_gen_time=-1#ms

        ### listen
        self.total_listen_time = 0 # for all time in simu

        ### multiple access
        self.total_retry=0 # stat for each dev during whole sim
        self.min_retries=1000 # stat for each dev during whole sim
        self.max_retries=0 # stat for each dev during whole sim

    # overridden in tech specific heirs (because it is tech dependent)
    def get_rx_sensitivity(self, packet):
        return False


    # node function to update the channel to which radio is tuned
    # does not yield any delay here (see main "communicate" loop)
    # return False if no new channel is available
    def update_channel(self,max_changes,limited_changes=False,after_fail=False):
        
        # a fail is a positive carrier sense
        if after_fail:
            self.failed_chans += 1

        if limited_changes and self.failed_chans == max_changes: #all possible channels have been listened
            return(False)

        skip_black_listed = 0
        found_chan = False

        num_chans_for_this_tech = len(constants.Channels[self.modulation]["Channel_list"])

        while (skip_black_listed < num_chans_for_this_tech and not found_chan):
            candidate = self.channel_sequence[(self.last_tx_ch_seq_id+1+self.failed_chans+skip_black_listed)%num_chans_for_this_tech]
            if candidate in self.transmitted_black_list:
                skip_black_listed += 1
            else:
                self.ch = candidate
                found_chan = True

        return(found_chan)


    # node function to update/reset the list of available channels to do carrier sense on at a given moment
    def update_available_channels(self):
        self.failed_chans = 0

        if len(self.transmitted_black_list) > self.simu.networks[self.net_id]["tx_blacklist_memory"]:
            if self.simu.networks[self.net_id]["tx_blacklist_update_strategy"] == "shift":
                self.transmitted_black_list = self.transmitted_black_list[1:]
            elif self.simu.networks[self.net_id]["tx_blacklist_update_strategy"] == "empty_but_last": # just avoid last
                self.transmitted_black_list = [self.transmitted_black_list[-1]]

     
    # function to generate future traffic
    # called when previous tx process finishes, but only gen frames if necessary
    def generate_new_frames(self):

        # while last frame gen was in past  
        while self.simu.env.now>self.next_gen_time or len(self.queue) == 0 :

            # if we enter too many times here, we need to drop
            if len(self.queue) == self.simu.networks[self.net_id]["max_queue_size"]:
                self.n_dropped+=1
            else:
                self.queue.append(0)

            self.queue = [f-1 for f in self.queue]

            # produce next packet
            if self.distrib== "spe100":
                #normally 9 nodes with 100ms delay between each node
                inter_gen_delay=self.cycle*self.period-self.simu.env.now+self.node_id*100
            if self.distrib== "spe500":
                #normally 5 nodes with 500ms delay between each node
                inter_gen_delay=self.cycle*self.period-self.simu.env.now+self.node_id*500            
            if self.distrib== constants.perioDistribType:
                inter_gen_delay=self.period
            if self.distrib==constants.expoDistribType:
                inter_gen_delay = self.simu.rng.exponential(float(self.period))
                # transmit_wait = rng.expovariate(1.0/float(self.period))
            if self.distrib==constants.uniformDistribType:
                inter_gen_delay = self.simu.rng.uniform(max(2000,self.period-5000),self.period+5000)        
            
            # next_gen_time=last_gen_time+inter_gen_delay
            if self.simu.networks[self.net_id]["shuffle_start"] and self.next_gen_time==-1:
                self.next_gen_time+=self.simu.rng.uniform(0,self.period)
            self.next_gen_time+=inter_gen_delay
            self.gen_times.append(self.next_gen_time)

            if self.cluster!=-2: # if it is not an interferer, but a legit node
                self.simu.nets[self.net_id].nrScheduled += 1


        ################ set next packet
        oldest_gen_non_dropped = self.queue.pop(0)
        self.packet.setDataPayloadSize()
        self.packet.setPacketType(constants.dataPacketType)                
        self.cycle = self.cycle + 1
        self.n_payload_gen += self.packet.dataPayloadSize 


        if self.simu.env.now < self.gen_times[oldest_gen_non_dropped]: # oldest gen is in future, lets wait

            transmit_wait=self.gen_times[oldest_gen_non_dropped] - self.simu.env.now
            yield self.simu.env.timeout(transmit_wait)
            self.queued_time = self.simu.env.now
            # self.want_transmit_time = self.simu.env.now

        else: # oldest gen is in past, has been queued
            self.queued_time = self.gen_times[oldest_gen_non_dropped]

        # change state and go attempting a tx
        self.MAC_state=constants.want_transmit