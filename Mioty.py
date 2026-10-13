# -*- coding: utf-8 -*-
######## The Mioty components for the LPWAN-Sim Simulator ####################################
######## https://github.com/Guillaumegaillard/LPWAN-Sim ######################################
######## 2026-05-15 ##########################################################################
######## License MIT: ########################################################################
######## https://github.com/Guillaumegaillard/LPWAN-Sim/blob/main/LICENSE ####################
######## Inspired from LoRaSim https://mcbor.github.io/lorasim/ ##############################
# and from  https://github.com/disk91/lpwan-collision-simulation/blob/main/src/MiotyModel.java


import numpy as np
# rng = np.random.default_rng()

import Generic_Elements
import constants


#
# the Mioty node (device) object definition
#
class Mioty_Node(Generic_Elements.Node): 
    # a Mioty node has a symbol_rate and a split (fragment) duration
    def __init__(self, simu, net_id, node_id, nodex, nodey, node_cluster, node_profile, nodeType, period, distrib, packetlen,
        symbol_rate, split_duration):
        # inherit from Generic_Elements
        super().__init__(simu, net_id, node_id, nodex, nodey, node_cluster, node_profile, nodeType, period, distrib, packetlen)

        self.modulation = "Mioty"

        self.transmitted = 0

        self.symbol_rate = symbol_rate # symb/s
        self.split_duration = split_duration # in symbs

        # Mioty's packets are telegrams
        self.packet = Mioty_Telegram(simu, self.net_id, self.node_id, self.packetlen, self.dist_GWs, self.profile, symbol_rate, split_duration)


    # currently, a Mioty device does not receive, so this func is not used
    def get_rx_sensitivity(self, properties_of_packet_being_received, receiver_type="DEVICE"):
        return Mioty_get_sensitivity(receiver_type=receiver_type, GW_sensitivity_gain = 0)
        # if dev: what is my sensi for sigs from neighs having such properties?

    # self.ch, the channel id defined in Generic_Elements, is tech dependent. Mioty's self.packet.subchan too. 
    # => get_chan() converts it to a bw and a center freq 
    def get_chan(self):
        main_chan_center, main_chan_bw = constants.Channels[self.modulation]["Channel_list"][self.ch]

        subchan_id = self.packet.subchan

        sub_bw = constants.Channels[self.modulation]["subchan_bw"]

        sub_center = main_chan_center - main_chan_bw/2 + sub_bw*(subchan_id+1/2)

        return(sub_center,sub_bw)

    # time on air depends on tech
    def get_ongoing_duration(self):

        return(self.split_duration*1000/self.symbol_rate)
        # return(self.packet.splits[self.packet.index_transmission].duration)

    # indexation of fragments is tech-dependent 
    def get_ongoing_start(self):
        return(self.packet.splits[self.packet.index_transmission].split_start)

    # Hyp: all signals in freq and power collision will impact a Mioty split, whatever the modulation is        
    def modulationCollision(self,params1,params2):
        return True

    # Not yet defined a Mioty modulation (all Mioty signals impact equally)
    def getModulationParam(self):
        return({})

    # Main communication loop
    def communicate(self): 

        #Main loop
        while True:

            # Currently, only ALOHA is defined for Mioty: no Carrier Sense, transmit when new frame arrives
            if self.simu.networks[self.net_id]["profiles"][self.profile]["protocol"] in ["ALOHA"]:
                # Schedule next tx
                #########################################################################
                yield self.simu.env.process(self.generate_new_frames())

                #########################################################################

                self.want_transmit_time=self.simu.env.now
                
                # I am a legitimate node
                if self.cluster!=-2:
                    self.simu.nets[self.net_id].n_transmit += 1
                    self.simu.n_transmit += 1
                    if self.simu.nets[self.net_id].n_transmit > 1:
                        net_current_inter_transmit_time = self.simu.env.now - self.simu.nets[self.net_id].last_transmit_time
                        self.simu.nets[self.net_id].inter_transmit_time += net_current_inter_transmit_time
                    if self.simu.n_transmit > 1:
                        simu_current_inter_transmit_time = self.simu.env.now - self.simu.last_transmit_time
                        self.simu.inter_transmit_time += simu_current_inter_transmit_time


                self.simu.last_transmit_time = self.simu.env.now                
                self.simu.nets[self.net_id].last_transmit_time = self.simu.env.now                
                
                # I was asleep? wake up then
                if self.simu.env.now - self.radio_on_last > self.radio_wake_up_time:
                    yield self.simu.env.timeout(self.radio_wake_up_time)
                    self.radio_wake_up_number +=1

                self.n_data_sent = self.n_data_sent + 1
                self.n_payload_sent += self.packet.dataPayloadSize


                if self.cluster!=-2:
                    self.simu.nets[self.net_id].nrSent += 1

                self.latency = self.latency + (self.simu.env.now-self.queued_time)


            ################################
            # splits' transmission
            ################################
                # patterns have been (re)defined in setDataPayloadSize (called in generate_new_frames)
                for split_id in range(len(self.packet.freq_pattern)):

                    self.packet.index_transmission = split_id
                    self.packet.subchan = self.packet.freq_pattern[split_id]
                    self.packet.splits[split_id].split_start = self.simu.env.now

                    # compute the impact of this split on the simu
                    self.simu.check_collision(self.pid)

                    # transmit
                    self.simu.insert_signal(self.pid) 
                    self.simu.add_channel_log((self.simu.env.now,self.split_duration*1000/self.symbol_rate,self.pid,self.cycle,int(self.ch),int(self.packet.subchan)))
                    yield self.simu.env.timeout(self.split_duration*1000/self.symbol_rate)
                    self.simu.remove_signal(self.pid) 

                    self.packet.splits[split_id].transmitted = 1
                    self.packet.stop_interfering() 
                    
                    if split_id != len(self.packet.freq_pattern)-1:
                        self.packet.repropagate() 
                        yield self.simu.env.timeout(self.packet.time_pattern[split_id]) # only wait between bursts/splits/fragments

                
                self.radio_on_last = self.simu.env.now  # I wait wakeup_time before going to sleep

                self.packet.try_decode() # compute receptions on GWs

                ################# build telegram's stats
                packet_totally_lost = True
                for f in self.packet.splits:
                    if f.transmitted == 1:
                        if self.cluster!=-2:
                            self.simu.nets[self.net_id].nrFragmentsSent += 1
                        self.n_fragment_sent += 1
                        if not f.lost:
                            packet_totally_lost = False
                            if f.collided: # on all GWs
                                if self.cluster!=-2:
                                    self.simu.nets[self.net_id].nrFragmentCollisions += 1 # globally a split collision implying e.g. 8 splits are considered 8 collisions
                                self.n_fragment_collided += 1
                        else:
                            if self.cluster!=-2:
                                self.simu.nets[self.net_id].nrFragmentLosses += 1
                            self.n_fragment_lost += 1
                if packet_totally_lost:
                    if self.cluster!=-2:
                        self.simu.nets[self.net_id].nrLost += 1
                    self.n_lost += 1

                if self.packet.success == 1: # received successfully on at least one GW
                    if self.cluster!=-2:
                        self.simu.nets[self.net_id].nrReceived += 1
                    # current_latency=self.simu.env.now-self.want_transmit_time
                    current_latency=self.simu.env.now-self.queued_time
                    self.min_success_latency = min(self.min_success_latency, current_latency)
                    self.success_latency = self.success_latency + current_latency
                    self.n_data_success+=1
                    self.n_payload_success += self.packet.dataPayloadSize



                # #end of transmission procedure
                
                # reset the packet
                self.packet.collided = 0
                self.packet.collided_at_GWs = [0 for _ in range(self.myGWs)]

                self.packet.processed = 0
                self.packet.lost = False
                self.packet.success = 0

                self.packet.repropagate()     
                
                #### CHANNEL UPDATE
                # this following alternates channels (only two channels), instead of pseudo-random process in Mioty spec 
                self.last_tx_ch_seq_id = self.channel_sequence.index(self.ch)
                self.transmitted_black_list.append(self.ch)
                self.update_available_channels()


                if self.update_channel(max_changes = len(constants.Channels[self.modulation]["Channel_list"])):
                    yield self.simu.env.timeout(self.radio_config_time)
                    self.radio_config_number +=1

            ################################
            ################################

            ##################################
            ######## Checking end of Simu ####
            ##################################

            if self.simu.check_all_networks_finished():
                self.simu.endSim = self.simu.env.now
                ## ! ##
                return 

    # compute impact of this split to the rest of the signal in the simu, and vice-versa
    def check_collision(self, splitsOnAir):
        split = self.packet.splits[self.packet.index_transmission]

        # (kinda obsolete) collision indicator (collided_at_some_GW)
        col = 0

        # check for all the GWs in the simu if this split is perceived or not, store it in an array 
        heard_at_gateways = self.simu.check_heard_at_GWs(self.pid)

        # report this info on my GWs
        for gw_id in range(self.myGWs): # for each of my gws
            if not heard_at_gateways[self.myGWs_ids][gw_id]:
                split.lost_at_GWs[gw_id] = 1
            else:
                split.lost_at_GWs[gw_id] = 0

        if not heard_at_gateways[self.myGWs_ids].any(): # none of my gws receive it, too far
            split.lost = True
        else:
            split.lost = False
            
        # if somebody else is transmitting
        if len(splitsOnAir)>0:
            # count detections and signal mWs at GWs
            ## count me
            in_ears_GWs=np.where(heard_at_gateways,1,0) # packet is not yet in packetsOnAir but will be part of the mess
            this_interf_mw_GWs = 10**(.1*self.packet.rss_GWs)

            # count all others on same channel for all the gws of the simu
            for other in splitsOnAir:
                if self.simu.frequencyCollision(self.pid, other) and self.simu.modulationCollision(self.pid, other): #HYP: any signal (Mioty or FHSS on same subchan, CSS on same CH) interferes; TODO: detail more  
                    other_heard_at_gateways = self.simu.check_heard_at_GWs(other) 
                    
                    in_ears_GWs+=np.where(other_heard_at_gateways,1,0)

                    other_packet_signal_at_gws_mw = 10**(.1*self.simu.get_rss_GWs(other))

                    # add interference to current packet
                    self.packet.interf_mw_GWs+=other_packet_signal_at_gws_mw
                    # add interference to other packet
                    self.simu.add_interf_GWs(other,this_interf_mw_GWs)

            ################################
            #### EVALUATE COLLISIONS AND CAPTURES 
            for other in splitsOnAir:
                other_heard_at_gateways = self.simu.check_heard_at_GWs(other) 

                for gw_id in range(len(heard_at_gateways)): # for each gw
                    if heard_at_gateways[gw_id]:
                        if other_heard_at_gateways[gw_id]:
                            # NB: splits channel collision verif is included in frequencyCollision()
                            if self.simu.frequencyCollision(self.pid, other):

                                # check who collides in the power domain, considering interference (and preamble lock if other has preamble)
                                c = self.simu.powerCollision(self.pid, other, in_ears=in_ears_GWs[gw_id], ptype="gw", loc_id=gw_id)
                                # either this one, the other one, or both

                                receiving_split = False
                                # mark all the collided packets
                                if self.pid in c: # split was collided by others 

                                    col = 1
                                    if gw_id in self.myGWs_ids:
                                        gw_local_id = self.myGWs_ids.index(gw_id)
                                        split.collided_at_GWs[gw_local_id] = 1
                                        self.n_captured +=1
                                        self.simu.nets[self.net_id].nrCaptures+=1
                                else:
                                    receiving_split = True

                                # lock receptor on split (Hyp: always true, there is no preamble)
                                if receiving_split:
                                    self.packet.receiving_GWs[gw_id]=True
                                    self.packet.receiving_start_GWs[gw_id]=split.split_start

                                # mark older packets as collided if needed
                                if other in c:
                                    self.simu.mark_collision(other, gw_id)
    
        # check impact on listening devices (LoRa-CSS devices)
        self.simu.check_collisions_local(self.pid, collided_at_some_GW = col)




#
# Class for telegrams (Mioty packets) (associated with a node)
# All Mioty specific parameters, patterns, etc.
#
class Mioty_Telegram(Generic_Elements.Packet): 
    ## inherits packets attributes, adds Mioty symbol_rate and split_duration (Mioty options in the standards)
    def __init__(self, simu, net_id, node_id, plen, distance_GWs, node_profile,
                symbol_rate, split_duration): #
        super().__init__(simu, net_id, node_id, plen, distance_GWs, node_profile)

        self.uses_sub_channels = True
        self.index_transmission = -1
        self.subchan = -1
        self.success = 0

        self.symbol_rate = symbol_rate
        self.split_duration = split_duration

        self.time_pattern = []
        self.freq_pattern = []
        self.splits = []

    
    # give tx config info to receiver        
    def get_tx_config(self):
        return({}) # TODO: describe split subchan... But not in use for now

    # overrides Generic_Element setDataPayloadSize()
    # calls to split sequence preparation
    def setDataPayloadSize(self):
        super().setDataPayloadSize()
        self.prepare_split_sequence()

    # based on data size, prepare sequence
    def prepare_split_sequence(self):

        nb_splits_core = 24 # core frame has 24 chunks
        if self.dataPayloadSize < 20: # core frame contains up to 20B data
            # phyTelegram_size = 186 #bits
            nb_splits_ext = 0
        else:
            nb_splits_ext = self.dataPayloadSize - 20

        # node transmitters may offset +-5 channels
        channel_offset = self.simu.rng.integers(-5,6) + 5 # this last +5 allows subchan ids to be in range(0,36) => get_chan() func currently computes subchan's center freqs starting from chan's bottom (lowest freq)


        pattern_id = self.simu.rng.integers(8) # 8 patterns available down here

        # times
        time_pattern = []

        #core frame, intervals after each burst
        for j in range(nb_splits_core):
            if ( j % 3 == 0 ):
                time_pattern.append(330)
            elif ( j % 3 == 1 ):
                time_pattern.append(387)
            else:
                # if ( j != 23 or nb_splits_ext>0): 
                if ( j != 23): 
                    time_pattern.append(timePatterns[pattern_id][j//3])

        #extension frame
        for j in range(nb_splits_ext): # intervals before each ext split
            inter_center_symb_interval = 337 + self.simu.rng.integers(128) # 337 (66 for g3)
            time_pattern.append( (inter_center_symb_interval - self.split_duration) *1000/self.symbol_rate) #ms (1 burst is 36 symb)

        # subchans
        freq_pattern = []
        #core frame
        for j in range(nb_splits_core):
            freq_pattern.append(frequencyPatterns[pattern_id][j]+channel_offset)

        #extension frame
        for j in range(nb_splits_ext):
            freq_pattern.append(self.simu.rng.integers(25)+channel_offset) 

        # 1 burst = ~15.12 ms = 36 symb at 2380.371/s
        # 300symb = ~126ms

        self.index_transmission = -1

        self.time_pattern = time_pattern
        self.freq_pattern = freq_pattern

        # create split list        
        self.splits = []
        for split_id in range(len(freq_pattern)):
            self.splits.append(Mioty_split(freq_pattern[split_id], self.net_id, self.node_id, self.myGWs))



    # remove my interference when I leave
    def stop_interfering(self):
        self.simu.stop_interfering(self.net_id, self.node_id)

    # determine receptions, losses, collisions, of all splits on all gws of this net
    def try_decode(self):

        for f in self.splits:
            if not f.lost: #has been heard somewhere
                f.collided = True
                # for gw_id in range(len(self.dist_GWs)):
                for gw_id in range(self.myGWs):
                    if f.lost_at_GWs[gw_id] == 0:
                        if f.collided_at_GWs[gw_id] == 0:
                            f.collided = False
                            break


        for gw_id in range(self.myGWs):
            f_success = 0 
            for f in self.splits:
                if f.lost_at_GWs[gw_id] == 0:
                    if f.collided_at_GWs[gw_id] == 0:
                        if f.transmitted==1:
                            f_success +=1
            

            success = 1 if  (f_success/len(self.splits) >= 50/100) else 0 # TODO: this 50% is claimed by Mioty...
            if success == 1: # telegram received on one GW!
                self.success = 1
                return True
        
        # telegram not received anywhere
        return False



# Mioty splits (fragments)
class Mioty_split():
    def __init__(self, subchannel, net_id, node_id, myGWs):
        self.node_id = node_id
        self.net_id = net_id

        self.success = 0
        self.transmitted = 0
        self.subchannel = subchannel

        self.collided = False
        self.lost = False

        self.collided_at_GWs = [0 for _ in range(myGWs)]
        self.lost_at_GWs = [0 for _ in range(myGWs)]



#
# the Mioty GW object definition
#
class Mioty_Gateway(Generic_Elements.Gateway): 
    def __init__(self, simu, net_id, gw_id, gw_x, gw_y):
        super().__init__(simu, net_id, gw_id, gw_x, gw_y)

        # self.modulation = "Mioty"

    def get_rx_sensitivity(self, properties_of_packet_being_received):
        return Mioty_get_sensitivity(receiver_type="GW", GW_sensitivity_gain = self.simu.networks[self.net_id]["GW_sensitivity_gain"])


#TODO: adjust sensitivities
def Mioty_get_sensitivity(receiver_type="GW", GW_sensitivity_gain = 0):
    # return 10
    if receiver_type=="DEVICE":
        return(100000) # a Mioty device does not receive in current state of this implem TODO: downlink and p2p 
        return(constants.sensi_subGHz[6, 1]) 

    # a MIOTY GW receives anything with the sensitivity of a lora gw for SF12 BW125 frames
    return(-GW_sensitivity_gain+constants.sensi_GW_boosted[125][12]) # -137 - 1.5



# from  https://github.com/disk91/lpwan-collision-simulation/blob/main/src/MiotyModel.java
frequencyPatterns =  [
    [ 5, 21, 13,  6, 22, 14,  1, 17,  9,  0, 16,  8,  7, 23, 15,  4, 20, 12,  3, 19, 11,  2, 18, 10],
    [ 4, 20, 12,  1, 17,  9,  0, 16,  8,  6, 22, 14,  7, 23, 15,  2, 18, 10,  5, 21, 13,  3, 19, 11],
    [ 4, 20, 12,  3, 19, 11,  6, 22, 14,  7, 23, 15,  0, 16,  8,  5, 21, 13,  2, 18, 10,  1, 17,  9],
    [ 6, 22, 14,  2, 18, 10,  7, 23, 15,  0, 16,  8,  1, 17,  9,  4, 20, 12,  5, 21, 13,  3, 19, 11],
    [ 7, 23, 15,  4, 20, 12,  3, 19, 11,  2, 18, 10,  6, 22, 14,  0, 16,  8,  1, 17,  9,  5, 21, 13],
    [ 3, 19, 11,  6, 22, 14,  2, 18, 10,  0, 16,  8,  7, 23, 15,  1, 17,  9,  4, 20, 12,  5, 21, 13],
    [ 3, 19, 11,  1, 17,  9,  5, 21, 13,  7, 23, 15,  0, 16,  8,  2, 18, 10,  6, 22, 14,  4, 20, 12],
    [ 0, 16,  8,  6, 22, 14,  3, 19, 11,  2, 18, 10,  4, 20, 12,  7, 23, 15,  5, 21, 13,  1, 17,  9]
]

timePatterns = [
    [388, 354, 356, 432, 352, 467, 620],
    [435, 409, 398, 370, 361, 472, 522],
    [356, 439, 413, 352, 485, 397, 444],
    [352, 382, 381, 365, 595, 604, 352],
    [380, 634, 360, 393, 352, 373, 490],
    [364, 375, 474, 355, 478, 464, 513],
    [472, 546, 501, 356, 359, 359, 364],
    [391, 468, 512, 543, 354, 391, 368]
]