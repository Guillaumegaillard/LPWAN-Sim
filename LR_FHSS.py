# -*- coding: utf-8 -*-
######## The LR-FHSS components for the LPWAN-Sim Simulator ##################################
######## https://github.com/Guillaumegaillard/LPWAN-Sim ######################################
######## 2026-05-18 ##########################################################################
######## License MIT: ########################################################################
######## https://github.com/Guillaumegaillard/LPWAN-Sim/blob/main/LICENSE ####################
######## Inspired from LoRaSim https://mcbor.github.io/lorasim/ ##############################
# and from https://github.com/Xexell/LR-FHSS-sim #############################################

import numpy as np
rng = np.random.default_rng()

import Generic_Elements
import constants


#
# the LR-FHSS node (device) object definition
#
class LR_FHSS_Node(Generic_Elements.Node): 
    # a LR_FHSS node has obw, nb_headers, header_duration, payload_duration, transceiver_wait, and LR-FHSS code
    def __init__(self, simu, net_id, node_id, nodex, nodey, node_cluster, node_profile, nodeType, period, distrib, packetlen
        , obw, nb_headers, header_duration, payload_duration, transceiver_wait, code):
        super().__init__(simu, net_id, node_id, nodex, nodey, node_cluster, node_profile, nodeType, period, distrib, packetlen)

        self.modulation = "LR-FHSS"

        self.transmitted = 0
        self.transceiver_wait = transceiver_wait
        self.obw = obw # amount of subchannels
        self.nb_headers = nb_headers
        self.header_duration = header_duration
        self.payload_duration = payload_duration
        self.code = code

        # translation to a packet
        self.packet = LR_FHSS_Packet(simu, self.net_id, self.node_id, self.packetlen, self.dist_GWs, self.profile, self.obw, self.nb_headers, self.header_duration, self.payload_duration, self.code) 


    # currently, a LR-FHSS device does not receive, so this func is not used
    def get_rx_sensitivity(self, properties_of_packet_being_received, receiver_type="DEVICE"):
        return LR_FHSS_get_sensitivity(receiver_type=receiver_type, GW_sensitivity_gain = 0)
        # if dev: what is my sensi for sigs from neighs in my tech?

    # self.ch, the channel id defined in Generic_Elements, is tech dependent. LR-FHSS's fragment.channel too. 
    # => get_chan() converts it to a bw and a center freq     
    def get_chan(self):
        main_chan_center, main_chan_bw = constants.Channels[self.modulation]["Channel_list"][self.ch]

        subchan_id = self.packet.fragments[self.packet.index_transmission-1].channel

        sub_bw = constants.Channels[self.modulation]["subchan_bw"]

        sub_center = main_chan_center - main_chan_bw/2 + sub_bw*(subchan_id+1/2)

        return(sub_center,sub_bw)

    # time on air depends on tech
    def get_ongoing_duration(self):
        return(self.packet.fragments[self.packet.index_transmission-1].duration)

    # indexation of fragments is tech-dependent 
    def get_ongoing_start(self):
        return(self.packet.fragments[self.packet.index_transmission-1].timestamp)

    # Hyp: all signals in freq and power collision will impact a LR-FHSS fragment, whatever the modulation is        
    def modulationCollision(self,params1,params2):
        return True

    # Not yet defined a FHSS modulation param here (all FHSS signals impact equally the others)
    def getModulationParam(self):
        return({})

    # Main communication loop
    def communicate(self): 

        #Main loop
        while True:

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
            # fragments' transmission
            ################################

                next_fragment = self.packet.next()
                first_payload = 0
                while next_fragment:
                    if first_payload == 0 and next_fragment.type=='payload': #account for the transceiver wait time between the last header and first payload fragment
                        first_payload=1
                        yield self.simu.env.timeout(self.transceiver_wait)
                    next_fragment.timestamp = self.simu.env.now

                    #checks if the fragment is colliding with the signals in transmission now and its rx/loss at GWs
                    self.simu.check_collision(self.pid) 

                    #add the fragment to the list of fragments being transmitted.
                    self.simu.insert_signal(self.pid)

                    #wait the duration (time on air) of the fragment
                    self.simu.add_channel_log((self.simu.env.now,next_fragment.duration,self.pid,self.cycle,int(self.ch), int(next_fragment.channel)))
                    yield self.simu.env.timeout(next_fragment.duration)
                    
                    #removes the fragment from the list.
                    self.simu.remove_signal(self.pid)
                    next_fragment.transmitted = 1

                    ### reset radio conditions
                    self.packet.stop_interfering()
                    #select the next fragment
                    next_fragment = self.packet.next()
                    if next_fragment: # for last fragment, repropagate after decode 
                        self.packet.repropagate()

                
                self.radio_on_last = self.simu.env.now # I will wait wakeup_time before going to sleep

                # check final receptions of whole packet on each of my GWs
                self.packet.try_decode()

                packet_totally_lost = True
                for f in self.packet.fragments:
                    if f.transmitted==1:
                        if self.cluster!=-2:
                            self.simu.nets[self.net_id].nrFragmentsSent += 1
                        self.n_fragment_sent += 1
                        if not f.lost:
                            packet_totally_lost = False
                            if f.collided: # on all GWs
                                if self.cluster!=-2:
                                    self.simu.nets[self.net_id].nrFragmentCollisions += 1 # globally a frag collision implying e.g. 8 fragments are considered 8 collisions
                                self.n_fragment_collided += 1
                        else:
                            if self.cluster!=-2:
                                self.simu.nets[self.net_id].nrFragmentLosses += 1
                            self.n_fragment_lost += 1
                if packet_totally_lost:
                    if self.cluster!=-2:
                        self.simu.nets[self.net_id].nrLost += 1
                    self.n_lost += 1

                if self.packet.success == 1:
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
                
    # compute impact of this fragment to the rest of the signal in the simu, and vice-versa
    def check_collision(self, fragsOnAir):
        fragment = self.packet.fragments[self.packet.index_transmission-1]

        # (kinda obsolete) collision indicator (collided_at_some_GW)
        col = 0

        # check for all the GWs in the simu if this fragment is perceived or not, store it in an array 
        heard_at_gateways = self.simu.check_heard_at_GWs(self.pid)

        # report this info on my GWs
        for gw_id in range(self.myGWs): # for each of my gws
            if not heard_at_gateways[self.myGWs_ids][gw_id]:
                fragment.lost_at_GWs[gw_id] = 1
            else:
                fragment.lost_at_GWs[gw_id] = 0


        if not heard_at_gateways[self.myGWs_ids].any(): # none of my gws receive it, too far
            fragment.lost = True
        else:
            fragment.lost = False

        
        # if somebody else is transmitting
        if len(fragsOnAir)>0:
            # count detections and signal mWs at GWs
            ## count me
            in_ears_GWs=np.where(heard_at_gateways,1,0) # packet is not yet in packetsOnAir but will be part of the mess
            this_interf_mw_GWs = 10**(.1*self.packet.rss_GWs)

            # count all others on same channel for all the gws of the simu
            for other in fragsOnAir:
                #HYP: any signal (FHSS or Mioty on same subchan, CSS on same CH) interferes; TODO: detail more  
                if self.simu.frequencyCollision(self.pid, other) and self.simu.modulationCollision(self.pid, other):
                    other_heard_at_gateways = self.simu.check_heard_at_GWs(other) 
                    
                    in_ears_GWs+=np.where(other_heard_at_gateways,1,0)

                    other_packet_signal_at_gws_mw = 10**(.1*self.simu.get_rss_GWs(other))

                    # add interference to current packet
                    self.packet.interf_mw_GWs+=other_packet_signal_at_gws_mw
                    # add interference to other packet
                    self.simu.add_interf_GWs(other,this_interf_mw_GWs)

        ################################

            #### EVALUATE COLLISIONS AND CAPTURES 
            for other in fragsOnAir:
                other_heard_at_gateways = self.simu.check_heard_at_GWs(other) 

                for gw_id in range(len(heard_at_gateways)): # for each gw
                    if heard_at_gateways[gw_id]:
                        if other_heard_at_gateways[gw_id]:
                            # NB: fragments channel collision verif is included in frequencyCollision()
                            if self.simu.frequencyCollision(self.pid, other):

                                # check who collides in the power domain, considering interference (and preamble lock if other has preamble)
                                c = self.simu.powerCollision(self.pid, other, in_ears=in_ears_GWs[gw_id], ptype="gw", loc_id=gw_id)
                                # either this one, the other one, or both

                                receiving_fragment = False
                                # mark all the collided packets
                                if self.pid in c: # frag was collided by others 

                                    col = 1
                                    if gw_id in self.myGWs_ids:
                                        gw_local_id = self.myGWs_ids.index(gw_id)
                                        fragment.collided_at_GWs[gw_local_id] = 1
                                        self.n_captured +=1
                                        self.simu.nets[self.net_id].nrCaptures+=1
                                else:
                                    receiving_fragment = True

                                # lock receptor on packet
                                if receiving_fragment:
                                    self.packet.receiving_GWs[gw_id]=True
                                    self.packet.receiving_start_GWs[gw_id]=fragment.timestamp

                                # mark older packets as collided if needed
                                if other in c:
                                    self.simu.mark_collision(other, gw_id)
                                    
        # check impact on listening devices (LoRa-CSS devices)
        self.simu.check_collisions_local(self.pid, collided_at_some_GW = col)


#
# this class creates LR-FHSS packets (associated with a node)
# it also sets all its parameters
#
class LR_FHSS_Packet(Generic_Elements.Packet): 
    def __init__(self, simu, net_id, node_id, plen, distance_GWs, node_profile,
                obw, nb_headers, header_duration, payload_duration, code): #, payload_frag_len
        super().__init__(simu, net_id, node_id, plen, distance_GWs, node_profile)

        self.uses_sub_channels = True
        self.index_transmission = 0
        self.success = 0

        self.code = code
        self.obw = obw
        self.nb_headers = nb_headers
        self.header_duration = header_duration
        self.payload_duration = payload_duration

        self.group = rng.integers(0,8) # random init in one of the 8 groups

    # give tx config info to receiver        
    def get_tx_config(self):
        return({}) # TODO: describe frag subchan... But not in use for now

    # overrides Generic_Element setDataPayloadSize()
    # calls to frag sequence preparation
    def setDataPayloadSize(self):
        super().setDataPayloadSize()
        self.prepare_fragment_sequence()

    # based on data size, prepare sequence
    def prepare_fragment_sequence(self):
        self.match_code(self.code)

        # choose frequencies according to group and obw 
        self.LR_FHSS_channels = rng.choice(range(self.group,self.obw*8,8), size=self.nb_headers+self.nb_payloads)
        self.index_transmission = 0
        self.fragments = []
        # headers
        for h in range(self.nb_headers):
            self.fragments.append(LR_FHSS_Fragment('header',self.header_duration, self.LR_FHSS_channels[h], self.net_id, self.node_id, self.myGWs))
        # payload frags
        for p in range(self.nb_payloads):
            self.fragments.append(LR_FHSS_Fragment('payload',self.payload_duration, self.LR_FHSS_channels[p+h+1], self.net_id, self.node_id, self.myGWs))

    # determine amount of payload fragments and packet successful rx threshold wrt payload size and LR-FHSS coding rate 
    def match_code(self, code):
        if code == '1/3':
                    self.nb_payloads = np.ceil((self.dataPayloadSize+3)/2).astype('int')
                    self.threshold = np.ceil(self.nb_payloads/3).astype('int')
        elif code == '2/3':
                    self.nb_payloads = np.ceil((self.dataPayloadSize+3)/4).astype('int')
                    self.threshold = np.ceil((2*self.nb_payloads)/3).astype('int')
        elif code == '5/6':
                    self.nb_payloads = np.ceil((self.dataPayloadSize+3)/5).astype('int')
                    self.threshold = np.ceil((5*self.nb_payloads)/6).astype('int')
        elif code == '1/2':
                    self.nb_payloads = np.ceil((self.dataPayloadSize+3)/3).astype('int')
                    self.threshold = np.ceil((self.nb_payloads)/2).astype('int')
        else:
                    self.nb_payloads = np.ceil((self.dataPayloadSize+3)/2).astype('int')
                    self.threshold = np.ceil(self.nb_payloads/3).astype('int')             

    # iterate over fragments
    def next(self):
        self.index_transmission+=1
        try:
            return self.fragments[self.index_transmission-1]
        except:
            return False

    # remove my interference when I leave
    def stop_interfering(self):
        self.simu.stop_interfering(self.net_id, self.node_id)

    # determine receptions, losses, collisions, of all splits on all gws of this net
    def try_decode(self):

        for f in self.fragments:
            if not f.lost: #has been heard somewhere
                f.collided = True
                # for gw_id in range(len(self.dist_GWs)):
                for gw_id in range(self.myGWs):
                    if f.lost_at_GWs[gw_id] == 0:
                        if f.collided_at_GWs[gw_id] == 0:
                            f.collided = False
                            break


        for gw_id in range(self.myGWs):
            h_success = 0 
            p_success = 0 
            for f in self.fragments:
                if f.lost_at_GWs[gw_id] == 0:
                    if f.collided_at_GWs[gw_id] == 0:
                        if f.transmitted==1:
                            # f.collided = False
                            if (f.type=='header'):
                                h_success = 1
                            else:
                                p_success +=1
            
            success = 1 if ((h_success>0) and (p_success >= self.threshold)) else 0
            if success == 1:
                self.success = 1
                return True
        return False

        # TODO: add inter-gateways collaboration







# LR-FHSS fragments
class LR_FHSS_Fragment():
    def __init__(self, type, duration, channel, net_id, node_id, myGWs):
        self.node_id = node_id
        self.net_id = net_id
        self.duration = duration
        self.success = 0
        self.transmitted = 0
        self.type = type
        self.channel = channel
        self.timestamp = 0
        self.collided = False
        self.lost = False

        self.collided_at_GWs = [0 for _ in range(myGWs)]
        self.lost_at_GWs = [0 for _ in range(myGWs)]





#
# the LR-FHSS GW object definition
#
class LR_FHSS_Gateway(Generic_Elements.Gateway): 
    def __init__(self, simu, net_id, gw_id, gw_x, gw_y):#, payload_frag_len):
        super().__init__(simu, net_id, gw_id, gw_x, gw_y)

        # self.modulation = "LR-FHSS"

    def get_rx_sensitivity(self, properties_of_packet_being_received):
        return LR_FHSS_get_sensitivity(receiver_type="GW", GW_sensitivity_gain = self.simu.networks[self.net_id]["GW_sensitivity_gain"])



#TODO: adjust sensitivities
def LR_FHSS_get_sensitivity(receiver_type="GW", GW_sensitivity_gain = 0):
    # return 10
    if receiver_type=="DEVICE":
        return(100000) # a FHSS device does not receive 
        return(constants.sensi_subGHz[6, 1]) 

    # a FHSS GW receives anything with the sensitivity of a lora gw for SF12 BW125 frames
    return(-GW_sensitivity_gain+constants.sensi_GW_boosted[125][12]) # -137 - 1.5

