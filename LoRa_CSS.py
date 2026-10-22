# -*- coding: utf-8 -*-
######## The LoRa-CSS components for the LPWAN-Sim Simulator #################################
######## https://github.com/Guillaumegaillard/LPWAN-Sim ######################################
######## 2026-05-18 ##########################################################################
######## License MIT: ########################################################################
######## https://github.com/Guillaumegaillard/LPWAN-Sim/blob/main/LICENSE ####################
######## Inspired from LoRaSim https://mcbor.github.io/lorasim/ ##############################
# and from https://github.com/Guillaumegaillard/LoRa-CSMA-Sim ################################

import simpy
import numpy as np
# rng = np.random.default_rng()

from scipy.signal import savgol_filter
import math

import sys

import Generic_Elements
import constants



############ LoRa vars
CAD_distrib_funcs = {}



#
# retrieve sensitivity according to scenario, config, and receiver type
#
def get_sensitivity(spreading_factor, bandwidth, receiver_type = "GW", techno = "lora868MHz", GW_sensitivity_gain = 0):
    if techno == "lora868MHz":
        if receiver_type == "DEVICE":
            return(constants.sensi_subGHz[spreading_factor - 6, [125, 250, 500].index(bandwidth) + 1]) 
        return(-GW_sensitivity_gain + constants.sensi_GW_boosted[bandwidth][spreading_factor]) # -137 - 1.5
    elif techno == "868MHz_SF7BW500CAD4":
        if receiver_type == "DEVICE":
            return(constants.sensi_subGHz[spreading_factor - 6, [125, 250, 500].index(bandwidth) + 1]) 
        return(-127)
        # return(-150)    
    elif techno == "lora24GHz":
        sensitivity = constants.sensi_2dot4GHz[spreading_factor - 5, [203.125, 406.25, 812.5, 1625].index(bandwidth) + 1]
    else:
        sensitivity = constants.sensi_subGHz[spreading_factor - 6, [125, 250, 500].index(bandwidth) + 1]

    return(sensitivity)


# Check if two packet are transmitted with same spreading factor
# TODO: what about sf/bw interfering pairs?
def sfCollision(sf1, sf2):
    return (sf1 == sf2)


#
# this function computes the airtime of a packet
# according to LoraDesignGuide_STD.pdf
def airtime(sf, cr, pl, bw, explicit = True, lora24GHz = False):
    
    DE = 0         # low data rate optimization enabled ( = 1) or not ( = 0)
    Npream = 8     # number of preamble symbol (12.25     from Utz paper)

    if explicit: # Header
        H = 0
    else:
        H = 1

    if lora24GHz:
        Npream = 12
        H = 1         # header for variable length packet (H = 1) or not (H = 0)        
        if sf > 10:
            # low data rate optimization mandated for SF > 10
            DE = 1
        Tsym = (2.0**sf)/bw
        if sf < 7:
            Tpream = (Npream + 6.25)*Tsym
        else:
            Tpream = (Npream + 4.25)*Tsym
        #print("sf", sf, " cr", cr, "pl", pl, "bw", bw
        if sf >= 7:
            payloadSymbNB = 8 + math.ceil(max((8.0*pl + 16-4.0*sf + 8 + 20*H), 0)/(4.0*(sf-2*DE)))*(cr + 4)
        else:
            payloadSymbNB = 8 + math.ceil(max((8.0*pl + 16-4.0*sf + 20*H), 0)/(4.0*(sf-2*DE)))*(cr + 4)
        Tpayload = payloadSymbNB * Tsym
        return (Tpream + Tpayload)     
    else:
        if bw == 125 and sf in [11, 12]:
            # low data rate optimization mandated for BW125 with SF11 and SF12
            DE = 1
        if sf == 6:
            # can only have implicit header with SF6
            H = 0
        Tsym = (2.0**sf)/bw
        Tpream = (Npream + 4.25)*Tsym
        # CR is 1, 2, 3 or 4 for respective coding rates 4/5, 4/6, 4/7 or 4/8
        payloadSymbNB = 8 + max(math.ceil((8.0*pl-4.0*sf + 28 + 16-20*H)/(4.0*(sf-2*DE)))*(cr + 4), 0)
        Tpayload = payloadSymbNB * Tsym
        return(Tpream + Tpayload)


#
# the LoRa-CSS node (device) object definition
#
class LoRa_CSS_Node(Generic_Elements.Node):
    # a LoRa_CSS node has no more external attributes than the generic ones
    def __init__(self, simu, net_id, node_id, nodex, nodey, node_cluster, node_profile, nodeType, period, distrib, packetlen):
        super().__init__(simu, net_id, node_id, nodex, nodey, node_cluster, node_profile, nodeType, period, distrib, packetlen)

        self.modulation = "LoRa-CSS"
        # node has a packet
        self.packet = LoRa_CSS_Packet(simu, self.net_id, self.node_id, self.packetlen, self.dist_GWs, self.profile)

        # packet has a time on air called rectime, init with default payload (init value of pl)
        self.data_rectime = airtime(self.packet.sf, self.packet.cr, self.packet.pl, self.packet.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0))

        # configure specific details for CANL22
        if self.simu.networks[self.net_id]["profiles"][self.profile]["protocol"] == "CANL22":
            # configure a RTS packet for CANL22 (not in use in 2025)
            # if a RTS is used, its payload is 1 byte
            self.rts_rectime = airtime(self.packet.sf, self.packet.cr, self.simu.networks[self.net_id]["profiles"][self.profile]["CANL22_RTS_hdr_size"] + 1, self.packet.bw, explicit = (self.simu.networks[self.net_id]["profiles"][self.profile]["CANL22_RTS_PHY_HDR"] == 0))

            # wait_PHY_interrupt: time a device stays listening in order to obtain data length information
            # device is capable of using header right after the "header valid" interrupt
            if self.simu.networks[self.net_id]["profiles"][self.profile]["Interrupts_on_header_valid"]: 
                # CR is 1, 2, 3 or 4 for respective coding rates 4/5, 4/6, 4/7 or 4/8
                # header is tx with cr 4, but airtime func must compute this "hdr at cr 4" phase
                # does not include a processing time.

                # 
                # self.wait_PHY_interrupt = airtime(self.packet.sf, 4, 0, self.packet.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0))
                self.wait_PHY_interrupt = airtime(self.packet.sf, self.packet.cr, 0, self.packet.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0))
            else:
                self.wait_PHY_interrupt = self.rts_rectime
        else: # RTS is not used anyway, let's consider a 5B length for compatibility
            self.rts_rectime = airtime(self.packet.sf, self.packet.cr, 5, self.packet.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0))
            self.wait_PHY_interrupt = airtime(self.packet.sf, self.packet.cr, 5, self.packet.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0))

        #### Variables for collision avoidance /listen
        self.MAC_listen_start_time = 0
        self.MAC_listen_end_time = 0
        self.listened_time = -1

        self.my_P = 0 # probability for an old CANL version (not in use 2025)
        self.backoff = 0 # time to be spent in BO, updated by protocols
        self.remainder = -1 # remaining steps to be done in a residual active BO

        ## variables for listening          
        self.heard_frames = [] # list of heard frames and metadata
        self.add_rx_time_opportunities = 1 # when a frame starts being received, how many times the listen window can be elongated.

        ## listen result variables
        self.I_heard_preamble = False
        self.I_heard_activity = False
        self.I_know_it_is_Data = False
        self.I_know_it_is_RTS = False
        self.next_payload_byte = 0

        ## DEFAULT NAV PERIOD, which is the toa(max sized packet) - one CAD time
        self.nav_period_default = airtime(self.packet.sf, self.packet.cr, self.simu.networks[self.net_id]["max_payload_size"], self.packet.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0)) - self.packet.symTime*(self.simu.networks[self.net_id]["nCadSym"] + 1/2)

        # variable initialized at default
        self.nav_period = self.nav_period_default

        self.nav = 0 # variable indicating a required NAV

        ### mechanism of n_retry:
        ## variable number of remaining possible attempts for the current packet 
        # init according to profile
        self.n_retry = self.simu.networks[self.net_id]["profiles"][self.profile]["n_retry"] if "n_retry" in self.simu.networks[self.net_id]["profiles"][self.profile] else -1

        ## BO exponent for BO when BUSY (init at value in profile)
        if "Wbusy_BE" in self.simu.networks[self.net_id]["profiles"][self.profile]:
            self.Wbusy_BE = self.simu.networks[self.net_id]["profiles"][self.profile]["Wbusy_BE"]



    # retrieve SF, BW of a transmitter, => enable simulation to do rx   
    def get_tx_config(self):
        return self.packet.get_tx_config()

    # no subchans for LoRa, here simply translate self.ch into its center+bw definition in constants
    def get_chan(self):
        return(constants.Channels[self.modulation]["Channel_list"][self.ch])

    # current time-on-air is called rectime here 
    def get_ongoing_duration(self):
        return(self.packet.rectime)

    # get start time of current transmission
    def get_ongoing_start(self):
        return(self.packet.addTime)

    # get sensitivity of a receiver, according to BW and SF of tx and rx
    # non-CSS are assumed sf12bw125
    def get_rx_sensitivity(self, properties_of_signal_being_received, receiver_type = "DEVICE"):
        # currently, a device may only receive on the SF it is configured on.
        # HYP: LRFHSS fragments are assumed to be sensed as SF12BW125 frames 
        if not ("sf" in properties_of_signal_being_received): 
            # FHSS fragment is sensed as a sf12 bw 125, TODO: differentiate
            properties_of_signal_being_received["sf"] = 12
            properties_of_signal_being_received["bw"] = 125
        if sfCollision(properties_of_signal_being_received["sf"], self.packet.sf): # TODO: and BW ?
            return get_sensitivity(self.packet.sf, self.packet.bw, receiver_type = receiver_type)
            # TODO: should be self.current_sf, etc. (ie dev's rx properties, not packet (tx) ones)
        return 10000 #(dB, that is very much, meaning that it is impossible to rx smth on another SF)

    # retrieve SF
    def getModulationParam(self):
        return self.packet.sf

    # compare sfs
    def modulationCollision(self, sf1, sf2):
        return sfCollision(sf1, sf2)



    ############################
    ############################
    # check for receptions/collisions/captures at gateway(s) and among devices
    # Note: called before a packet (identified with node id) is inserted into the list of ongoing signals
    def check_collision(self, packetsOnAir):

        heard_at_gateways = self.simu.check_heard_at_GWs(self.pid)

        # for each of my gws
        for gw_id in range(self.myGWs):
            if not heard_at_gateways[self.myGWs_ids][gw_id]:
                self.packet.lost_at_GWs[gw_id] = 1
            else:
                self.packet.lost_at_GWs[gw_id] = 0

        if not heard_at_gateways[self.myGWs_ids].any(): # none of my gws receive it, too far
            self.packet.lost = True
        else:
            self.packet.lost = False

        col = 0 # flag needed since there might be several collisions for packet

        
        # TODO: packet.processed not in use/not fully implemented
        self.packet.processed = self.simu.can_process(self.net_id)

        # if somebody else is transmitting
        if len(packetsOnAir) > 0:
            # count detections and signal mWs at GWs
            ## count me
            in_ears_GWs = np.where(heard_at_gateways, 1, 0) # packet is not yet in packetsOnAir but will be part of the mess
            this_interf_mw_GWs = 10**(.1*self.packet.rss_GWs)

            # count all others
            for other in packetsOnAir:
                if self.simu.frequencyCollision(self.pid, other) and self.simu.modulationCollision(self.pid, other): #TODO: bw? 
                    other_heard_at_gateways = self.simu.check_heard_at_GWs(other) 

                    in_ears_GWs += np.where(other_heard_at_gateways, 1, 0)

                    other_packet_signal_at_gws_mw = 10**(.1*self.simu.get_rss_GWs(other))

                    # add interference to current packet
                    self.packet.interf_mw_GWs += other_packet_signal_at_gws_mw
                    # add interference to other packet
                    self.simu.add_interf_GWs(other, this_interf_mw_GWs)



            #### EVALUATE COLLISIONS AND CAPTURES AT GWs
            for other in packetsOnAir:

                other_heard_at_gateways = self.simu.check_heard_at_GWs(other) 

                for gw_id in range(len(heard_at_gateways)): # for each gw
                    if heard_at_gateways[gw_id]:
                        #check if this other packet is (different than packet and) heard at GW
                        if other_heard_at_gateways[gw_id]:

                            # check if same channel, and if modulations are not orthogonal
                            if self.simu.frequencyCollision(self.pid, other) and self.simu.modulationCollision(self.pid, other):
                                if self.simu.networks[self.net_id]["full_collision"]: # (this global is always true since 2022...)
                                    # check who collides in the power domain, considering interference and preamble lock
                                    # either this one, the other one, or both
                                    c = self.simu.powerCollision(self.pid, other, in_ears = in_ears_GWs[gw_id], ptype = "gw", loc_id = gw_id)


                                    receiving_packet = False
                                    # mark all the collided packets
                                    if self.pid in c: # frag was collided by others 
                                        # packet collides in power, check if the other finishes before most of preamble 
                                        clear_pream_time_heard_here, not_enough_preamble = self.simu.timingCollision(self.pid, other, ptype = "gw", loc_id = gw_id)

                                        if not_enough_preamble:# packet won't be rec there
                                            if gw_id in self.myGWs_ids:
                                                gw_local_id = self.myGWs_ids.index(gw_id)
                                                self.packet.collided_at_GWs[gw_local_id] = 1
                                                self.n_captured += 1
                                                self.simu.nets[self.net_id].nrCaptures += 1

                                                col = 1

                                                if self.simu.log_events:
                                                    self.simu.MainLogger.info(("GW{0}".format(gw_id), "col", self.pid, other, self.simu.env.now))
                                        else:
                                            receiving_packet = True
                                    else:
                                        receiving_packet = True

                                    # lock receptor on packet
                                    if receiving_packet:
                                        self.packet.receiving_GWs[gw_id] = True
                                        self.packet.receiving_start_GWs[gw_id] = self.simu.env.now

                                    # mark older packets as collided if needed
                                    if other in c:
                                        self.simu.mark_collision(other, gw_id)
                       
                                else: # both collide

                                    self.simu.mark_collision(self.pid, gw_id)
                                    self.simu.mark_collision(other, gw_id)
                                    col = 1
                

        ############### Packet has been checked for reception at GWs. Let's see elsewhere
        # if not full_distances:    
        if not self.simu.networks[self.net_id]["full_distances"]:
            #if col == 1 it means that the new packet can not be decoded
            if col:
                return col    
                
        # TODO: update old logs for multiGW
        if not col:
            if self.simu.log_events:
                self.simu.MainLogger.info(("GW", "rx", self.packet.pid, self.simu.env.now))

        # check receptions/collisions/impact on listening devices
        self.simu.check_collisions_local(self.pid, collided_at_some_GW = col)

        return col


    # determine how much extra time needed to detect preamble, if possible
    def time_to_detect_preamble(self):
        nec_time = -1

        lhf = len(self.heard_frames)
        if lhf == 0:#I heard nothing, go back listening
            pass 
        else:
            for frame_heard_id in range(lhf):# in chrono order
                frame_heard = self.heard_frames[frame_heard_id]

                if self.simu.networks[frame_heard["node_id"][0]]["modulation"] == "LoRa-CSS": # signals other than lora do not trigger preamble detection

                    # was it captured from the beginning?
                    if len(frame_heard["interfered_by"]) == 0: # no
                        #did I hear it start? (was i listening, is preamble finished?)
                        # yes, => wait for more info / stop listening
                        # no, => wait for more info / stop listening

                        # I am listening and the preamble had not yet finished
                        if (self.simu.env.now < frame_heard["start"] + self.packet.Tpream - self.packet.Tmin_preamb_heard):
                            
                            # two cases: no frame before//frame before
                            # frame before
                            if frame_heard["clear_pream_time_heard"] < self.packet.Tpream:
                                start_hearing_it = max(
                                    frame_heard["start"] + self.packet.Tpream - frame_heard["clear_pream_time_heard"],
                                    self.simu.env.now
                                    )
                            else:
                                start_hearing_it = max(
                                    frame_heard["start"],
                                    self.simu.env.now
                                    )


                            # was it captured after its beginning (and before now)? 
                            first_capturer = -1
                            for nextid in range(frame_heard_id + 1, lhf): # last does not enter here
                                if (frame_heard["node_id"], constants.rtsPacketType if frame_heard["is_RTS"] else constants.dataPacketType) in self.heard_frames[nextid]["interfering"]:
                                    first_capturer = nextid
                                    break
                            if first_capturer != -1: # yes
                                # Then I don't hear it, no matter what...
                                pass
                            else:
                                # (will be) hearing enough preamble symbols if not interrupted 
                                return (frame_heard["id"], (start_hearing_it + self.packet.Tmin_preamb_heard - self.simu.env.now))


        return (-1, nec_time)


    # determine how much extra time needed to detect full (valid) header (vh), if possible
    def time_to_wait_vh(self, start_of_listening, frame_id):

        lhf = len(self.heard_frames)
    
        for frame_heard_id in range(lhf):# in chrono order
            frame_heard = self.heard_frames[frame_heard_id]
            if frame_heard["id"] == frame_id:
                break

        # two cases: no frame before//frame before
        # frame before
        if frame_heard["clear_pream_time_heard"] < self.packet.Tpream:
            started_hearing_it = max(
                frame_heard["start"] + self.packet.Tpream - frame_heard["clear_pream_time_heard"],
                start_of_listening
            )
        else:
            started_hearing_it = max(
                frame_heard["start"],
                start_of_listening
            )

        return((frame_heard["start"] + self.wait_PHY_interrupt - self.simu.env.now), (started_hearing_it + self.wait_PHY_interrupt + self.packet.symTime - self.simu.env.now)) # margin one symbol

    # determine if frame has been impacted by later frames
    def check_frame_captured(self, frame_id):
        lhf = len(self.heard_frames)
    
        for frame_heard_id in range(lhf):# in chrono order
            frame_heard = self.heard_frames[frame_heard_id]
            if frame_heard["id"] == frame_id:
                break

        # was it captured after its beginning (and before now)? 
        first_capturer = -1
        for nextid in range(frame_heard_id + 1, lhf): # 
            if (frame_heard["node_id"], constants.rtsPacketType if frame_heard["is_RTS"] else constants.dataPacketType) in self.heard_frames[nextid]["interfering"]:
                first_capturer = nextid
                break

        return(first_capturer != -1)              


    # grab info from header
    def read_header(self, frame_id):

        lhf = len(self.heard_frames)
    
        for frame_heard_id in range(lhf):# in chrono order
            frame_heard = self.heard_frames[frame_heard_id]
            if frame_heard["id"] == frame_id:
                break

        if frame_heard["toa"] > self.wait_PHY_interrupt: # I know it has payload
            self.I_know_it_is_Data = True
            # assert(frame_heard["is_RTS"] == False)
            if self.simu.networks[self.net_id]["profiles"][self.profile]["Interrupts_on_header_valid"]:
                self.next_payload_byte = frame_heard["dataPayloadSize_in_EH"]
        else: # I know it has no payload
            if frame_heard["is_RTS"]: #I know it by CANL header differentiation (data, RTS, ACK) 
                self.I_know_it_is_RTS = True
                self.next_payload_byte = frame_heard["dataPayloadSize_in_RTS"]
            else: # short data
                self.I_know_it_is_Data = True
                if self.simu.networks[self.net_id]["profiles"][self.profile]["Interrupts_on_header_valid"]:
                    self.next_payload_byte = frame_heard["dataPayloadSize_in_EH"]

    # listen process: can be initiated by protocol main loop or elongated if necessary
    def listen(self, duration):
        start_of_listening = self.simu.env.now

        # check if a rx has started (otherwise detecting_preamble == -1)
        frame_id, detecting_preamble = self.time_to_detect_preamble()

        # elongate if permitted
        while self.add_rx_time_opportunities >= 0 and detecting_preamble > 0:
            # listen full preamble or catch a new heard rx 
            try:
                yield self.simu.env.timeout(detecting_preamble)
                self.I_heard_preamble = True                
                break
            except simpy.Interrupt:
                # if current captured during preamble, new packet can be heard (e.g. if strong enough)
                # recheck detection
                frame_id, detecting_preamble = self.time_to_detect_preamble()
                # if later detected, remove one opportunity 
                if detecting_preamble != -1 and (self.simu.env.now + detecting_preamble - start_of_listening > duration):
                    self.add_rx_time_opportunities -= 1

        # same idea, once preamble heard, wait for header
        if self.I_heard_preamble:

            #wait vh
            waiting_for_vh, waiting_default = self.time_to_wait_vh(start_of_listening, frame_id)
            start_waiting_for_vh = self.simu.env.now

            heard_vh = True # will get False if capture
            time_waiting = waiting_for_vh
            while True:

                # whether wait or be interrupted
                try:
                    yield self.simu.env.timeout(time_waiting)
                    break
                except simpy.Interrupt:
                    captured = self.check_frame_captured(frame_id)
                    if captured: # I won't hear valid header, so I wait default (full preamble)
                        heard_vh = False
                        time_waiting = waiting_default - (self.simu.env.now - start_waiting_for_vh)
                    else: # I can hear end of header, and stop listening 
                        time_waiting = waiting_for_vh - (self.simu.env.now - start_waiting_for_vh)

            # if header heard, update listener's info
            if heard_vh:
                self.read_header(frame_id)
            else:
                pass
            return(-1)

        else:
            #default listening
            start_default_listen = self.simu.env.now
            remaining_to_listen = duration - (start_default_listen - start_of_listening)
            while remaining_to_listen > 0:
                # wait or be interrupted
                try:
                    yield self.simu.env.timeout(remaining_to_listen)
                    break
                except simpy.Interrupt: # something was heard, break, and re_entering, decide if resume or stop listening
                    remaining_to_listen -= (self.simu.env.now - start_default_listen)
                    return(remaining_to_listen)
            return(-1)

        
    # node function called at beginning of a listen phase
    # Check if packets on air are heard and impacting the node
    def start_listening(self, packetsOnAir):  

        # for each packet on air oldest first 
        #     was it heard?
        #     has it been captured before even its begin?
        #     does it capture previous?

        if len(packetsOnAir) > 0:
            # count current mWs of interference and current interferers 


            full_mw_devs = 0
            interfering_me = []
            heards = []
            in_ears = 0
            for other in packetsOnAir:
                if self.simu.frequencyCollision(self.pid,other) and self.modulationCollision(self.pid,other):
                    # full_mw_devs += 10**(.1*(nodes[other].packet.rx_array[self.pid]))
                    these_mWs = 10**(.1*(self.simu.get_rss(other, self.pid)))
                    full_mw_devs += these_mWs
                    interfering_me.append((other,these_mWs))
                if self.simu.check_heard(other, self.pid):
                    in_ears += 1
                    heards.append(other)


            # interference suffered by a packet p (other) at position self is the full sum of strengths received at self minus p's received mWs 
            for other,these_mWs in interfering_me:
                self.simu.set_interf(other, self.pid, full_mw_devs - these_mWs)


            ## check collisions/captures, in chrono order and chrono order of preceding
            for pkid in range(len(packetsOnAir)):
                # packet = nodes[packetsOnAir[pid]].packet
                # if check_heard(packet, self.pid): #NB: check_heard checks frequency channel only
                # if self.simu.check_heard(packetsOnAir[pkid], self.pid):
                if packetsOnAir[pkid] in heards:
                    previous_frames_impacting_this_one = []
                    previous_frames_impacted_by_this_one = []   
                    clear_pream_time_heard = 10000 # 10s  
                    locally_collided = False                  
                    if pkid != 0: # more than one on air
                        for opkid in range(pkid):
                            other = packetsOnAir[opkid]

                            #NB: check_heard checks channel and modulation (sf) for tx and rx
                            # if self.simu.modulationCollision(self.pid, other) and self.simu.modulationCollision(self.pid, packetsOnAir[pkid]):
                            # if self.simu.check_heard(other, self.pid):
                            if other in heards:
                                if self.simu.networks[self.net_id]["full_collision"]:
                                    # check who collides/captures/interfers in the power domain
                                    # account for interf impact and lock of rx
                                    c = self.simu.powerCollision(packetsOnAir[pkid], other, in_ears = in_ears, loc_id = self.pid, ocurring_now = False)
                                    # either this one, the other one, or both

                                    receiving_packet = False
                                    # mark all the power-impacted packets
                                    if packetsOnAir[pkid] in c:
                                        # check timing collision (whether packet is spared by preamble or not) 
                                        clear_pream_time_heard_here, not_enough_preamble = self.simu.timingCollision(packetsOnAir[pkid], other, ocurring_now = False, ptype = "dev", loc_id = self.pid)

                                        # preamble is heard clearly only after the last ending interfering frame (not necessarily the last starting if sizes are different)
                                        clear_pream_time_heard = min(clear_pream_time_heard, clear_pream_time_heard_here)


                                        if not_enough_preamble:# packet missed
                                            locally_collided = True
                                            previous_frames_impacting_this_one.append((other, self.simu.get_ptype(other)))
                                            if self.simu.log_events:
                                                self.simu.MainLogger.info((self.pid, "col", packetsOnAir[pkid], other, self.simu.env.now))
                                        else:
                                            receiving_packet = True
                                    else:
                                        receiving_packet = True

                                    # if preamble clear, start receiving it until a following packet comes impacting (further in loop)
                                    if receiving_packet:
                                        self.simu.set_local_receiving(packetsOnAir[pkid], self.pid)

                                    # impact on others (preceding)
                                    if other in c:
                                        if self.simu.log_events:
                                            self.simu.MainLogger.info((self.pid, "col", packetsOnAir[pkid], other, self.simu.env.now))
                                        previous_frames_impacted_by_this_one.append((other, self.simu.get_ptype(other)))
                                        # unlock if impacted:
                                        self.simu.set_local_receiving(other, self.pid, unset = True)
                                        # nodes[other].packet.receiving[self.pid] = False

                                else: # both collide
                                    if self.simu.log_events:
                                        self.simu.MainLogger.info((self.pid, "col", packetsOnAir[pkid], other, self.simu.env.now))
                                        self.simu.MainLogger.info((self.pid, "col", other, packetsOnAir[pkid], self.simu.env.now))
                                    previous_frames_impacting_this_one.append((other, self.simu.get_ptype(other)))
                                    previous_frames_impacted_by_this_one.append((other, self.simu.get_ptype(other)))
                                    locally_collided = True

                    dpls = self.simu.get_dataPayloadSize(packetsOnAir[pkid])
                    # store info for analysis at the end of the listen
                    self.heard_frames.append({
                        "id":len(self.heard_frames),
                        "node_id":packetsOnAir[pkid],
                        "toa":self.simu.get_rectime(packetsOnAir[pkid]),
                        "is_RTS":self.simu.get_ptype(packetsOnAir[pkid]) == constants.rtsPacketType,
                        "dataPayloadSize_in_RTS":self.simu.rng.integers(0, self.simu.networks[self.net_id]["max_payload_size"] + 1) if dpls == (self.simu.networks[self.net_id]["profiles"][self.profile]["CANL22_RTS_hdr_size"] + 1) else dpls, # if Data of RTS size, random size
                        "dataPayloadSize_in_EH":dpls, # in explicit header
                        "start":self.simu.get_addTime(packetsOnAir[pkid]),
                        "interfering":previous_frames_impacted_by_this_one,
                        "interfered_by":previous_frames_impacting_this_one,
                        "clear_pream_time_heard":clear_pream_time_heard
                    })

                    if not locally_collided:
                        if self.simu.log_events:
                            self.simu.MainLogger.info((self.pid, "rx", packetsOnAir[pkid], self.simu.env.now))



    #
    ## CAD mechanism "requires" energy is received from a transmitter during all the CAD duration, hence we need a copy of the global on-air list at beginning of CAD
    def start_CAD(self):
        return self.simu.get_transmitters_in_band(self.pid)

    #
    ## compute CAD success for transmissions assumed continuous during full period  
    def stop_CAD(self, on_air_at_CAD_start):
        on_air_at_CAD_stop = self.simu.get_transmitters_in_band(self.pid)

        #Hyp: no blank of less than CAD duration between two tx of same device (if device n is transmitting at start and at stops => it is assumed to have been transmitting during all the CAD time)
        for devid in on_air_at_CAD_stop:
            if devid in on_air_at_CAD_start:
                if self.simu.networks[self.net_id]["var_CAD_prob"]: #general case 
                    if self.simu.networks[self.net_id]["full_distances"]:

                        if self.simu.rng.random()*100 <= get_CAD_prob(self.simu.get_dist(self.pid, devid), self.simu.get_txpow(devid), self.net_id, self.simu.same_cluster(self.pid, devid)):
                            if self.simu.log_events:
                                self.simu.MainLogger.info((self.pid, "CAD + ", self.simu.env.now))
                            return (True)
                    else:
                        if self.simu.rng.random()*100 <= get_CAD_prob(self.dist_to_center, self.simu.get_txpow(devid), self.net_id, self.simu.same_cluster(self.pid, devid)):
                            if self.simu.log_events:
                                self.simu.MainLogger.info((self.pid, "CAD + ", self.simu.env.now))                        
                            return (True)                    
                else: # for testing e.g. all CAD have 80% success rate
                    if self.simu.rng.random()*100 <= self.simu.networks[self.net_id]["CAD_prob"] and self.simu.networks[self.net_id]["CAD_prob"] != 0:
                        if self.simu.log_events:
                            self.simu.MainLogger.info((self.pid, "CAD + ", self.simu.env.now))                    
                        return (True)                    
        if self.simu.log_events:
            self.simu.MainLogger.info((self.pid, "CAD-", self.simu.env.now))
        return False              


    # main loop for device
    def communicate(self):

        my_node_profile = self.simu.networks[self.net_id]["profiles"][self.profile]

        #Main loop for each node
        while True:
        
            ###////////////////////////////////////////////////////////
            # CANL - xCANL_CAD - variants with RTS                    /
            ###////////////////////////////////////////////////////////        
            if my_node_profile["protocol"] == "CANL22":
                ###########################################################
                # schedule_tx -> want_transmit                            #
                # want_transmit -> listen_1                               #
                # listen_1 -> NAV_state | send_RTS                        #
                # NAV_state -> want_transmit                              #
                # send_RTS -> listen_2 | send_DATA                        #
                # listen_2 -> NAV_state | send_DATA                       #
                # send_DATA -> schedule_tx                                #
                ###########################################################

                ###########################################################
                # schedule_tx -> want_transmit                            #
                ###########################################################
                if self.MAC_state == constants.schedule_tx:
                
                    ## scheduling new generations
                    yield self.simu.env.process(self.generate_new_frames())



                ###########################################################
                # want_transmit -> listen_1                               #
                ###########################################################
                if self.MAC_state == constants.want_transmit:# there is something to transmit (first time or retry)
                    if self.n_retry == 0: # no more trials possible, abort or ALOHA
                        self.remainder = -1
                        if my_node_profile["abort_after_retries"]: # abort
                            self.n_aborted = self.n_aborted + 1
                            #reset for sending a new packet              
                            
                            self.total_retry += my_node_profile["n_retry"]
                            self.max_retries = my_node_profile["n_retry"]
                            self.n_retry = my_node_profile["n_retry"]
                            self.nav = 0
                            self.attempt_latency += (self.simu.env.now - self.attempt_start)
                            self.min_attempt_latency = min(self.min_attempt_latency, (self.simu.env.now - self.attempt_start))
                            self.max_attempt_latency = max(self.max_attempt_latency, (self.simu.env.now - self.attempt_start))


                            self.update_available_channels()

                            self.MAC_state = constants.schedule_tx

                        else: # kamikaze the frame
                            yield self.simu.env.timeout(self.nav_period) # wait for known
                            self.backoff = self.simu.rng.integers(my_node_profile["Wcanl_min_sym"], my_node_profile["Wcanl_max_sym"] + 1)*self.packet.symTime# 64 == 2**Wbusy_maxBE
                            yield self.simu.env.timeout(self.backoff) # wait a random bo
                            self.kamikaze += 1
                            self.MAC_state = constants.CANL_send_DATA # go sending
                            self.backoff = 0
                    else: #try or retry
                        if self.nav != 0: # then it is a retry
                            #reset nav to start again a complete CA procedure
                            self.nav = 0
                            self.nav_period = self.nav_period_default
                            self.attempt_latency += (self.simu.env.now - self.attempt_start)
                            self.min_attempt_latency = min(self.min_attempt_latency, (self.simu.env.now - self.attempt_start))
                            self.max_attempt_latency = max(self.max_attempt_latency, (self.simu.env.now - self.attempt_start))
                            
                        else:
                            #this is an initial transmit attempt
                            # set stats


                            self.want_transmit_time = self.simu.env.now

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
                
                        self.attempt_start = self.simu.env.now
                        
                        if my_node_profile["CANL22_P"] != 0: # never used since 2023
                            #Obsolete: set proba defining if the node transmits data right after RTS or after a "listen 2" phase
                            self.my_P = self.simu.rng.integers(0, 101)

                        # CADs before LISTEN, optional
                        channel_found_busy = False                    
                        CAD_time = 0
                        if my_node_profile["CANL22_check_busy"] and self.channel_just_changed:
                            self.channel_just_changed = False
                            self.MAC_state = constants.CANL_CAD

                            if my_node_profile["nCAD_DIFS"] > 0:
                                no_difs_has_been_found_clear_yet = True
                                while no_difs_has_been_found_clear_yet:

                                    for CAD_id in range(my_node_profile["nCAD_DIFS"]):
                                        self.n_CAD = self.n_CAD + 1

                                        if self.simu.env.now - self.radio_on_last > self.radio_wake_up_time:
                                            yield self.simu.env.timeout(self.radio_wake_up_time)
                                            self.radio_wake_up_number += 1

                                        ### Wait CAD Duration
                                        CAD_time = self.packet.symTime*self.simu.networks[self.net_id]["nCadSym"]
                                        CAD_start_time = self.simu.env.now

                                        on_air_at_CAD_start = self.start_CAD() # simulator stores transmitters at CAD start
                                        yield self.simu.env.timeout(CAD_time)
                                        channel_found_busy = self.stop_CAD(on_air_at_CAD_start) # determines CAD + or -
                                        yield self.simu.env.timeout(self.packet.symTime/2) # process CAD
                                        self.simu.add_listen_log((self.simu.env.now, CAD_start_time, CAD_time, self.pid, self.cycle, int(self.ch))) ## convert numpy.int64 to int for jsonify
                                        self.radio_on_last = self.simu.env.now

                                        if channel_found_busy or (CAD_id == (my_node_profile["nCAD_DIFS"]-1)) :
                                            break
                                        to_wait_bw_cads = self.packet.symTime*my_node_profile["DIFS_inter_CAD_sym"]

                                        if to_wait_bw_cads > self.radio_wake_up_time:
                                            self.radio_on_last = -10000 # force re-wake_up of radio
                                            to_wait_bw_cads -= self.radio_wake_up_time

                                        yield self.simu.env.timeout(to_wait_bw_cads)


                                    if channel_found_busy: # DIFS went busy
                                        if my_node_profile["CH_DIFS"]: 
                                            if self.update_channel(max_changes = len(constants.Channels[self.modulation]["Channel_list"]), limited_changes = True, after_fail = True):
                                                yield self.simu.env.timeout(self.radio_config_time)
                                                self.radio_config_number += 1
                                            else: # all channels have been trialed, go NAV
                                                # self.update_available_channels() 

                                                break # while
                                        else: # Do single difs, break anyway
                                            break # while                        
                                    else: # chan found free
                                        break # while    
            
                        if channel_found_busy:
                            self.nav += 1         
                            #will go into NAV
                            self.MAC_state = constants.CANL_NAV_state  
                            
                        else: # go channel clear state, then

                            # change packet type to get the correct time-on-air for calculation
                            self.packet.setPacketType(constants.rtsPacketType)
                            # packet is now of RTS type even in scenario without rts.. will be changed back later on.


                            if my_node_profile["BO_when_clear"]:
                                
                                if my_node_profile["active_clear_BO"]: #perform CADs during BO
                                    if self.remainder == -1:

                                        # self.backoff = rng.integers(Wclear_min_sym, 2**self.Wclear_BE + 1)                        
                                        self.remainder = self.simu.rng.integers(my_node_profile["Wclear_min_CAD"], my_node_profile["Wclear_max_CAD"] + 1)                        

                                    for CAD_id in range(self.remainder):
                                        self.n_CAD = self.n_CAD + 1

                                        if self.simu.env.now - self.radio_on_last > self.radio_wake_up_time:
                                            yield self.simu.env.timeout(self.radio_wake_up_time)
                                            self.radio_wake_up_number += 1

                                        ### Wait CAD Duration
                                        CAD_time = self.packet.symTime*self.simu.networks[self.net_id]["nCadSym"]
                                        CAD_start_time = self.simu.env.now


                                        on_air_at_CAD_start = self.start_CAD() # simulator stores transmitters at CAD start
                                        yield self.simu.env.timeout(CAD_time)
                                        channel_found_busy = self.stop_CAD(on_air_at_CAD_start) # determines CAD + or -
                                        yield self.simu.env.timeout(self.packet.symTime/2) # process CAD
                                        self.simu.add_listen_log((self.simu.env.now, CAD_start_time, CAD_time, self.pid, self.cycle, int(self.ch)))
                                        self.radio_on_last = self.simu.env.now                                

                                        if channel_found_busy:
                                            self.I_heard_activity = True
                                            break

                                        if (CAD_id == (self.remainder-1)) : # all clear, go transmit
                                            break

                                        to_wait_bw_cads = self.packet.symTime*my_node_profile["BO_inter_CAD_sym"]

                                        if to_wait_bw_cads > self.radio_wake_up_time:
                                            self.radio_on_last = -10000 # force re-wake_up of radio
                                            to_wait_bw_cads -= self.radio_wake_up_time

                                        yield self.simu.env.timeout(to_wait_bw_cads)

                                    if channel_found_busy:
                                        self.remainder = self.remainder-CAD_id # last was busy, we need remainder clears to exit BO

                                    else: # BO is successfully finished
                                        self.remainder = -1
                                        # print("BO is successfully finished", file = sys.stderr)

                                else: #passive BO when clear

                                    self.backoff = self.packet.symTime*self.simu.rng.integers(my_node_profile["Wclear_min_sym"], my_node_profile["Wclear_max_sym"] + 1)
                                    yield self.simu.env.timeout(self.backoff)
                                    self.backoff = 0


                                    if my_node_profile["nCAD_DIFS"] > 0:
                                        for CAD_id in range(my_node_profile["nCAD_DIFS"]):
                                            self.n_CAD = self.n_CAD + 1

                                            if self.simu.env.now - self.radio_on_last > self.radio_wake_up_time:
                                                yield self.simu.env.timeout(self.radio_wake_up_time)
                                                self.radio_wake_up_number += 1

                                            ### Wait CAD Duration
                                            CAD_time = self.packet.symTime*self.simu.networks[self.net_id]["nCadSym"]
                                            CAD_start_time = self.simu.env.now

                                            on_air_at_CAD_start = self.start_CAD() # simulator stores transmitters at CAD start
                                            yield self.simu.env.timeout(CAD_time)
                                            channel_found_busy = self.stop_CAD(on_air_at_CAD_start) # determines CAD + or -
                                            yield self.simu.env.timeout(self.packet.symTime/2) # process CAD
                                            self.simu.add_listen_log((self.simu.env.now, CAD_start_time, CAD_time, self.pid, self.cycle, int(self.ch)))
                                            self.radio_on_last = self.simu.env.now                                

                                            if channel_found_busy:
                                                self.I_heard_activity = True
                                                break

                                            if (CAD_id == (my_node_profile["nCAD_DIFS"]-1)) :
                                                break
                                            
                                            to_wait_bw_cads = self.packet.symTime*my_node_profile["DIFS_inter_CAD_sym"]

                                            if to_wait_bw_cads > self.radio_wake_up_time:
                                                self.radio_on_last = -10000 # force re-wake_up of radio
                                                to_wait_bw_cads -= self.radio_wake_up_time

                                            yield self.simu.env.timeout(to_wait_bw_cads)

                                    else:
                                        channel_found_busy = False


                            # listen will be done here:
                            ## 1: when no BO is applied
                            ## 2: when BO gives BUSY
                            ## TODO: truly passive BO? 
                            ## TODO: if first CAD in DIFS is busy, frame probably started long ago; worth a Listen?
                            if (my_node_profile["BO_when_clear"] and channel_found_busy) or not my_node_profile["BO_when_clear"]:

                                if self.simu.env.now - self.radio_on_last > self.radio_wake_up_time:
                                    yield self.simu.env.timeout(self.radio_wake_up_time)
                                    self.radio_wake_up_number += 1

                                self.MAC_state = constants.CANL_listen1
                                self.listening = True

                                self.MAC_listen_start_time = self.simu.env.now

                                self.simu.start_listening(self.pid) 


                                # switch bw softer fairness or basic fairness (reduction of listen win max wrt n_retry)
                                if my_node_profile["CANL22_softer_fair"]:
                                    if self.n_retry < my_node_profile["n_retry"]:
                                        CANL_win_max = max(my_node_profile["CANL22_L1_min"], my_node_profile["CANL22_L1_MAX"] - my_node_profile["CANL22_fair_factor"]*(my_node_profile["n_retry"] - self.n_retry - 1))
                                    else:
                                        CANL_win_max = max(my_node_profile["CANL22_L1_min"], my_node_profile["CANL22_L1_MAX"])
                                else:
                                    CANL_win_max = max(my_node_profile["CANL22_L1_min"], my_node_profile["CANL22_L1_MAX"] - my_node_profile["CANL22_fair_factor"]*(my_node_profile["n_retry"] - self.n_retry))

                                # compute listen window
                                listen_time = self.simu.rng.integers(
                                    my_node_profile["CANL22_L1_min"],
                                    CANL_win_max + 1
                                    )*self.packet.symTime # + self.packet.rectime# + .131


                                self.MAC_listen_end_time = self.simu.env.now + listen_time
                                if self.simu.log_events:
                                    self.simu.MainLogger.info((self.pid, "lis1_start", self.simu.env.now))

                                # create and yield for a listen process that can be interrupted by the beginning of a signal 
                                self.listen_process = self.simu.env.process(self.listen(listen_time))
                                remaining_to_listen = yield self.listen_process

                                # listen can be interrupted and/or elongated if needed
                                reenter = 0
                                while remaining_to_listen != -1:

                                    self.listen_process = self.simu.env.process(self.listen(remaining_to_listen))
                                    remaining_to_listen = yield self.listen_process

                                    reenter += 1

                                self.radio_on_last = self.simu.env.now

                                # log and update variables
                                self.simu.add_listen_log((self.simu.env.now, self.MAC_listen_start_time, listen_time, self.pid, self.cycle, int(self.ch)))

                                self.listened_time = self.simu.env.now - self.MAC_listen_start_time

                            
                            #                                 
                            if (my_node_profile["BO_when_clear"] and not channel_found_busy):
                                #go transmit without listening (transit by CANL_send_RTS state)
                                self.MAC_state = constants.CANL_listen1 # only transitional
                                self.listening = True # only transitional
                                self.listened_time = 0


                ###########################################################
                # listen_1 -> NAV_state | send_RTS                        #
                ###########################################################
                #node was in CANL_listen1
                #### WE stopped listening earlier! 
                if self.MAC_state == constants.CANL_listen1 and self.listened_time != -1:
                    if self.simu.log_events:
                        self.simu.MainLogger.info((self.pid, "lis1_stop", self.MAC_listen_start_time + self.listened_time))
                    self.total_listen_time = self.total_listen_time + self.listened_time
                    self.listening = False
                    
                    #did we receive a DATA with a ValidHeader?
                    if self.I_know_it_is_Data == True:
                        #nav period is the time-on-air of the maximum data size which is returned in self.nav or max_payload_size
                        self.nav_period = airtime(self.packet.sf, self.packet.cr, self.simu.networks[self.net_id]["max_payload_size"], self.packet.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0)) - self.wait_PHY_interrupt
                        if my_node_profile["Interrupts_on_header_valid"]:# we were able to find out the data size from its header
                            self.nav_period = airtime(self.packet.sf, self.packet.cr, self.next_payload_byte, self.packet.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0)) - self.wait_PHY_interrupt #self.data_rectime

                        #will go into NAV
                        self.MAC_state = constants.CANL_NAV_state        
                        self.nav += 1        

                    elif self.I_know_it_is_RTS == True:#it did receive an RTS
                        #we process this event at the end of the listening period, normally the RTS has been received in the past
                        self.nav_period = airtime(self.packet.sf, self.packet.cr, self.next_payload_byte, self.packet.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0)) #self.data_rectime
                        if my_node_profile["CANL22_P"] != 0:
                            self.nav_period += my_node_profile["CANL22_L2"]

                        #go into NAV
                        self.MAC_state = constants.CANL_NAV_state            
                        self.nav += 1


                    elif self.I_heard_preamble or self.I_heard_activity:
                        #nav period is the time-on-air of the maximum data size which is returned in self.nav
                        self.nav_period = self.nav_period_default
                        #will go into NAV
                        self.MAC_state = constants.CANL_NAV_state                
                        self.nav += 1

                    else:# end of listening -> now send RTS (if any to send wrt scenario)

                        self.MAC_state = constants.CANL_send_RTS

                        if self.packet.dataPayloadSize > my_node_profile["CANL22_RTS_min_payload_size"]:# min is set very high in scenarios without RTS, this way this section does not apply in such cases
                            
                            if self.simu.env.now - self.radio_on_last > self.radio_wake_up_time:
                                yield self.simu.env.timeout(self.radio_wake_up_time)
                                self.radio_wake_up_number += 1


                            self.n_rts_sent = self.n_rts_sent + 1

                            self.simu.check_collision(self.pid)

                            if not (self.packet.lost and not self.simu.networks[self.net_id]["full_distances"]):
                                self.simu.insert_signal(self.pid)
                                self.packet.addTime = self.simu.env.now

                            if self.simu.log_events:
                                self.simu.MainLogger.info((self.pid, "TX_start", self.simu.env.now))

                            self.simu.add_channel_log((self.simu.env.now, self.packet.rectime, self.pid, self.cycle, int(self.ch),-1))
                            yield self.simu.env.timeout(self.packet.rectime)

                            if self.simu.log_events:
                                self.simu.MainLogger.info((self.pid, "TX_stop", self.simu.env.now))

                            self.radio_on_last = self.simu.env.now

                            yield self.simu.env.timeout(.0000001) # avoid further data being caught at exact end of a listen

                            if self.packet.lost:
                                self.packet.collided = 0
                                self.simu.nets[self.net_id].nrRTSLost += 1
                            else: # check if received and not collided somewhere
                                self.packet.collided = 1
                                for gw_id in range(len(self.packet.collided_at_GWs)):
                                    if self.packet.lost_at_GWs[gw_id] == 0 and self.packet.collided_at_GWs[gw_id] == 0:
                                        self.packet.collided = 0
                                        break

                            if self.packet.collided == 1:
                                self.simu.nets[self.net_id].nrRTSCollisions += 1
                            if self.packet.collided == 0 and not self.packet.lost:
                                self.simu.nets[self.net_id].nrRTSReceived += 1
                            if self.packet.processed == 1:
                                self.simu.nets[self.net_id].nrRTSProcessed += 1                                

                            # complete packet has been received by GW
                            # can remove it
                            self.simu.remove_signal(self.pid)

                            # reset the packet
                            self.packet.stop_interfering()
                            self.packet.collided = 0
                            self.packet.collided_at_GWs = [0 for _ in range(self.myGWs)]
                            self.packet.processed = 0
                            self.packet.lost = False       
                            self.packet.repropagate()     
                        else: # payload is too short to consider sending RTS
                            pass


                    # reinit listen params
                    self.I_heard_preamble = False
                    self.I_heard_activity = False
                    self.I_know_it_is_Data = False
                    self.I_know_it_is_RTS = False
                    self.next_payload_byte = 0
                    self.listened_time = -1

                    self.heard_frames = []
                    self.add_rx_time_opportunities = 1 # 


                ###########################################################
                # NAV_state -> want_transmit                              #
                ###########################################################
                if self.MAC_state == constants.CANL_NAV_state:
                    #we arrive at the end of the nav period
                    #so we try again from the beginning of the CANL22 procedure
                    self.MAC_state = constants.want_transmit
                    self.packet.setPacketType(constants.dataPacketType)
                    # decrement retry counter
                    self.n_retry = self.n_retry - 1   

                    if self.n_retry > 0:
                        if self.nav_period > my_node_profile["CANL22_MAX_NAV_x_chan_threshold"]:
                            if self.update_channel(max_changes = len(constants.Channels[self.modulation]["Channel_list"]), limited_changes = True, after_fail = True):
                                self.channel_just_changed = True
                                yield self.simu.env.timeout(self.radio_config_time)
                                self.radio_config_number += 1
                            else:
                                yield self.simu.env.timeout(self.nav_period)
                                self.update_available_channels()                          
                        else:
                            yield self.simu.env.timeout(self.nav_period) 
                            self.update_available_channels()                          


                ###########################################################
                # send_RTS -> listen_2 | send_DATA                        #
                ###########################################################
                if self.MAC_state == constants.CANL_send_RTS:
                    if my_node_profile["CANL22_P"] == 0:
                        #default: there is no "listening phase 2"
                        self.MAC_state = constants.CANL_send_DATA
                    else:
                        if self.simu.env.now - self.radio_on_last > self.radio_wake_up_time:
                            yield self.simu.env.timeout(self.radio_wake_up_time)
                            self.radio_wake_up_number += 1

                        #we have sent RTS, and now we go for another listening period
                        self.MAC_state = constants.CANL_listen_2   
                        self.listening = True


                        #store time at which listening period began
                        self.MAC_listen_start_time = self.simu.env.now
                        if self.simu.log_events:
                            self.simu.MainLogger.info((self.pid, "lis2_start", self.simu.env.now))

                        self.simu.start_listening(self.pid)

                        #listen period is CANL22_L2*DIFS, with DIFS = preamble duration
                        listen_duration = my_node_profile["CANL22_L2"]*self.packet.Tpream
                        self.MAC_listen_end_time = self.simu.env.now + listen_duration


                        self.listen_process = self.simu.env.process(self.listen(listen_time))
                        remaining_to_listen = yield self.listen_process

                        reenter = 0
                        while remaining_to_listen != -1:
                            self.listen_process = self.simu.env.process(self.listen(remaining_to_listen))
                            remaining_to_listen = yield self.listen_process
                            reenter += 1

                        self.simu.add_listen_log((self.simu.env.now, self.MAC_listen_start_time, listen_duration, self.pid, self.cycle, int(self.ch)))

                        self.radio_on_last = self.simu.env.now

                        self.listened_time = self.simu.env.now - self.MAC_listen_start_time



                ###########################################################
                # listen_2 -> NAV_state | send_DATA                       #
                ###########################################################
                #### WE stopped listening earlier! See self.listened_time == -1 ###
                if self.MAC_state == constants.CANL_listen2 and self.listened_time != -1:
                    if self.simu.log_events:
                        self.simu.MainLogger.info((self.pid, "lis2_stop", self.MAC_listen_start_time + self.listened_time))
                    self.total_listen_time = self.total_listen_time + self.listened_time
                    self.listening = False

                    #did we receive a DATA with a ValidHeader?
                    if self.I_know_it_is_Data == True:
                        self.nav_period = airtime(self.packet.sf, self.packet.cr, self.simu.networks[self.net_id]["max_payload_size"], self.packet.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0)) - self.wait_PHY_interrupt

                        if my_node_profile["Interrupts_on_header_valid"]:
                            self.nav_period = airtime(self.packet.sf, self.packet.cr, self.next_payload_byte, self.packet.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0)) - self.wait_PHY_interrupt #self.data_rectime

                        #will go into NAV
                        self.MAC_state = constants.CANL_NAV_state
                        self.nav += 1



                    elif self.I_know_it_is_RTS == True:#it did receive an RTS
                        self.nav_period = airtime(self.packet.sf, self.packet.cr, self.next_payload_byte, self.packet.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0)) #self.data_rectime
                        if my_node_profile["CANL22_P"] != 0:
                            self.nav_period += my_node_profile["CANL22_L2"]

                        self.MAC_state = constants.CANL_NAV_state
                        self.nav += 1
                        


                    elif self.I_heard_preamble or self.I_heard_activity:
                        #nav period is the time-on-air of the maximum data size which is returned in self.nav
                        self.nav_period = self.nav_period_default
                        #will go into NAV
                        self.MAC_state = constants.CANL_NAV_state
                        self.nav += 1

                    else:    
                        self.MAC_state = constants.CANL_send_DATA

                    # reinit listen params
                    self.I_heard_preamble = False
                    self.I_heard_activity = False
                    self.I_know_it_is_Data = False
                    self.I_know_it_is_RTS = False
                    self.next_payload_byte = 0
                    self.listened_time = -1

                    self.heard_frames = []
                    self.add_rx_time_opportunities = 1 # 

                ###########################################################
                # send_DATA -> schedule_tx                                #
                ###########################################################
                if self.MAC_state == constants.CANL_send_DATA:      

                    #change packet type to get the correct time-on-air
                    self.packet.setPacketType(constants.dataPacketType)




                    #wake up radio if not yet
                    if self.simu.env.now - self.radio_on_last > self.radio_wake_up_time:
                        yield self.simu.env.timeout(self.radio_wake_up_time)
                        self.radio_wake_up_number += 1

                    self.attempt_latency += (self.simu.env.now - self.attempt_start)
                    self.min_attempt_latency = min(self.min_attempt_latency, (self.simu.env.now - self.attempt_start))
                    self.max_attempt_latency = max(self.max_attempt_latency, (self.simu.env.now - self.attempt_start))

                    # DATA time sending and receiving
                    self.n_data_sent = self.n_data_sent + 1
                    self.n_payload_sent += self.packet.dataPayloadSize - my_node_profile["CANL22_data_hdr_size"]
                    self.simu.nets[self.net_id].nrSent += 1
                    self.total_retry += my_node_profile["n_retry"] - self.n_retry
                    self.max_retries = max(self.max_retries, my_node_profile["n_retry"] - self.n_retry)
                    self.min_retries = min(self.min_retries, my_node_profile["n_retry"] - self.n_retry)

                    self.latency = self.latency + (self.simu.env.now - self.queued_time)


                    # check impact on/from other ongoing transmissions/receptions
                    self.simu.check_collision(self.pid)

                    if not (self.packet.lost and not self.simu.networks[self.net_id]["full_distances"]):
                        self.simu.insert_signal(self.pid)
                        self.packet.addTime = self.simu.env.now

                    if self.simu.log_events:
                        self.simu.MainLogger.info((self.pid, "TX_start", self.simu.env.now))



                    self.simu.add_channel_log((self.simu.env.now, self.packet.rectime, self.pid, self.cycle, int(self.ch),-1))
                    yield self.simu.env.timeout(self.packet.rectime)
                    if self.simu.log_events:
                        self.simu.MainLogger.info((self.pid, "TX_stop", self.simu.env.now))

                    self.radio_on_last = self.simu.env.now

                        
                    if self.packet.lost:
                        self.packet.collided = 0
                        self.simu.nets[self.net_id].nrLost += 1
                        self.n_lost += 1
                        # print("node {} {}: DATA packet was lost".format(self.pid, self.simu.env.now))
                    else: # check if received and not collided somewhere
                        self.packet.collided = 1
                        for gw_id in range(len(self.packet.collided_at_GWs)):
                            if self.packet.lost_at_GWs[gw_id] == 0 and self.packet.collided_at_GWs[gw_id] == 0:
                                self.packet.collided = 0
                                break

                    if self.packet.collided == 1:
                        self.simu.nets[self.net_id].nrCollisions += 1
                        self.n_collided += 1
                        # print("node {} {}: DATA packet was collided".format(self.pid, self.simu.env.now))
                    if self.packet.collided == 0 and not self.packet.lost:
                        self.simu.nets[self.net_id].nrReceived += 1
                        # print("node {} {}: DATA packet has been correctly transmitted".format(self.pid, self.simu.env.now))
                        # current_latency = self.simu.env.now - self.want_transmit_time
                        current_latency = self.simu.env.now - self.queued_time
                        self.min_success_latency = min(self.min_success_latency, current_latency)
                        self.success_latency = self.success_latency + current_latency
                        self.n_data_success += 1
                        self.n_payload_success += self.packet.dataPayloadSize - my_node_profile["CANL22_data_hdr_size"]
                    if self.packet.processed == 1:
                        self.simu.nets[self.net_id].nrProcessed += 1                                

                    # complete packet has been received by gateway
                    # can remove it
                    self.simu.remove_signal(self.pid)
                    


                    #### reset the packet
                    self.packet.stop_interfering()
                    self.packet.collided = 0
                    self.packet.collided_at_GWs = [0 for _ in range(self.myGWs)]
                    self.packet.processed = 0
                    self.packet.lost = False
                    self.packet.repropagate()

                    self.last_tx_ch_seq_id = self.channel_sequence.index(self.ch)
                    self.transmitted_black_list.append(self.ch)
                    self.update_available_channels()
                    if self.update_channel(max_changes = len(constants.Channels[self.modulation]["Channel_list"])):
                        self.channel_just_changed = True
                        yield self.simu.env.timeout(self.radio_config_time)
                        self.radio_config_number += 1

                    self.n_retry = my_node_profile["n_retry"]
                    self.nav_period = self.nav_period_default
                    self.nav = 0
                    self.MAC_state = constants.schedule_tx


            ###////////////////////////////////////////////////////////
            # Ideal ideal_FIFO                                        /
            ###////////////////////////////////////////////////////////      
            elif my_node_profile["protocol"] == "ideal_FIFO":

                ###########################################################
                # schedule_tx -> want_transmit                            #
                # want_transmit -> send_DATA                              #
                # send_DATA -> schedule_tx                                #
                ###########################################################

                ###########################################################
                # schedule_tx -> want_transmit                            #
                ###########################################################
                if self.MAC_state == constants.schedule_tx:
                    yield self.simu.env.process(self.generate_new_frames())

                ###########################################################
                # want_transmit -> listen_1                               #
                ###########################################################
                if self.MAC_state == constants.want_transmit:# and self.packet.ptype == constants.dataPacketType:

                    #this is an initial transmit attempt
                    self.want_transmit_time = self.simu.env.now

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

                    my_transmission_date = self.simu.nets[self.net_id].ideal_latest_start[self.ch] + self.simu.nets[self.net_id].ideal_latest_time[self.ch] + 1/100000 #add 10 ns to avoid floating point error (4104.19200000001) and collisions!
                    self.MAC_state = constants.Ideal_send_DATA    
                    self.simu.nets[self.net_id].ideal_latest_time[self.ch] = airtime(self.packet.sf, self.packet.cr, self.packet.dataPayloadSize, self.packet.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0))
                    if self.want_transmit_time < my_transmission_date:
                        self.simu.nets[self.net_id].ideal_latest_start[self.ch] = my_transmission_date
                        #let's wait for the end of the previous tx:
                        yield self.simu.env.timeout(my_transmission_date - self.want_transmit_time)
                    else:
                        # self.simu.nets[self.net_id].ideal_latest_start[self.ch] = self.want_transmit_time
                        if self.simu.env.now - self.radio_on_last > self.radio_wake_up_time:
                            yield self.simu.env.timeout(self.radio_wake_up_time)
                            self.radio_wake_up_number += 1
                        self.simu.nets[self.net_id].ideal_latest_start[self.ch] = self.simu.env.now
                        

                ###########################################################
                # send_DATA -> schedule_tx                                #
                ###########################################################
                if self.MAC_state == constants.Ideal_send_DATA:        


                    # DATA time sending and receiving
                    self.n_data_sent = self.n_data_sent + 1
                    self.n_payload_sent += self.packet.dataPayloadSize
                    self.simu.nets[self.net_id].nrSent += 1
                    self.latency = self.latency + (self.simu.env.now - self.queued_time)


                    self.simu.check_collision(self.pid)

                    if not (self.packet.lost and not self.simu.networks[self.net_id]["full_distances"]):
                        self.simu.insert_signal(self.pid)
                        self.packet.addTime = self.simu.env.now

                    if self.simu.log_events:
                        self.simu.MainLogger.info((self.pid, "TX_start", self.simu.env.now))

                    self.simu.add_channel_log((self.simu.env.now, self.packet.rectime, self.pid, self.cycle, int(self.ch),-1))
                    yield self.simu.env.timeout(self.packet.rectime)
                    if self.simu.log_events:
                        self.simu.MainLogger.info((self.pid, "TX_stop", self.simu.env.now))

                    self.radio_on_last = self.simu.env.now

                        
                    if self.packet.lost:
                        self.packet.collided = 0
                        self.simu.nets[self.net_id].nrLost += 1
                        self.n_lost += 1
                        # print("node {} {}: DATA packet was lost".format(self.pid, self.simu.env.now))
                    else: # check if received and not collided somewhere
                        self.packet.collided = 1
                        for gw_id in range(len(self.packet.collided_at_GWs)):
                            if self.packet.lost_at_GWs[gw_id] == 0 and self.packet.collided_at_GWs[gw_id] == 0:
                                self.packet.collided = 0
                                break

                    if self.packet.collided == 1:
                        self.simu.nets[self.net_id].nrCollisions += 1
                        self.n_collided += 1
                    if self.packet.collided == 0 and not self.packet.lost:
                        self.simu.nets[self.net_id].nrReceived += 1
                        current_latency = self.simu.env.now - self.queued_time
                        self.min_success_latency = min(self.min_success_latency, current_latency)
                        self.success_latency = self.success_latency + current_latency
                        self.n_data_success += 1
                        self.n_payload_success += self.packet.dataPayloadSize
                    if self.packet.processed == 1:
                        self.simu.nets[self.net_id].nrProcessed += 1                                

                    # complete packet has been received by gateway
                    # can remove it
                    self.simu.remove_signal(self.pid)


                    # reset the packet
                    self.packet.stop_interfering()
                    self.packet.collided = 0
                    self.packet.collided_at_GWs = [0 for _ in range(self.myGWs)]
                    self.packet.processed = 0
                    self.packet.lost = False
                    self.packet.repropagate()

                    self.last_tx_ch_seq_id = self.channel_sequence.index(self.ch)
                    self.transmitted_black_list.append(self.ch)
                    self.update_available_channels()
                    if self.update_channel(max_changes = len(constants.Channels[self.modulation]["Channel_list"])):
                        yield self.simu.env.timeout(self.radio_config_time)
                        self.radio_config_number += 1
                    self.MAC_state = constants.schedule_tx


            ###////////////////////////////////////////////////////////                
            # CSMA APPROACHES                                         /
            # ALOHA protocol                                          /
            ###//////////////////////////////////////////////////////// 
            elif my_node_profile["protocol"] in ["CAD+Backoff", "ALOHA"]:
                # Schedule next tx
                #########################################################################
                yield self.simu.env.process(self.generate_new_frames())

                #########################################################################


                ### a new frame to transmit 
                self.want_transmit_time = self.simu.env.now
                
                # legit node? => stats
                if self.cluster != -2:
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

                
                # start with a bad feeling
                channel_found_busy = True
                
                # but with opportunities
                broken_by_lack_of_channels = False

                # retry until...
                while self.n_retry and channel_found_busy:

                    self.attempt_start = self.simu.env.now

                    if my_node_profile["protocol"] == "CAD+Backoff":
                        # perform DIFS
                        if my_node_profile["nCAD_DIFS"] > 0:
                            no_difs_has_been_found_clear_yet = True
                            while no_difs_has_been_found_clear_yet: # on (x) channels
                                for CAD_id in range(my_node_profile["nCAD_DIFS"]):
                                    self.n_CAD = self.n_CAD + 1

                                    # wake up the radio if needed
                                    if self.simu.env.now - self.radio_on_last > self.radio_wake_up_time:
                                        yield self.simu.env.timeout(self.radio_wake_up_time)
                                        self.radio_wake_up_number += 1

                                    ### Wait CAD Duration
                                    CAD_time = self.packet.symTime*self.simu.networks[self.net_id]["nCadSym"]
                                    CAD_start_time = self.simu.env.now

                                    on_air_at_CAD_start = self.start_CAD() # simulator stores transmitters at CAD start
                                    yield self.simu.env.timeout(CAD_time)
                                    channel_found_busy = self.stop_CAD(on_air_at_CAD_start) # determines CAD + or -
                                    yield self.simu.env.timeout(self.packet.symTime/2) # process CAD
                                    self.simu.add_listen_log((self.simu.env.now, CAD_start_time, CAD_time, self.pid, self.cycle, int(self.ch)))
                                    self.radio_on_last = self.simu.env.now

                                    # don't wait inter cad if busy, or if last cad
                                    if channel_found_busy or (CAD_id == (my_node_profile["nCAD_DIFS"] - 1)) :
                                        break

                                    to_wait_bw_cads = self.packet.symTime*my_node_profile["DIFS_inter_CAD_sym"]

                                    if to_wait_bw_cads > self.radio_wake_up_time:
                                        self.radio_on_last = -10000 # force re-wake_up of radio
                                        to_wait_bw_cads -= self.radio_wake_up_time

                                    yield self.simu.env.timeout(to_wait_bw_cads)                            

                                if channel_found_busy: # DIFS went busy
                                    if my_node_profile["CH_DIFS"]: # currently false <=> x == 1 tested channel, true <=> x == 8 tested channels
                                        if self.update_channel(max_changes = len(constants.Channels[self.modulation]["Channel_list"]), limited_changes = True, after_fail = True):
                                            yield self.simu.env.timeout(self.radio_config_time)
                                            self.radio_config_number += 1
                                        else: # no change possible, break anyway, but before, reset avail chan
                                            self.update_available_channels() 
                                            break # while
                                    else: # Do single difs, break anyway
                                        break # while                        
                                else: # chan found free
                                    break # while


                        else:
                            channel_found_busy = False

                    else:
                        channel_found_busy = False

                    # busy after (x)DIFS                            
                    if channel_found_busy:

                        if my_node_profile["active_BO_when_busy"]: # to wait channel being freed
                            cads_in_busy = 0
                            cads_clear = 0
                            while my_node_profile["active_BO_when_busy_max"] == -1 or cads_in_busy < my_node_profile["active_BO_when_busy_max"]:
                                self.n_CAD = self.n_CAD + 1

                                if self.simu.env.now - self.radio_on_last > self.radio_wake_up_time:
                                    yield self.simu.env.timeout(self.radio_wake_up_time)
                                    self.radio_wake_up_number += 1

                                ### Wait CAD Duration
                                CAD_time = self.packet.symTime*self.simu.networks[self.net_id]["nCadSym"]
                                CAD_start_time = self.simu.env.now


                                on_air_at_CAD_start = self.start_CAD() # simulator stores transmitters at CAD start
                                yield self.simu.env.timeout(CAD_time)
                                channel_found_busy = self.stop_CAD(on_air_at_CAD_start) # determines CAD + or -
                                yield self.simu.env.timeout(self.packet.symTime/2) # process CAD
                                self.simu.add_listen_log((self.simu.env.now, CAD_start_time, CAD_time, self.pid, self.cycle, int(self.ch)))
                                self.radio_on_last = self.simu.env.now                                

                                if channel_found_busy:
                                    self.I_heard_activity = True
                                    cads_in_busy += 1
                                    cads_clear = 0
                                else:
                                    cads_clear += 1

                                    if cads_clear == my_node_profile["nCAD_DIFS"]: 
                                        break
                                    

                                to_wait_bw_cads = self.packet.symTime*my_node_profile["active_BO_when_busy_iCs"]

                                if to_wait_bw_cads > self.radio_wake_up_time:
                                    self.radio_on_last = -10000 # force re-wake_up of radio
                                    to_wait_bw_cads -= self.radio_wake_up_time

                                yield self.simu.env.timeout(to_wait_bw_cads)


                        if channel_found_busy:

                            self.n_retry = self.n_retry - 1


                            # if passive_BO_when_busy and (self.n_retry > 1 or (not abort_after_retries)):# don't BO if you're gonna abort anyway
                            if my_node_profile["passive_BO_when_busy"] and (self.n_retry > 0):# don't do busy BO at the end, do ultimate BO
                                #here we just delay by a random back off timer to retry again
                                #random backoff [Wbusy_min_airtime_B, 2**Wbusy_BE]
                                self.backoff = airtime(self.packet.sf, self.packet.cr, my_node_profile["Wbusy_min_airtime_B"], self.packet.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0)) + self.packet.symTime*self.simu.rng.integers(0, 2**self.Wbusy_BE + 1)
                                if my_node_profile["Wbusy_exp_backoff"]:
                                    if self.Wbusy_BE < my_node_profile["Wbusy_maxBE"]:
                                        self.Wbusy_BE = self.Wbusy_BE + 1
                                if my_node_profile["Wbusy_add_max_toa"]:            
                                    yield self.simu.env.timeout(airtime(self.packet.sf, self.packet.cr, self.simu.networks[self.net_id]["max_payload_size"], self.packet.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0)) + self.backoff)
                                else:
                                    yield self.simu.env.timeout(self.backoff)
                                self.backoff = 0

                    
                    # DIFS says "clear"
                    if not channel_found_busy:
                        if my_node_profile["protocol"] == "CAD+Backoff" and my_node_profile["BO_when_clear"]:
                            
                            if my_node_profile["active_clear_BO"]: #perform CADs during BO
                                if self.remainder == -1:

                                    # self.backoff = rng.integers(Wclear_min_sym, 2**self.Wclear_BE + 1)                        
                                    self.remainder = self.simu.rng.integers(my_node_profile["Wclear_min_CAD"], my_node_profile["Wclear_max_CAD"] + 1)                        

                                for CAD_id in range(self.remainder):
                                    self.n_CAD = self.n_CAD + 1

                                    if self.simu.env.now - self.radio_on_last > self.radio_wake_up_time:
                                        yield self.simu.env.timeout(self.radio_wake_up_time)
                                        self.radio_wake_up_number += 1

                                    ### Wait CAD Duration
                                    CAD_time = self.packet.symTime*self.simu.networks[self.net_id]["nCadSym"]
                                    CAD_start_time = self.simu.env.now


                                    on_air_at_CAD_start = self.start_CAD() # simulator stores transmitters at CAD start
                                    yield self.simu.env.timeout(CAD_time)
                                    channel_found_busy = self.stop_CAD(on_air_at_CAD_start) # determines CAD + or -
                                    yield self.simu.env.timeout(self.packet.symTime/2) # process CAD
                                    self.simu.add_listen_log((self.simu.env.now, CAD_start_time, CAD_time, self.pid, self.cycle, int(self.ch)))
                                    self.radio_on_last = self.simu.env.now                                

                                    if channel_found_busy:
                                        self.n_retry = self.n_retry - 1
                                        break

                                    if (CAD_id == (self.remainder - 1)) : # all clear, go transmit
                                        break

                                    to_wait_bw_cads = self.packet.symTime*my_node_profile["BO_inter_CAD_sym"]

                                    if to_wait_bw_cads > self.radio_wake_up_time:
                                        self.radio_on_last = -10000 # force re-wake_up of radio
                                        to_wait_bw_cads -= self.radio_wake_up_time

                                    yield self.simu.env.timeout(to_wait_bw_cads)

                                if channel_found_busy:
                                    # self.remainder = self.remainder -1 - CAD_id
                                    self.remainder = self.remainder - CAD_id # last was busy, we need remainder clears to exit BO
                                    # self.remainder = max(1, self.remainder -1 - CAD_id)
                                else: # BO is successfully finished
                                    self.remainder = -1
                            else:
                                self.backoff = self.packet.symTime*self.simu.rng.integers(my_node_profile["Wclear_min_sym"], my_node_profile["Wclear_max_sym"] + 1)
                                yield self.simu.env.timeout(self.backoff)
                                self.backoff = 0


                                if my_node_profile["nCAD_DIFS"] > 0:
                                    for CAD_id in range(my_node_profile["nCAD_DIFS"]):
                                        self.n_CAD = self.n_CAD + 1

                                        if self.simu.env.now - self.radio_on_last > self.radio_wake_up_time:
                                            yield self.simu.env.timeout(self.radio_wake_up_time)
                                            self.radio_wake_up_number += 1

                                        ### Wait CAD Duration
                                        CAD_time = self.packet.symTime*self.simu.networks[self.net_id]["nCadSym"]
                                        CAD_start_time = self.simu.env.now

                                        on_air_at_CAD_start = self.start_CAD() # simulator stores transmitters at CAD start
                                        yield self.simu.env.timeout(CAD_time)
                                        channel_found_busy = self.stop_CAD(on_air_at_CAD_start) # determines CAD + or -
                                        yield self.simu.env.timeout(self.packet.symTime/2) # process CAD
                                        self.simu.add_listen_log((self.simu.env.now, CAD_start_time, CAD_time, self.pid, self.cycle, int(self.ch)))
                                        self.radio_on_last = self.simu.env.now                                

                                        if channel_found_busy:
                                            self.n_retry = self.n_retry - 1
                                            break

                                        if (CAD_id == (my_node_profile["nCAD_DIFS"]-1)) :
                                            break
                                        
                                        to_wait_bw_cads = self.packet.symTime*my_node_profile["DIFS_inter_CAD_sym"]

                                        if to_wait_bw_cads > self.radio_wake_up_time:
                                            self.radio_on_last = -10000 # force re-wake_up of radio
                                            to_wait_bw_cads -= self.radio_wake_up_time

                                        yield self.simu.env.timeout(to_wait_bw_cads)

                                else:
                                    channel_found_busy = False

                    self.attempt_latency += (self.simu.env.now - self.attempt_start)
                    self.min_attempt_latency = min(self.min_attempt_latency, (self.simu.env.now - self.attempt_start))
                    self.max_attempt_latency = max(self.max_attempt_latency, (self.simu.env.now - self.attempt_start))

                    if channel_found_busy:
                        if my_node_profile["CA_Change_Channel"]:
                            if self.n_retry > 0:
                                if self.update_channel(max_changes = len(constants.Channels[self.modulation]["Channel_list"]), limited_changes = True, after_fail = True):
                                    yield self.simu.env.timeout(self.radio_config_time)
                                    self.radio_config_number += 1
                                else:
                                    broken_by_lack_of_channels = True
                                    break # go aloha
                                            


                # end of big while
                self.update_available_channels() # reset list of channels
                self.Wbusy_BE = my_node_profile["Wbusy_BE"] if "Wbusy_BE" in my_node_profile else -1
                # self.backoff = 0

                if "n_retry" in my_node_profile:
                    self.total_retry += my_node_profile["n_retry"] - self.n_retry
                    self.max_retries = max(self.max_retries, my_node_profile["n_retry"] - self.n_retry)
                    self.min_retries = min(self.min_retries, my_node_profile["n_retry"] - self.n_retry)


                self.remainder = -1
                

                # we have exited the main while without transmitting => abort or ALOHA
                if (self.n_retry == 0 or broken_by_lack_of_channels ) and my_node_profile["abort_after_retries"]:
                    self.n_aborted = self.n_aborted + 1
                else:
                    if (self.n_retry == 0 or broken_by_lack_of_channels ): # aloha after retries
                        self.backoff = self.packet.symTime*self.simu.rng.integers(my_node_profile["Wultim_min_sym"], my_node_profile["Wultim_max_sym"] + 1)
                        yield self.simu.env.timeout(self.backoff)   
                        self.backoff = 0
                        self.kamikaze += 1


                    if self.simu.env.now - self.radio_on_last > self.radio_wake_up_time:
                        yield self.simu.env.timeout(self.radio_wake_up_time)
                        self.radio_wake_up_number += 1

                    self.n_data_sent = self.n_data_sent + 1
                    self.n_payload_sent += self.packet.dataPayloadSize
                    if self.cluster != -2:
                        self.simu.nets[self.net_id].nrSent += 1
                    self.latency = self.latency + (self.simu.env.now - self.queued_time)

                    self.simu.check_collision(self.pid)

                    if not (self.packet.lost and not self.simu.networks[self.net_id]["full_distances"]):
                        self.simu.insert_signal(self.pid)
                        self.packet.addTime = self.simu.env.now

                    if self.simu.log_events:
                        self.simu.MainLogger.info((self.pid, "TX_start", self.simu.env.now))

                    self.simu.add_channel_log((self.simu.env.now, self.packet.rectime, self.pid, self.cycle, int(self.ch),-1))
                    yield self.simu.env.timeout(self.packet.rectime)
                    if self.simu.log_events:
                        self.simu.MainLogger.info((self.pid, "TX_stop", self.simu.env.now))

                    self.radio_on_last = self.simu.env.now
            
                    if self.packet.lost:
                        self.packet.collided = 0
                        if self.cluster != -2:
                            self.simu.nets[self.net_id].nrLost += 1
                        self.n_lost += 1
                    else: # check if received and not collided somewhere
                        self.packet.collided = 1
                        for gw_id in range(len(self.packet.collided_at_GWs)):
                            if self.packet.lost_at_GWs[gw_id] == 0 and self.packet.collided_at_GWs[gw_id] == 0:
                                self.packet.collided = 0
                                break
                    if self.packet.collided == 1:
                        if self.cluster != -2:
                            self.simu.nets[self.net_id].nrCollisions += 1
                        self.n_collided += 1
                    if self.packet.collided == 0 and not self.packet.lost:
                        if self.cluster != -2:
                            self.simu.nets[self.net_id].nrReceived += 1

                        current_latency = self.simu.env.now - self.queued_time
                        self.min_success_latency = min(self.min_success_latency, current_latency)
                        self.success_latency = self.success_latency + current_latency
                        self.n_data_success += 1
                        self.n_payload_success += self.packet.dataPayloadSize
                    if self.packet.processed == 1:
                        if self.cluster != -2:
                            self.simu.nets[self.net_id].nrProcessed += 1                                
                

                    # complete packet has been received by base station
                    # can remove it
                    self.simu.remove_signal(self.pid)


                    # reset the packet
                    self.packet.stop_interfering()
                    self.packet.collided = 0
                    self.packet.collided_at_GWs = [0 for _ in range(self.myGWs)]
                    self.packet.processed = 0
                    self.packet.lost = False
                    self.packet.repropagate()

                    self.last_tx_ch_seq_id = self.channel_sequence.index(self.ch)
                    self.transmitted_black_list.append(self.ch)
                    self.update_available_channels()
                    if self.update_channel(max_changes = len(constants.Channels[self.modulation]["Channel_list"])):
                        yield self.simu.env.timeout(self.radio_config_time)
                        self.radio_config_number += 1

                self.n_retry = my_node_profile["n_retry"] if "n_retry" in my_node_profile else -1


            if self.simu.check_all_networks_finished():
                self.simu.endSim = self.simu.env.now
                ## ! ##
                return 



#
# this class creates a LoRa-CSS packet (associated with a node)
# it also sets all its parameters
#
class LoRa_CSS_Packet(Generic_Elements.Packet):
    def __init__(self, simu, net_id, node_id, plen, distance_GWs, node_profile):
        super().__init__(simu, net_id, node_id, plen, distance_GWs, node_profile)

        # (obsolete) randomize configuration values
        if self.simu.networks[self.net_id]["lora24GHz"]:
            self.sf = self.simu.rng.integers(5, 13)
            self.bw = self.simu.rng.choice([203.125, 406.250, 812.5, 1625])
        else:    
            self.sf = self.simu.rng.integers(6, 13)
            self.bw = self.simu.rng.choice([125, 250, 500])
        self.cr = self.simu.rng.integers(1, 5)


        #2026 October
        self.sf = self.simu.networks[self.net_id]["net_SF"]
        self.bw = self.simu.networks[self.net_id]["net_BW"]
        self.cr = self.simu.networks[self.net_id]["net_CR"]


            
        # this is by default not applied: adjust tx power according to expected path-loss 
        # a la np-CECADA (Np-CECADA: Enhancing Ubiquitous Connectivity of LoRa Networks, ” in MASS’2021)
        if self.simu.networks[self.net_id]["transmit_at_adjusted_power"]: 

            pred_noise_dB = 0
            if self.simu.radio_params["gaussian_noise"] or self.simu.radio_params["correlated_noise"]: # consider Gaussian in this prediction
                pred_noise_dB = 2*constants.noise_mu_dB

            pred_rayleigh_dB = 0
            if self.simu.radio_params["rayleigh_fading"]:
                pred_rayleigh_dB = rayleigh_mean_dB


            pred_local_obstruction_dB = 0 
            if self.simu.radio_params["locally_obstructed"]:
                pred_local_obstruction_dB = 2*constants.obstruction_dB_GW

            pred_buildings_atten_dB = 0 
            if self.simu.radio_params["buildings_attenuated"]:
                pred_buildings_atten_dB = self.simu.radio_params["buildings_km"]*distance_GWs/1000*constants.loss_per_building_GW

            # matrix of every path loss
            for txp in constants.power_levels:
                rss_GWs = np.minimum(txp, txp + constants.GL_GW - constants.Lpld0 - 10*self.simu.networks[self.net_id]["gamma_GW"]*np.log10(distance_GWs/constants.d0) - pred_rayleigh_dB - pred_local_obstruction_dB - pred_buildings_atten_dB)

                heard_at_gateways = (rss_GWs - pred_noise_dB > self.simu.get_gws_sensitivities(self.get_tx_config()))

                if heard_at_gateways[self.myGWs_ids].any(): # one of my gws would receive it, enough
                    break
                if txp == self.simu.networks[self.net_id]["Ptx"]:
                    break

            self.txpow = txp

        else:
            self.txpow = self.simu.networks[self.net_id]["Ptx"]



        self.symTime = (2.0**self.sf)/self.bw


        if self.simu.networks[self.net_id]["lora24GHz"]:
            Npream = 12     
            if self.sf < 7:
                self.Tpream = (Npream + 6.25)*self.symTime
            else:
                self.Tpream = (Npream + 4.25)*self.symTime                 
        else:        
            Npream = 8     # number of preamble symbol (12.25     from Utz paper) 
            self.Tpream = (Npream + 4.25)*self.symTime        

        #this is init only, will be updated by self.setDataPayloadSize() and setPacketType()
        self.rectime = airtime(self.sf, self.cr, self.pl, self.bw)

        # min # of preamble symbols that should be heard to enable detection
        self.Tmin_preamb_heard = self.symTime *  self.simu.networks[self.net_id]["detection_min_preamb_symb"]


    # retrieve tx config for this lora packet
    def get_tx_config(self):
        return({"sf":self.sf, "bw":self.bw})


    #
    # give payload size according to distribution
    def setDataPayloadSize(self):
        if self.simu.networks[self.net_id]["variablePayloadSize"]:
            if self.simu.networks[self.net_id]["normalPayloadSize"]:
                self.dataPayloadSize = self.simu.rng.normal(self.simu.networks[self.net_id]["normaldist_mean_payload_size"], self.simu.networks[self.net_id]["normaldist_sigma_payload_size"], 1).astype('int').clip(self.simu.networks[self.net_id]["dist_min_payload_size"], self.simu.networks[self.net_id]["dist_max_payload_size"])[0]
            else: # uniform
                self.dataPayloadSize = self.simu.rng.integers(self.simu.networks[self.net_id]["dist_min_payload_size"], self.simu.networks[self.net_id]["dist_max_payload_size"] + 1)

            if self.simu.networks[self.net_id]["profiles"][self.profile]["protocol"] == "CANL22":
            # if CANL22: #depends on scenario, data length included in header or in data... 
                self.dataPayloadSize += self.simu.networks[self.net_id]["profiles"][self.profile]["CANL22_data_hdr_size"]


    #
    ## change packet type and size accordingly
    def setPacketType(self, ptype):
        self.ptype = ptype
        
        if ptype == constants.rtsPacketType:
            self.pl = 5
            # if CANL22:
            if self.simu.networks[self.net_id]["profiles"][self.profile]["protocol"] == "CANL22":
                self.pl = self.simu.networks[self.net_id]["profiles"][self.profile]["CANL22_RTS_hdr_size"] + 1
                self.rectime = airtime(self.sf, self.cr, self.pl, self.bw, explicit = (self.simu.networks[self.net_id]["profiles"][self.profile]["CANL22_RTS_PHY_HDR"] == 0))         
            else:
                self.rectime = airtime(self.sf, self.cr, self.pl, self.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0))         
        else:
            self.pl = self.dataPayloadSize # self.data_len
            # if nodes[self.pid].modulation == "LoRa-CSS":
            self.rectime = airtime(self.sf, self.cr, self.pl, self.bw, explicit = (self.simu.networks[self.net_id]["LoRa_PHY_HDR"] == 0))

    # remove my interference when I leave
    def stop_interfering(self):
        self.simu.stop_interfering(self.net_id, self.node_id)




#
# the LR-FHSS GW object definition
#
class LoRa_CSS_Gateway(Generic_Elements.Gateway): 
    def __init__(self, simu, net_id, gw_id, gw_x, gw_y):
        super().__init__(simu, net_id, gw_id, gw_x, gw_y)


    def get_rx_sensitivity(self, properties_of_signal_being_received):
        if not ("sf" in properties_of_signal_being_received): 
            # FHSS fragment is sensed as a sf12 bw 125, TODO: differentiate
            properties_of_signal_being_received["sf"] = 12
            properties_of_signal_being_received["bw"] = 125
        return get_sensitivity(properties_of_signal_being_received["sf"], properties_of_signal_being_received["bw"], receiver_type = "GW", GW_sensitivity_gain = self.simu.networks[self.net_id]["GW_sensitivity_gain"])










#####################
# CAD definitions #
####################


# compute the CAD prob of success (true positive) 
def get_CAD_prob(distance, txpower, net_id, in_same_cluster = False):

    if in_same_cluster:
        return(CAD_distrib_funcs[txpower][net_id][1][round(distance)-1])
    return(CAD_distrib_funcs[txpower][net_id][0][round(distance)-1])



# pre-simulate the CAD proba of success as the smoothed reception rate of 100 packets at each meter and interpolated
# depends on propa variables and lora params
def prepare_distrib_func(loss_per_building, aper, maxDist, radio_params, networks, rng):
    # global CAD_distrib_funcs
    distrib_funcs = {}
    for txp in constants.power_levels:
        my_matshape = (max(2000, int(1.1*maxDist)), 100)

        distances_OL = np.linspace(1, my_matshape[0], my_matshape[0])
        distances = np.vstack([distances_OL]*my_matshape[1]).T

        noise = 0
        if radio_params["gaussian_noise"] or radio_params["correlated_noise"]: # consider gaussian noise for CADs
            noise = np.clip(rng.normal(constants.noise_mu_dB, constants.noise_sigma_dB, my_matshape), 0, 2*constants.noise_mu_dB)

        rayleigh = 0
        if radio_params["rayleigh_fading"]:
            # rayleigh = rng.rayleigh(scale = np.sqrt(2 / np.pi)*rayleigh_mean_dB, size = my_matshape) - rayleigh_mean_dB
            rayleigh = -10*np.log10(rng.exponential(scale = 10**(.1*radio_params["rayleigh_mean_dB"]), size = my_matshape))


        apertures = np.clip(rng.normal(aper, aper, my_matshape), 0, 2*aper)
        buildings_atten = radio_params["buildings_km"]*distances/1000*loss_per_building + apertures

        distrib_funcs[txp] = {}
        for net_id in range(len(networks)):

            net = networks[net_id]

            # Path loss exponents EDED for CADs, regardless of clustering
            CAD_gamma_EDs = np.zeros(my_matshape)
            if net["CAD_normal_gamma_ED"]:
                CAD_gamma_EDs = rng.normal(net["gamma_ED"] + net["CAD_gamma_ED_delta"], net["CAD_sigma_gamma_ED"], CAD_gamma_EDs.shape)
            else:
                CAD_gamma_EDs += net["gamma_ED"] + net["CAD_gamma_ED_delta"]


            distrib_funcs[txp][net_id] = {}

            if not (net["modulation"] == "LoRa-CSS"):
                continue
            for intra_cluster_scenario in [0, 1]:

                Lpl = constants.Lpld0 + 10*(CAD_gamma_EDs - net["intra_cluster_gamma_gain"]*intra_cluster_scenario)*np.log10(distances/constants.d0)

                CAD_Prx = np.clip(-1000, txp, txp + constants.GL - Lpl - noise - rayleigh - buildings_atten)

                # 
                predicted_PDRs = np.count_nonzero((CAD_Prx > get_sensitivity(net["net_SF"], net["net_BW"], receiver_type = "DEVICE")), axis = 1)

                # >>> scipy.version.version : '1.6.2' requires window_length to be odd, not version 1.12.0 anymore
                # window_length = max(50, int(2.2*maxDist/20))
                window_length = max(50, int(1.1*maxDist/20))
                if window_length%2 == 0:
                    window_length += 1

                y = savgol_filter(predicted_PDRs, window_length, 2)

                if max(y) < 100:
                    y = y/max(y)*100.1
                
                val_id = 0
                while y[val_id] < 100:
                    y[val_id] = 101
                    val_id += 1

                y = np.clip(y, a_min = 0, a_max = 100)

                distrib_funcs[txp][net_id][intra_cluster_scenario] = y

    # return distrib_funcs => update global var
    for txp in distrib_funcs:
        CAD_distrib_funcs[txp] = distrib_funcs[txp]




def prepare_CAD_model(maxDist, radio_params, networks, rng):


    prepare_distrib_func(constants.loss_per_building_CAD, constants.obstruction_dB_CAD, maxDist, radio_params, networks, rng)

    # CAD_distrib_funcs = prepare_distrib_func(constants.loss_per_building_CAD, constants.obstruction_dB_CAD, 12, 125, maxDist, networks)
    # prepared for 1 sf 1 bw for each net (net_SF)
    # in future scenarios with multi modifiable sf/ multi bw => # TODO: prepare a distrib_func matrix with all sf/bw possibilities


