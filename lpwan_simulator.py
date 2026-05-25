# -*- coding: utf-8 -*-
######## Core module of the LPWAN-Sim Simulator ##############################################
######## https://github.com/Guillaumegaillard/LPWAN-Sim ######################################
######## 2026-05-05 ##########################################################################
######## License MIT: ########################################################################
######## https://github.com/Guillaumegaillard/LPWAN-Sim/blob/main/LICENSE ####################
######## Inspired from LoRaSim https://mcbor.github.io/lorasim/ ##############################



import simpy
import numpy as np
rng = np.random.default_rng()

import sys
import os
import pickle
import logging

import constants 
import config_networks
import Radio_environment 

# techno-specific modules
import LoRa_CSS
import LR_FHSS
import Mioty


#####################################################
if not os.path.exists('results'):
    os.makedirs('results')



#############################################
# Simulation Core Technical Things #
#############################################

# turn on/off print
# disabling printing to stdout should make simulation faster
print_sim = True
print_sim = False
stdout_print_target = sys.stdout

#disable printing to stdout to make simulation much faster
if print_sim == False:
    f = open('/dev/null', 'w')
    sys.stdout = f    

# sys.stdout = stdout_print_target



################################
##### LOG FUNCTIONS
################################

### If log_events is activated, the call to this function will initiate the logs process
def load_main_logger(start_time):
    
    ###########LOGGING MAIN EVENTS
    MainLogger = logging.getLogger('Main_Logger')
    MainLogger.setLevel(logging.INFO)
    # create file handler which logs even debug messages
    mfh = logging.FileHandler('{0}.log'.format(start_time), mode='w')
    mfh.setLevel(logging.INFO)
    #console
    mch = logging.StreamHandler()
    mch.setLevel(logging.CRITICAL)
    # create formatter and add it to the handlers
    mformatter = logging.Formatter('%(asctime)s %(message)s')
    mfh.setFormatter(mformatter)
    mch.setFormatter(mformatter)
    
    close_logger_handlers(MainLogger)
    
    # add the handlers to logger
    MainLogger.addHandler(mfh)
    MainLogger.addHandler(mch)
    
    return (MainLogger)

### shut down logging when simulation stops
def close_logger_handlers(logger_obj):
    handlers = logger_obj.handlers[:]
    for handler in handlers:
        handler.close()
        logger_obj.removeHandler(handler)




#####################################################
# Simulation Core #
#############################################

nodes = [] # list of node objects
gateways = [] # list of gateways
active_nodes = [] # list of each network's amount of active nodes (a network topology may have more nodes defined than its nrNodes)

flat_ndcs = [] # indices of nodes in their net
flat_net_ids = [] # net indices of nodes

OnAir = [] # list of (sub-)frames on air
# TODO: make OnAir multi-channel (with subchannnels?) (overlap)? i.e. change simu's structure (here any signal is confronted to all concomitant others, whatever band, modulation, tx/rx params... inefficient?)

channel_log = [] # chronology of all transmissions 
listen_log = [] # chronology of all device listening phases 

######### Simu Stats Vars ####################
powerCaptures = [] # TODO: clarify/rename

# get the unique identifier (uid) of a device in the simu, from its network id and its id in that network
def uid(net_id, node_id):
    return(sum(active_nodes[:net_id]) + node_id)

# get the slice of uids for the devices of a given network
def slice_net_nodes(net_id):
    first_id = sum(active_nodes[:net_id])
    pop_net = active_nodes[net_id]
    return(slice(first_id,first_id+pop_net))

# an abstract class: live Network objects gathers each network's simulation variables' increments  
class Network:
    def __init__(self):
        # self.name = "network"
        pass

# the main class for this simulation: controls/runs the simulation scenario
class Simulator:
    def __init__(self,env):
        self.env = env # env is the Simpy environment
        self.endSim = 0

        #### variables updated with the transmissions
        self.nrLost = 0
        self.nrCollisions = 0
        self.nrCaptures = 0
        self.nrReceived = 0
        self.nrProcessed = 0
        self.nrSent = 0
        self.nrScheduled = 0

        self.nrRTSLost = 0
        self.nrRTSCollisions = 0
        self.nrRTSReceived = 0
        self.nrRTSProcessed = 0
                        
        self.n_transmit = 0
        self.inter_transmit_time = 0
        self.last_transmit_time = 0
        self.lastDisplayTime = -1
        self.max_inter_transmit_time = 40

        self.nrFragmentLosses = 0
        self.nrFragmentCollisions = 0
        self.nrFragmentsSent = 0


        #### parameters for each whole network (tech+proto(s))
        self.networks = []
        self.nets = [] # live networks objects

        self.maxDist = 0
        self.xmax = 0
        self.ymax = 0

        ######### GATEWAYS
        #TODO: add sectors (band, modulation) available/sense-able on each GW and (simultaneous) capacity of each GW
        self.gateways = {"xs":[],"ys":[],"net_ids":[]}

    # generate net objects that will gather net-level stats during simulation 
    def init_network_variables(self):
        for net in self.networks:

            net_obj = Network()

            #### variables updated with the transmissions
            net_obj.nrLost = 0
            net_obj.nrCollisions = 0
            net_obj.nrCaptures = 0
            net_obj.nrReceived = 0
            net_obj.nrProcessed = 0
            net_obj.nrSent = 0
            net_obj.nrScheduled = 0

            net_obj.nrRTSLost = 0
            net_obj.nrRTSCollisions = 0
            net_obj.nrRTSReceived = 0
            net_obj.nrRTSProcessed = 0
                            
            net_obj.n_transmit = 0
            net_obj.inter_transmit_time = 0
            net_obj.last_transmit_time = 0
            net_obj.lastDisplayTime = -1
            net_obj.max_inter_transmit_time = 40

            net_obj.nrFragmentLosses = 0
            net_obj.nrFragmentCollisions = 0
            net_obj.nrFragmentsSent = 0

            # ###########
            # net_obj.powerCaptures = []

            ######### Ideal Mechanism/protocol Vars ####################
            net_obj.ideal_latest_start = [0]*len(constants.Channels[net["modulation"]]["Channel_list"])
            net_obj.ideal_latest_time = [0]*len(constants.Channels[net["modulation"]]["Channel_list"])    


            self.nets.append(net_obj)

    # place a frame/fragment on OnAir, the shared list of currently transmitted signals.
    # OnAir is actually a list of pair id (net_id,dev_id) (a device transmits at most 1 signal at a time)
    def insert_signal(self,pair_id):
        OnAir.append(pair_id)

    # remove from onair at the end of transmission
    def remove_signal(self,pair_id):
        if pair_id in OnAir:
            OnAir.remove(pair_id)        

    # add transmissions info (start, length, channel, ids, seqnum, etc) to a shared channel log list
    def add_channel_log(self, data):
        channel_log.append(data)

    # add listening/rx info to a shared listen log list
    def add_listen_log(self, data):
        listen_log.append(data)        

    # helper to retrieve packet type (data, RTS) for old CANL approach
    def get_ptype(self,pair_id):
        hid = uid(*pair_id)
        return(nodes[hid].packet.ptype)

    # helper to retrieve current transmission's (frame/fragment) time on air 
    def get_rectime(self,pair_id):
        hid = uid(*pair_id)
        return(nodes[hid].get_ongoing_duration())

    # helper to retrieve current transmission's full payload size (in bytes) 
    def get_dataPayloadSize(self,pair_id):
        hid = uid(*pair_id)
        return(nodes[hid].packet.dataPayloadSize)

    # helper to retrieve current transmission's (frame/fragment) start time 
    def get_addTime(self,pair_id):
        hid = uid(*pair_id)
        return(nodes[hid].get_ongoing_start())

    # helper to retrieve a device's transmission power
    def get_txpow(self,pair_id):
        hid = uid(*pair_id)
        return(nodes[hid].packet.txpow)

    # retrieve a list of (partially) overlapping transmissions
    # TODO: this function is used in LoRa-CSS CAD evaluation to determine if a transmission occurs both at beginning and end of a CAD. => refine (there?) using proportion of both time and frequency overlaps, to weigh impact 
    def get_transmitters_in_band(self,pair_id):
        return [transmitter for transmitter in OnAir if self.frequencyCollision(transmitter, pair_id)]
        # return [transmitter for transmitter in OnAir if nodes[uid(*transmitter)].ch == nodes[uid(*pair_id)].ch]

    # provides current OnAir list to tech-specific start_listening() func.
    def start_listening(self,pair_id):
        hid = uid(*pair_id)
        nodes[hid].start_listening(OnAir)

    # set or unset a receiver's phase lock over a transmission: multiple receivers can lock on on the same transmission, but each receiver can only lock on 1 transmission. 
    def set_local_receiving(self, tx_id, rx_id, unset = False):
        tx = uid(*tx_id)
        rx = uid(*rx_id)

        if unset:
            nodes[tx].packet.receiving[rx] = False
        else:
            nodes[tx].packet.receiving[rx] = True
            nodes[tx].packet.receiving_start[rx] = self.env.now


    # provides current OnAir list to tech-specific check_collision() func.
    def check_collision(self,pair_id):
        hid = uid(*pair_id)
        nodes[hid].check_collision(OnAir)

    # simulation continues (nodes continue transmitting) until all the networks reach their transmission target 
    def check_all_networks_finished(self):
        # for net in self.networks:
        for net_id in range(len(self.networks)):
            if self.nets[net_id].nrScheduled < self.networks[net_id]["targetSchedPacket"]:
                return False
        return True

    # get all distances to a given device
    def get_dist_vector(self, net_id, node_id):
        return(distance_matrix[uid(net_id, node_id)])

    # get an individual distance value
    def get_dist(self, pid1, pid2):
        return(distance_matrix[uid(*pid1)][uid(*pid2)])

    # remove the interference when transmission ends
    def stop_interfering(self, net_id, node_id):
        finishing_uid = uid(net_id, node_id)
        # if somebody else is transmitting
        if len(OnAir)>0:
            for other_id in OnAir:
                other_uid = uid(*other_id)
                # if frequencyCollision(nodes[uid(net_id, node_id)].packet, nodes[other].packet): 
                if self.frequencyCollision((net_id, node_id),other_id) and self.modulationCollision((net_id, node_id),other_id):
                    # remove interf caused by finishing on all signals onair on the same freq at all simu's GWs
                    nodes[other_uid].packet.interf_mw_GWs -= 10**(.1*nodes[finishing_uid].packet.rss_GWs)

                    # remove interf on listening devices 
                    for node_uid in range(len(nodes)):
                        node = nodes[node_uid]
                        if node.listening:
                            if self.frequencyCollision(node.pid,(net_id, node_id)) and self.modulationCollision(node.pid,(net_id, node_id)):
                                this_mw_this_dev = 10**(.1*(nodes[finishing_uid].packet.rx_array[node_uid]))
                                nodes[other_uid].packet.interf_mw[node_uid] -= this_mw_this_dev


        # NB: 
        #     for gws, two signals interfere iif freq and modulation collision
        #     for listening devs, two signals interfere iif freq and modulation collision and device is listening onto this freq&modulation


    # studies the collision, capture, interference conditions on listening devices, when starting a new transmission 
    def check_collisions_local(self, tx_id, collided_at_some_GW=0):
        txid = uid(*tx_id)
        net_id = tx_id[0]

        # old "everywhere is the same as GW" case 
        if not self.networks[net_id]["full_collision"]:
            
            if collided_at_some_GW:
                return collided_at_some_GW
            # not collided at GW, packet is alone on air 


            #without full_collision, assume that since the gateways received a packet
            #then all other nodes in listening state should also have receive it
            for node_uid in range(len(nodes)):
                node = nodes[node_uid]
                if node.listening:

                    # mark the packet in list of receptions during listening 
                    # depending on the scenario in terms of headers, the data payload size could be known, unknown or mistaken by the receiver
                    # dataPayloadSize_in_RTS is used only in a scenario with RTS 
                    # packet could be partially interfered by/interfering other packets on air - checked later wrt times & powers 
                    node.heard_frames.append({
                        "id":len(node.heard_frames),
                        "node_id":nodes[txid].pid,
                        "toa":self.get_rectime(tx_id),
                        # "toa":nodes[txid].packet.rectime,
                        "is_RTS":nodes[txid].packet.ptype == constants.rtsPacketType,
                        "dataPayloadSize_in_RTS":rng.integers(0,self.networks[net_id]["max_payload_size"]+1) if nodes[txid].packet.dataPayloadSize==(self.networks[net_id]["profiles"][node.profile]["CANL22_RTS_hdr_size"]+1) else nodes[txid].packet.dataPayloadSize, # if Data of RTS size, random size
                        "dataPayloadSize_in_EH":nodes[txid].packet.dataPayloadSize, # in explicit header
                        "start":self.env.now,
                        "interfering":[(i,nodes[uid(*i)].packet.ptype) for i in OnAir],
                        "interfered_by":[(i,nodes[uid(*i)].packet.ptype) for i in OnAir],
                        })
                    if self.log_events:
                        self.MainLogger.info((node.pid,"rx",nodes[txid].pid,self.env.now))

        else:
            # # general case 
            # for each node listening
            #     if heard
            #         for other on air 
            #             if collide?Capture
            #                 adjust
            #         record impact/or not


            for node_uid in range(len(nodes)):
                node = nodes[node_uid]
                if node.listening:

                    locally_collided=False
                    previous_frames_impacted_by_this_one=[]
                    previous_frames_impacting_this_one=[]
                    clear_pream_time_heard = 10000 # 10s, a lot

                    # interference
                    if self.frequencyCollision(node.pid,tx_id) and self.modulationCollision(node.pid,tx_id):
                        # tx packet has those few mW of power strength perceived at node
                        # stored in tx node's rx_array
                        this_mw_this_dev = 10**(.1*(nodes[txid].packet.rx_array[node_uid]))

                        # update interference levels for/from other signals on air
                        for other in OnAir:
                            if self.frequencyCollision(node.pid,other) and self.modulationCollision(node.pid,other):
                                other_signal_mw = 10**(.1*(nodes[uid(*other)].packet.rx_array[node_uid]))
                                nodes[txid].packet.interf_mw[node_uid] += other_signal_mw
                                nodes[uid(*other)].packet.interf_mw[node_uid] += this_mw_this_dev

                    # update reception conditions of nodes if perceived above sensitivity
                    if self.check_heard(tx_id,node.pid): 

                        # determine overlaps at rx (node)
                        in_ears=1 # packet is in ears

                        heards = []
                        for other in OnAir:
                            if self.check_heard(other,node.pid): 
                                in_ears+=1
                                heards.append(other)

                        # study collisions//captures on node.pid
                        for other in heards:
                            # general case: receiver's net applies full collision model
                            if self.networks[node.net_id]["full_collision"]:
                                # check who collides in the power domain
                                c = self.powerCollision(tx_id, other, in_ears=in_ears,loc_id=node.pid)
                                # either this one, the other one, or both

                                # check if node could focus on packet's preamble to "lock" its reception on it
                                receiving_packet = False
                                # mark all the collided packets

                                # if this tx is power collided:
                                if tx_id in c:

                                    clear_pream_time_heard_here, not_enough_preamble = self.timingCollision(tx_id, other, ptype="dev", loc_id=node.pid)

                                    clear_pream_time_heard = min(clear_pream_time_heard, clear_pream_time_heard_here)
                                    if not_enough_preamble:# tx is not late enough (not enough preamble heard, no preamble or not a LoRa-CSS receiver)
                                        locally_collided=True
                                        previous_frames_impacting_this_one.append((other,nodes[uid(*other)].packet.ptype))
                                        if self.log_events:
                                            self.MainLogger.info((node.pid,"col",tx_id,tx_id,self.env.now))
                                    else:
                                        receiving_packet = True
                                else:
                                    receiving_packet = True


                                if receiving_packet: # in good condition to lock on packet when enough preamble symbols heard (but another frame may occur in future and impair that)
                                    nodes[txid].packet.receiving[node_uid]=True
                                    nodes[txid].packet.receiving_start[node_uid]=self.env.now

                                # if other current tx power collided
                                if other in c:
                                    if self.log_events:
                                        self.MainLogger.info((node.pid,"col",tx_id,other,self.env.now))
                                    previous_frames_impacted_by_this_one.append((other,nodes[uid(*other)].packet.ptype))
                                    nodes[uid(*other)].packet.receiving[node_uid]=False
                            
                            else: # heard signals always collide if not "full_collision"
                                if self.log_events:
                                    self.MainLogger.info((node.pid,"col",tx_id,other,self.env.now))
                                    self.MainLogger.info((node.pid,"col",other,tx_id,self.env.now))
                                previous_frames_impacting_this_one.append((other,nodes[uid(*other)].packet.ptype))
                                previous_frames_impacted_by_this_one.append((other,nodes[uid(*other)].packet.ptype))
                                locally_collided=True


                        # mark the packet in list of receptions during listening 
                        # in CANL with RTS, a possible weakness of the design is when a data packet has the size of a RTS. In that case, a receiver misreads the data as a RTS and expects a following data with a mismatch random size.
                        node.heard_frames.append({
                            "id":len(node.heard_frames),
                            "node_id":tx_id,
                            "toa":self.get_rectime(tx_id),
                            # "toa":nodes[txid].packet.rectime,
                            "is_RTS":nodes[txid].packet.ptype == constants.rtsPacketType,
                            "dataPayloadSize_in_RTS":rng.integers(0,self.networks[net_id]["max_payload_size"]+1) if nodes[txid].packet.dataPayloadSize==(self.networks[net_id]["profiles"][node.profile]["CANL22_RTS_hdr_size"]+1) else nodes[txid].packet.dataPayloadSize, # if Data of RTS size, random size

                            "dataPayloadSize_in_EH":nodes[txid].packet.dataPayloadSize, # in explicit header
                            "start":self.env.now,
                            "interfering":previous_frames_impacted_by_this_one,
                            "interfered_by":previous_frames_impacting_this_one,
                            "clear_pream_time_heard":clear_pream_time_heard
                        })

                        ######### packet is detected, stop normal listen process and keep listening the time necessary for a valid header to be received 
                        try:
                            node.listen_process.interrupt()
                        except RuntimeError:
                            # node has just stopped listening, listen_process is already interrupted
                            # should not occur anymore (2025-04-10)
                            pass

                        
                        if not locally_collided:
                            if self.log_events:
                                self.MainLogger.info((node.pid,"rx",tx_id,self.env.now))
        return 0


    # For 2 channels ch1 and ch2, computes how much ch1 is overlapped by ch2, and vice versa
    def frequencyOverlaps(self,ch1,ch2):
        
        if ch1 == ch2:
            return(1,1)

        center1,bw1 = ch1
        center2,bw2 = ch2

        start1 = center1 - bw1/2
        end1 = center1 + bw1/2

        start2 = center2 - bw2/2
        end2 = center2 + bw2/2

        if start1 <= start2:
            if start2 >= end1:
                return(0,0)
            if end2 <= end1: # all 2 in 1
                return(bw2/bw1,1)
            # both partially overlap
            return((end1 - start2)/bw1,(end1 - start2)/bw2)

        # start2 before
        if start1 >= end2:
            return(0,0)
        if end1 <= end2: #all 1 in 2
            return(1,bw1/bw2)
        return((end2 - start1)/bw1,(end2 - start1)/bw2)


    # Check if two packets/signals/fragments are transmitted on overlapping (sub)channels
    def frequencyCollision(self, id1, id2):
        uid1 = uid(*id1)
        uid2 = uid(*id2)

        ch1 = nodes[uid1].get_chan()
        ch2 = nodes[uid2].get_chan()

        o1,o2 = self.frequencyOverlaps(ch1,ch2)

        # a minimal overlap causes freq coll. 
        return(o1>0 or o2>0)
        # TODO: refine, store these overlap portions and act tech dependently (e.g. (these are not realistic values) a FHSS fragment overlapped more than 10% is lost, while a LoRa overlapped at 30% still ok)
            # # cf an older frequencyCollision  
            # # frequencyCollision, conditions
            # #
            # #         |f1-f2| <= 120 kHz if f1 or f2 has bw 500
            # #         |f1-f2| <= 60 kHz if f1 or f2 has bw 250
            # #         |f1-f2| <= 30 kHz if f1 or f2 has bw 125


    # Check if two packets/signals/fragments transmitted on overlapping (sub)channels impact each other or if their modulations are orthogonal 
    def modulationCollision(self, id1, id2):
        uid1 = uid(*id1)
        uid2 = uid(*id2)
        if nodes[uid1].modulation == nodes[uid2].modulation:
            return nodes[uid1].modulationCollision(nodes[uid1].getModulationParam(),nodes[uid2].getModulationParam())
        return True# conservative hyp: 2 different modulations collide 
        # TODO: design other coexistence hyps


    # check the time overlap of two transmissions
    # check if enough preamble of tx id1 is heard so that it is spared
    # at a given device (loc_id=devID) // at a GW (loc_id=gw_ID)
    # NB: p1 start is posterior to p2 start
    def timingCollision(self, id1, id2, loc_id=-1, ptype="dev", ocurring_now=True):
        uid1 = uid(*id1)
        uid2 = uid(*id2)
        p1 = nodes[uid1].packet
        p2 = nodes[uid2].packet


        # assuming p1 is the freshly arrived packet and this is the last collision check
        # we know p2 is not finished (it would not be on air, if the contrary was true)
        # if we entered here, we know that p2 is in time collision

        # we've already determined that p1 is a weak packet, (it lost powercollision()) so the only
        # way we can win is by being late enough (only the first n - x preamble symbols overlap)
        # x being detection_min_preamb_symb

        # rx must be LoRa, p1 must have a preamble
        if ptype=="gw":
            preamble_dep_RX = False
            for net_id in range(len(self.networks)):
                if loc_id in self.networks[net_id]["gateways"]["gw_ids"]:
                    if self.networks[net_id]["modulation"] == "LoRa-CSS":
                        preamble_dep_RX = True
                    break
        else:
            preamble_dep_RX = self.networks[loc_id[0]]["modulation"] == "LoRa-CSS"
        preamble_dep_id1 = self.networks[id1[0]]["modulation"] == "LoRa-CSS"
        # preamble_dep_id2 = self.networks[id2[0]]["modulation"] == "LoRa-CSS"

        if not (preamble_dep_RX and preamble_dep_id1):
            return (0,True)

        # check whether p2 ends in p1's critical section
        p2_end = self.get_rectime(id2) + self.get_addTime(id2)

        p1_cs_start = self.env.now + p1.Tpream - p1.Tmin_preamb_heard if ocurring_now else p1.addTime + p1.Tpream - p1.Tmin_preamb_heard 

        # free chunk of p1's preamble
        preamb_heard = self.env.now + p1.Tpream - p2_end if ocurring_now else p1.addTime + p1.Tpream - p2_end

        if p1_cs_start < p2_end: #p2 is somewhere overlapping critical section of p1
            # p1 collided with p2 and lost
            # print("not late enough")
            return (preamb_heard, True)
        # print("spared by the preamble")
        return (preamb_heard, False)



    #
    # Check and store the reception conditions of two packets 
    # at a given device (loc_id=devID) // at a GW (loc_id=gw_ID)
    # depends on how many transmissions are perceived at receiver at the time
    # depends on how much interference there is
    # depends whether the receiver is tuned ("locked", "synced") on a signal or not
    # NB: p1 start is posterior to p2 start
    def powerCollision(self, id1, id2, in_ears=2,loc_id=-1, ptype="dev", ocurring_now=True):

        uid1 = uid(*id1)
        uid2 = uid(*id2)
        p1 = nodes[uid1].packet
        p2 = nodes[uid2].packet

        # mWs have been collected earlier
        if ptype!="gw": # receptor is a (listening) device 
            uloc_id = uid(*loc_id)

            if p1.interf_mw[uloc_id]>0: 
                SIR_1 = p1.rx_array[uloc_id] - 10*np.log10(p1.interf_mw[uloc_id])
            else:
                SIR_1 = 200 # default max

            if p2.interf_mw[uloc_id]>0:
                SIR_2 = p2.rx_array[uloc_id] - 10*np.log10(p2.interf_mw[uloc_id])
            else:
                SIR_2 = 200

            # RX is working with preambles here, because ptype=="dev" (only lora devs listen, for now)
            # count preamble symbs of old P2
            p2synced = False
            if p2.receiving[uloc_id]:
                p1a = self.env.now if ocurring_now else self.get_addTime(id1)
                if p1a - p2.receiving_start[uloc_id] >= nodes[uloc_id].packet.Tmin_preamb_heard : #Rx has synced on old (enough symbs)
                    p2synced = True

            # when the receiver is "locked", it is less impacted by interference
            interf_coef = 1
            if p2synced and SIR_2 > 0:
                interf_coef = self.networks[loc_id[0]]["capture_interf_lock_coef"]


            # check receptions, considering a non linear impact of interference
            p1_rec = p1.rx_array[uloc_id] - p1.noise_dB_arr[uloc_id] + constants.interf_impact_sensi(SIR_1) >= nodes[uloc_id].get_rx_sensitivity(p1.get_tx_config(),"DEVICE")

            p2_rec = p2.rx_array[uloc_id] - p2.noise_dB_arr[uloc_id] + interf_coef*constants.interf_impact_sensi(SIR_2) >= nodes[uloc_id].get_rx_sensitivity(p2.get_tx_config(),"DEVICE")


        else: # same on a GW
            if p1.interf_mw_GWs[loc_id]>0:
                SIR_1 = p1.rss_GWs[loc_id] - 10*np.log10(p1.interf_mw_GWs[loc_id])
            else:
                SIR_1 = 200
            if p2.interf_mw_GWs[loc_id]>0:
                SIR_2 = p2.rss_GWs[loc_id] - 10*np.log10(p2.interf_mw_GWs[loc_id])
            else:
                SIR_2 = 200

            # check if loc_id is a preamble dependent GW (LoRa, not LR_FHSS)
            preamble_dep = False
            for GW_net_id in range(len(self.networks)):
                if loc_id in self.networks[GW_net_id]["gateways"]["gw_ids"]:
                    if self.networks[GW_net_id]["modulation"] == "LoRa-CSS":
                        preamble_dep = True
                    break

            p2synced = False

            if not preamble_dep:
                # if gw is receiving a fragment A, it is locked on A
                p2synced = p2.receiving_GWs[loc_id]
            else:
                if p2.receiving_GWs[loc_id]:
                    p1a = self.env.now if ocurring_now else self.get_addTime(id1)

                    # value for GW is value for first node of same network
                    this_Tmin_preamb_heard = nodes[uid(GW_net_id,0)].packet.Tmin_preamb_heard
                    if p1a - p2.receiving_start_GWs[loc_id] >= this_Tmin_preamb_heard: #Rx has synced on old (enough symbs)
                        p2synced = True

            interf_coef = 1
            if p2synced and SIR_2 > 0:
                interf_coef = self.networks[GW_net_id]["capture_interf_lock_coef"]

            p1_rec = p1.rss_GWs[loc_id] - p1.noise_dB[loc_id] + constants.interf_impact_sensi(SIR_1) >=             gateways[loc_id].get_rx_sensitivity(p1.get_tx_config())

            p2_rec = p2.rss_GWs[loc_id] - p2.noise_dB[loc_id] + interf_coef*constants.interf_impact_sensi(SIR_2) >= gateways[loc_id].get_rx_sensitivity(p2.get_tx_config())

        
        ## Conclude: did p1 break p2's reception? was p1 hidden by p2?
        # TODO: rename powerCaptures into a more suitable metric name
        if (not p1_rec):
            if (not p2_rec):
                powerCaptures.append((in_ears,False,loc_id,ptype))
                return (id1, id2)
            powerCaptures.append((in_ears,True,loc_id,ptype))
            return (id1,)
        # if p1_rec then p2 is not rec (both cannot be rec simultaneously (on devices... on GW? => TODO: check consistency))
        powerCaptures.append((in_ears,True,loc_id,ptype))
        return (id2,)

    # helper to update stats for a dev and its network object
    def mark_collision(self,pair_id, gw_id):
        hid = uid(*pair_id)

        if gw_id in nodes[hid].myGWs_ids: 
            gw_local_id = nodes[hid].myGWs_ids.index(gw_id)

            # only mark a collision if it was not already marked (collision with a previously started transmission)
            if nodes[hid].packet.collided_at_GWs[gw_local_id] == 0:
                nodes[hid].n_captured +=1
                nodes[hid].packet.collided_at_GWs[gw_local_id] = 1
                self.nets[pair_id[0]].nrCaptures+=1

        # if log_events:
        #     MainLogger.info(("GW","col",packet.node_id,nodes[other].packet.node_id,self.simu.env.now))

        nodes[hid].packet.receiving_GWs[gw_id]=False            


    # Check if the GWs of a network can process a new reception 
    def can_process(self, net_id):

        # TODO: different for each GW, wrt check_heard, techno dep...
        ### self.packet.processed is not used anyway in this version 2026-05-05
        processing = 0
        if len(OnAir)>0:
            for other_id in OnAir:
                if other_id[0] == net_id:
                    other_uid = uid(*other_id)
                    if nodes[other_uid].packet.processed == 1 :
                        processing = processing + 1


        if (processing > self.networks[net_id]["maxBSReceives"]):
            # print("too many packets on air:", len(OnAir))
            return 0
        else:
            return 1

    # access the packet object of a node
    def get_packet(self,pair_id):
        hid = uid(*pair_id)
        return(nodes[hid].packet)

    # access a node object
    def get_node(self,pair_id):
        hid = uid(*pair_id)
        return(nodes[hid])

    # access the current strengths of a signal at all the GWs 
    def get_rss_GWs(self,pair_id):
        hid = uid(*pair_id)
        return(nodes[hid].packet.rss_GWs)

    # helper to add mWs of interference impacting a transmission at all the GWs
    def add_interf_GWs(self,pair_id,interf_mw_GWs):
        hid = uid(*pair_id)
        nodes[hid].packet.interf_mw_GWs += interf_mw_GWs

    # helper to set mWs of interference impacting a transmission at a given device
    # TODO: check if vectorializable
    def set_interf(self,pair_id,loc_id,interf_mw):
        hid = uid(*pair_id)
        lid = uid(*loc_id)
        nodes[hid].packet.interf_mw[lid] = interf_mw

    # Check if a transmission is perceived at all the GWs
    def check_heard_at_GWs(self,pair_id):
        hid = uid(*pair_id)
        return(nodes[hid].packet.rss_GWs - nodes[hid].packet.noise_dB  > self.get_gws_sensitivities(nodes[hid].packet.get_tx_config()))

    # access the current strengths of a signal at a given device
    def get_rss(self,tx_id,rx_id):
        tx_uid = uid(*tx_id)
        rx_uid = uid(*rx_id)
        rss = nodes[tx_uid].packet.rx_array[rx_uid]
        return rss

    # 
    # check if a device perceives a neighbor frame above its sensitivity threshold 
    # (on/partially on the channel/modulation its tuned on)
    def check_heard(self,tx_id,rx_id):
        tx_uid = uid(*tx_id)
        rx_uid = uid(*rx_id)
        rss = nodes[tx_uid].packet.rx_array[rx_uid]
        if self.frequencyCollision(tx_id,rx_id) and self.modulationCollision(tx_id,rx_id):
            # TODO: different SF/BW pairs? current simu (2026-05-05) is SF12/BW125 only, mono, but...
            return (rss - nodes[tx_uid].packet.noise_dB_arr[rx_uid] >= nodes[rx_uid].get_rx_sensitivity(nodes[tx_uid].packet.get_tx_config()))

        return False

    # gather the sensitivity levels of all the GWs according to the type of signal (modulation, technique, etc.)
    # TODO: adjust levels in a finer coexistent scenario
    def get_gws_sensitivities(self, properties_of_signal_being_received):
        gws_sensitivities = []
        for gw in gateways:
            gws_sensitivities.append(gw.get_rx_sensitivity(properties_of_signal_being_received))

        gws_sensitivities = np.array(gws_sensitivities)
        return gws_sensitivities

    # determines belonging to a same cluster
    def same_cluster(self, devid, other_devid):
        node = nodes[uid(*devid)]
        other = nodes[uid(*other_devid)]

        if node.cluster not in [-1,-2]:# cluster -1 is the no-cluster, -2 is the cluster of externals
            if node.net_id == other.net_id:
                if node.cluster==other.cluster:
                    return True
        return False


    # translates noise map updates to the devices info
    def update_noise(self, noise_map): 

        xres, yres = noise_map.shape 

        # adjust scale: coordinates of the GWs vs coords in noise map
        GWx_idx = np.round(xres*self.gateways["xs"]/self.xmax).astype(int)
        GWy_idx = np.round(yres*self.gateways["ys"]/self.ymax).astype(int)
        noise_at_GWs = noise_map[GWx_idx,GWy_idx]

        noise_at_nodes = []
        for node in nodes:
            node.packet.noise_dB = noise_at_GWs # noise levels at GWs are related to each GW, not dev dependent 
            noise_at_nodes.append(noise_map[round(node.x*xres/self.xmax)][round(node.y*yres/self.ymax)])

        noise_at_nodes = np.array(noise_at_nodes)
        
        for node in nodes:
            node.packet.noise_dB_arr = noise_at_nodes # idem, noise is defined at receiver, stored at transmitters


    # build the statistics/results/metrics for all devices and networks of the simulation
    # (option) store the results in a pickle dump 
    # in case of an external script running multiple simulations, each of them should be identified by a run_index
    def compute_simu_stats(self, pickle_each = False, start_time = "current_simu", run_index = 0):

        res = []
        for net_id in range(len(self.networks)):
            res.append(self.compute_network_stats(net_id))

        if not pickle_each:
            return res

        pickle.dump(res, open('results/{0}/{0}_{1}_data.dat'.format(start_time,run_index), 'wb'))

        return {}



    # build at the end of the simulation the statistics/results/metrics for all devices of a given network
    def compute_network_stats(self, net_id):

        netp = self.networks[net_id] # parameters
        netv = self.nets[net_id] # state variables

        res={}

        # compute energy levels (old)
        TX = constants.TX 
        RX = constants.RX
        V = constants.V

        # compute energy levels
        # TODO: adjust to each technique 
        TX = constants.TX_868_14dBm
        RX = constants.RX_boosted_DCDC_mode
        V = constants.V

        MCU_on = constants.MCU_on
        MCU_sleep = constants.MCU_sleep


        #################################################################
        #################################################################
        #statistics per node
        res["nodes"]={}

        # gather all the nodes in this network 
        this_nets_nodes = nodes[slice_net_nodes(net_id)]

        for node in this_nets_nodes:
            res["nodes"][node.node_id]={
                "number_of_CAD": node.n_CAD,
                "node_type": 'endDevice' if node.nodeType==constants.endDeviceType else 'relayDevice',
                "node_traffic": 'expo' if node.distrib==constants.expoDistribType else 'uniform',
                # "x":node.x,
                # "y":node.y, 
                "dist_to_center":node.dist_to_center,
                "cluster":node.cluster,
                "profile":node.profile,
                "modulation":node.modulation
            }


            energy = 0

            if node.modulation == "LoRa-CSS":
                #### ENERGY in CAD ####
                str_sf = "SF"+str(node.packet.sf)
                str_bw = "BW"+str(node.packet.bw)
                str_cadsym = str(netp["nCadSym"])+"S"

                #consumption must be converted into mA
                energy = ((constants.cad_consumption[str_sf][str_bw][str_cadsym]*3600/1e9) * V * node.n_CAD )
                
                #microprocessor is on during CAD (sampling+processing)
                energy += (node.packet.symTime * (1/2+netp["nCadSym"]) * node.n_CAD * MCU_on * V / 1e6)


            node.CAD_energy=energy
            res["nodes"][node.node_id]["energy_in_CAD_J"]=energy

            ##### TIME in TX 
            time_sending_data=0
            start_sending_data=0
            stop_sending_data=0
            for cl in channel_log:
                if cl[2]==node.pid:
                    if start_sending_data==0:
                        start_sending_data=cl[0]
                    time_sending_data+=cl[1]        
                    stop_sending_data=cl[0]+cl[1]        

            #### ENERGY in TX
            energy = (time_sending_data * constants.TX_currents[constants.power_levels.index(node.packet.txpow)] * V) / 1e6


            #microprocessor is on
            energy += (time_sending_data * MCU_on * V) / 1e6

            res["nodes"][node.node_id]["energy_in_transmission_J"]=energy

            #### ENERGY in RX
            energy = (node.total_listen_time * RX * V) / 1e6
            #mike is on
            energy += (node.total_listen_time * MCU_on * V) / 1e6

            res["nodes"][node.node_id]["energy_in_listening_J"]=energy

            #### TOTAL ENERGY
            time_on = time_sending_data + node.total_listen_time

            if node.modulation == "LoRa-CSS":
                time_on += node.packet.symTime * (1/2+netp["nCadSym"]) * node.n_CAD

            # #mike is on during radio wakeups and configs 
            time_on += (node.radio_wake_up_time * node.radio_wake_up_number )
            time_on += (node.radio_config_time * node.radio_config_number )

            time_off = self.endSim - time_on

            res["nodes"][node.node_id]["radio_config_number"]= node.radio_config_number

            res["nodes"][node.node_id]["total_energy_J"]=(
                time_sending_data * constants.TX_currents[constants.power_levels.index(node.packet.txpow)] * V + \
                node.total_listen_time * RX * V) / 1e6 + node.CAD_energy + \
                (node.radio_wake_up_time * node.radio_wake_up_number * MCU_on * V / 1e6) + \
                (node.radio_config_time * node.radio_config_number * MCU_on * V / 1e6) + \
                time_off * MCU_sleep * V / 1e6

            #### ENERGY per SUCCESS
            res["nodes"][node.node_id]["energy_per_success"]=res["nodes"][node.node_id]["total_energy_J"]/node.n_data_success if node.n_data_success>0 else -1
            res["nodes"][node.node_id]["length_of_success"]=node.n_payload_success/node.n_data_success if node.n_data_success>0 else -1

            res["nodes"][node.node_id]["mean_payload_length"]= node.n_payload_gen/node.cycle if node.n_payload_gen>0 else -1
            
            res["nodes"][node.node_id]["energy_per_payload_success"]= res["nodes"][node.node_id]["total_energy_J"]/node.n_payload_success if node.n_payload_success>0 else -1

            res["nodes"][node.node_id]["end_simulation_time"]=" {}ms {}h".format(self.endSim, float(self.endSim/3600000))
            res["nodes"][node.node_id]["cumulated_TX_time_s"]=time_sending_data/1000
            res["nodes"][node.node_id]["duty_cycle"]=time_sending_data/(stop_sending_data-start_sending_data) if stop_sending_data>start_sending_data else 0

            res["nodes"][node.node_id]["cumulated_RX_time_s"]=node.total_listen_time/1000

            # res["nodes"][node.node_id]["number_of_CAD"]= node.n_CAD #sum (n.n_CAD for n in nodes)
            res["nodes"][node.node_id]["sent_data_packets"]= node.n_data_sent
            res["nodes"][node.node_id]["success_data_packets"]= node.n_data_success
            res["nodes"][node.node_id]["DER"]= node.n_data_success/node.n_data_sent if node.n_data_sent>0 else -1
            res["nodes"][node.node_id]["DER_method_2"]= (node.n_data_sent-node.n_collided)/node.n_data_sent if node.n_data_sent>0 else -1
            res["nodes"][node.node_id]["PDR"]= node.n_data_success/(node.n_data_sent+node.n_dropped+node.n_aborted) if (node.n_data_sent+node.n_dropped+node.n_aborted > 0 ) else -1

            res["nodes"][node.node_id]["payload_byte_delivery_ratio"]= node.n_payload_success/node.n_payload_gen if node.n_payload_gen>0 else -1

            res["nodes"][node.node_id]["mean_latency"]= node.latency/node.n_data_sent if node.n_data_sent>0 else -1
            res["nodes"][node.node_id]["mean_success_latency"]= node.success_latency/node.n_data_success if node.n_data_success>0 else -1
            res["nodes"][node.node_id]["min_success_latency"]= node.min_success_latency
            res["nodes"][node.node_id]["aborted_packets"]= node.n_aborted
            res["nodes"][node.node_id]["collided_packets"]= node.n_collided
            res["nodes"][node.node_id]["captured_packets"]= node.n_captured
            res["nodes"][node.node_id]["lost_packets"]= node.n_lost
            res["nodes"][node.node_id]["dropped_packets"]= node.n_dropped
            res["nodes"][node.node_id]["kamikazed_packets"]= node.kamikaze

            res["nodes"][node.node_id]["mean_retry"]= node.total_retry/node.n_data_sent if node.n_data_sent>0 else -1

            res["nodes"][node.node_id]["sent_rts_packets"]=node.n_rts_sent

            res["nodes"][node.node_id]["fragments_sent"]= node.n_fragment_sent
            res["nodes"][node.node_id]["fragments_collided"]= node.n_fragment_collided
            res["nodes"][node.node_id]["fragments_lost"]= node.n_fragment_lost


        res["settings"]={
            "Nodes": netp["nrNodes"],
            "externals_prop":netp["externals_prop"],
            "AvgSendTime": netp["avgSendTime"],
            "Distribution": 'expoDistribType' if netp["distribType"]==constants.expoDistribType else 'uniformDistribType',
            "Experiment": netp["experiment"],
            # "Simtime": simtime,
            "Full Collision": netp["full_collision"],
            # "Default_TOA DATA": nodes[0].data_rectime,
            # "TOA RTS": nodes[0].rts_rectime,
            # "DIFS": nodes[0].packet.Tpream,
            # "n_retry": n_retry,
            "CAD_prob": netp["CAD_prob"],
            "Packet length": netp["packetLength"],
            # "targetSentPacket": targetSentPacket, 
            "targetSchedPacket": netp["targetSchedPacket"], 
            "gaussian_noise":self.radio_params["gaussian_noise"],
            # "CAD_proba":{i:list(CAD_distrib_funcs[i]) for i in CAD_distrib_funcs} if False else {}
        }

        # legitimate nodes are non external ones, those whose cluster is not -2
        # network metrics imply legit nodes only
        legit_nodes=[]
        for n in this_nets_nodes:
            if n.cluster!=-2:
                legit_nodes.append(n)

        legit_nrNodes = len(legit_nodes)


        sent = sum(n.n_data_sent for n in legit_nodes)
        rts_sent = sum(n.n_rts_sent for n in legit_nodes)
        
        # this TOTAL dict gathers network metrics
        res["TOTAL"]={
            "radio_config_number": sum( n.radio_config_number for n in legit_nodes) / legit_nrNodes,
            "energy_in_CAD_J":sum( n.CAD_energy    for n in legit_nodes)  / legit_nrNodes,
            "energy_in_transmission_J":sum( [res["nodes"][n.node_id]["energy_in_transmission_J"] for n in legit_nodes])  / legit_nrNodes,
            # "energy_in_listening_J":sum( n.total_listen_time * RX * V for n in legit_nodes) / 1e6,
            "energy_in_listening_J":sum( [res["nodes"][n.node_id]["energy_in_listening_J"] for n in legit_nodes])   / legit_nrNodes,
            "total_energy_J":sum( [res["nodes"][n.node_id]["total_energy_J"] for n in legit_nodes])  / legit_nrNodes,
            "end_simulation_time":" {}ms {}h".format(self.endSim, float(self.endSim/3600000)),

            "cumulated_TX_time_s":sum( [res["nodes"][n.node_id]["cumulated_TX_time_s"] for n in legit_nodes])  / legit_nrNodes,
            "number_of_CAD":sum (n.n_CAD for n in legit_nodes)  / legit_nrNodes,

            "sent_data_packets": sent / legit_nrNodes,
            # "mean_latency": sum (float(n.latency)/float(n.n_data_sent) for n in legit_nodes) / legit_nrNodes if ( node.n_data_sent>0 and legit_nrNodes > 0 )else -1,
            "min_success_latency": sum (n.min_success_latency for n in legit_nodes) / legit_nrNodes,
            "aborted_packets": sum (n.n_aborted for n in legit_nodes)  / legit_nrNodes,
            "collided_packets": netv.nrCollisions  / legit_nrNodes,
            "captured_packets": netv.nrCaptures / legit_nrNodes,
            "lost_packets": netv.nrLost  / legit_nrNodes,
            "dropped_packets": sum (n.n_dropped for n in legit_nodes)  / legit_nrNodes,
            "kamikazed_packets": sum (n.kamikaze for n in legit_nodes)  / legit_nrNodes,        

            "fragments_sent": netv.nrFragmentsSent / legit_nrNodes,
            "fragments_collided": netv.nrFragmentCollisions / legit_nrNodes,
            "fragments_lost": netv.nrFragmentLosses / legit_nrNodes,
            
            
            "nrCollisions":netv.nrCollisions,
            "nrCaptures":netv.nrCaptures,
            "nrReceived":netv.nrReceived,
            "nrProcessed":netv.nrProcessed,
            "nrSent":netv.nrSent,
            "nrLost":netv.nrLost,
            "nrScheduled":netv.nrScheduled
        }


        # "mean_latency":
        sum_lat=0
        sum_data_sent=0
        legit_having_sent=0
        for n in legit_nodes:
            if n.n_data_sent>0:
                sum_lat+=n.latency
                sum_data_sent+=n.n_data_sent
                legit_having_sent+=1
        if sum_data_sent>0:
            res["TOTAL"]["mean_latency"]=sum_lat/sum_data_sent/legit_having_sent
        else:
            res["TOTAL"]["mean_latency"]=-1    


        # "mean_success_latency":
        sum_suc_lat=0
        sum_data_suc=0
        for n in legit_nodes:
            if n.n_data_success>0:
                sum_suc_lat+=n.success_latency
                sum_data_suc+=n.n_data_success
        if sum_data_suc>0:
            res["TOTAL"]["mean_success_latency"]=sum_suc_lat/sum_data_suc
        else:
            res["TOTAL"]["mean_success_latency"]=-1        


        res["TOTAL"]["energy_per_success"]=sum( [res["nodes"][n.node_id]["total_energy_J"] for n in legit_nodes])/sum_data_suc if sum_data_suc>0 else -1

        res["TOTAL"]["length_of_success"]=sum(n.n_payload_success for n in legit_nodes)/sum_data_suc if sum_data_suc>0 else -1

        res["TOTAL"]["cumulated_RX_time_s"]=sum( (n.total_listen_time) for n in legit_nodes)/legit_nrNodes/1000


        # "mean_retries": 
        sum_retries=0
        sum_data_gen=0
        legit_having_gen=0
        for n in legit_nodes:
            if n.cycle>0:
                sum_retries+=n.total_retry
                sum_data_gen+=n.cycle
                legit_having_gen+=1
        if sum_data_gen>0:
            res["TOTAL"]["mean_retry"]=sum_retries/sum_data_gen/legit_having_gen
            res["TOTAL"]["mean_payload_length"]=sum(n.n_payload_gen for n in legit_nodes)/sum_data_gen
        else:
            res["TOTAL"]["mean_retry"]=-1 
            res["TOTAL"]["mean_payload_length"]=-1 
        # res["TOTAL"]["mean_retry"]=sum((float(n.total_retry)/float(n.n_data_sent)) for n in legit_nodes)/legit_nrNodes

        
        res["TOTAL"].update({
            "sent_rts_packets":rts_sent,
            "nrRTSCollisions":netv.nrRTSCollisions,
            "RTS_received_packets": netv.nrRTSReceived,
            "RTS_processed_packets": netv.nrRTSProcessed,
            "RTS_lost_packets": netv.nrRTSLost,
        })


        if sent>0:
            # old data extraction rate did not include losses
            der = (sent-netv.nrCollisions)/float(sent)
            res["TOTAL"]["DER_method_2"]=der
            der = (netv.nrReceived)/float(sent)
            res["TOTAL"]["DER"]=der

        res["TOTAL"]["duty_cycle"]=sum(res["nodes"][n.node_id]["duty_cycle"] for n in legit_nodes)/legit_nrNodes

        res["TOTAL"]["PDR"]= sum(n.n_data_success for n in legit_nodes)/sum(n.n_data_sent+n.n_dropped+n.n_aborted for n in legit_nodes)
        try:
            res["TOTAL"]["payload_byte_delivery_ratio"]= sum(n.n_payload_success for n in legit_nodes)/sum(n.n_payload_gen for n in legit_nodes)
        except:
            res["TOTAL"]["payload_byte_delivery_ratio"]=0

        try:
            res["TOTAL"]["energy_per_payload_success"]= sum( [res["nodes"][n.node_id]["total_energy_J"] for n in legit_nodes])  / sum(n.n_payload_success for n in legit_nodes)
        except:
            res["TOTAL"]["energy_per_payload_success"] = -1
     
        res["TOTAL"]["n_transmit"]= netv.n_transmit
        res["TOTAL"]["mean_inter_transmit_time_ms"]=netv.inter_transmit_time/float(netv.n_transmit)


        #### Control inter generation times
        TT_gen_times = [0]
        TT_IGTs=[]
        for n in this_nets_nodes:
            TT_gen_times+=n.gen_times[1:]
            npks_gen=len(n.gen_times)
            IGTs=[n.gen_times[i]-n.gen_times[i-1] for i in range(1,npks_gen)]
            if n.cluster!=-2:
                TT_IGTs+=IGTs
        
            res["nodes"][n.node_id]["mean_IGT"] = np.mean(IGTs)
            res["nodes"][n.node_id]["std_dev_IGT"] = np.std(IGTs)

        res["TOTAL"]["mean_IGT"]=np.mean(TT_IGTs)
        res["TOTAL"]["std_dev_IGT"]=np.std(TT_IGTs)

        TT_gen_times.sort()
        npks_gen=len(TT_gen_times)
        Global_TT_IGTs=[TT_gen_times[i]-TT_gen_times[i-1] for i in range(1,npks_gen)]
        if self.keep_Global_TT_IGTs:
            res["TOTAL"]["Global_TT_IGTs"]=Global_TT_IGTs
        res["TOTAL"]["short_IGTs"]=np.count_nonzero(np.array(Global_TT_IGTs)<1000)/len(Global_TT_IGTs)

        ############### Channel metrics ################
        # assert a chronological order (should be, from Simpy)
        channel_log.sort(key=lambda tup: tup[0])
        ### channel_log.append((env.now,node.packet.rectime,node.node_id,node.cycle,int(node.ch),subchan))
        # for r in range(100):
        #     print(type(channel_log[r][-1]), file=sys.stderr)

        res["channels"] = {}                    

        # TODO: chan id is tech dependent, not channel_log!!! => next constructions (channel_occupation, channel_overlap_ratio) are tech-wise (interest is limited in a coex scenario) => make a full multitech analysis (coex)
            # could prepare a unique list of chan, with ids, and a list of overlapping channels
            # or check overlap with func frequencyOverlaps and subchan field
            # should consider portions of band where signals overlap, wrt modulations...


        # prepare the list of channels for this net
        subchannel_list = []
        for chan_id in range(len(constants.Channels[netp["modulation"]]["Channel_list"])):
            ch_bw = constants.Channels[netp["modulation"]]["Channel_list"][chan_id][1]
            if "subchan_bw" in constants.Channels[netp["modulation"]]:
                num_subchans = np.floor(ch_bw /(constants.Channels[netp["modulation"]]["subchan_bw"] + constants.Channels[netp["modulation"]]["subchan_inter_bw"])).astype(int)
                for sch_id in range(num_subchans):
                    subchannel_list.append((chan_id,sch_id))
            else:
                subchannel_list.append((chan_id,-1))
        
        # take from channel log the signals of same modulation and bands (subchan if exists)
        for subchannel_main_id in range(len(subchannel_list)):
            res["channels"][subchannel_main_id]={}

            chan_id,subchan_id = subchannel_list[subchannel_main_id]

            channel_log_local = [cl for cl in channel_log if (cl[4]==chan_id and self.networks[cl[2][0]]["modulation"] == netp["modulation"] and cl[5]==subchan_id)]

            busy_dur=0

            if len(channel_log_local) == 0:
                res["channels"][subchannel_main_id]["channel_occupation"] = 0
            elif len(channel_log_local) == 1:
                res["channels"][subchannel_main_id]["channel_occupation"] = 1
                busy_dur += channel_log_local[0][1]
            else:
                busy_start=channel_log_local[0][0]
                busy_stop=max([channel_log_local[i][0]+channel_log_local[i][1] for i in range(len(channel_log_local))])
                last_previous_end=channel_log_local[0][0]+channel_log_local[0][1]
                prev_tx_id=0
                for tx_id in range(1,len(channel_log_local)):
                    tx=channel_log_local[tx_id]
                    prev_tx=channel_log_local[prev_tx_id]
                    if (tx[0]<last_previous_end): #starts before end of previous 
                        if (tx[0]+tx[1]>last_previous_end) : #ends after end of previous 
                            busy_dur += (tx[0]+tx[1]) - (prev_tx[0]+prev_tx[1])
                            last_previous_end=tx[0]+tx[1]
                            prev_tx_id=tx_id
                    else:
                        busy_dur += tx[1]
                        last_previous_end=tx[0]+tx[1]
                        prev_tx_id=tx_id

                res["channels"][subchannel_main_id]["channel_occupation"]=busy_dur/(busy_stop-busy_start)

            
            max_idv_chan_occ_time = max([sum([cl[1] for cl in channel_log_local if cl[2]==n.pid]) for n in this_nets_nodes])
            totalsum_idv_chan_occ_time = sum([sum([cl[1] for cl in channel_log_local if cl[2]==n.pid]) for n in this_nets_nodes])
            
            try:
                res["channels"][subchannel_main_id]["channel_overlap_ratio"]=(totalsum_idv_chan_occ_time-busy_dur)/(totalsum_idv_chan_occ_time-max_idv_chan_occ_time)
            except:
                res["channels"][subchannel_main_id]["channel_overlap_ratio"]=0

        res["TOTAL"]["channel_overlap_ratio"] = np.mean([res["channels"][subchannel_main_id]["channel_overlap_ratio"] for subchannel_main_id in range(len(subchannel_list))])
        res["TOTAL"]["channel_occupation"] = np.mean([res["channels"][subchannel_main_id]["channel_occupation"] for subchannel_main_id in range(len(subchannel_list))])

        # if kept, e.g. for graphical representation, takes disk space
        if self.keep_chan_log:
            res["TOTAL"]["chanlog"] = channel_log
            res["TOTAL"]["listenlog"] = listen_log
            self.keep_chan_log = False # only keep logs for first net_id, since the logs include all signals
            # TODO: make a separate simu-lev dump
            # TODO: compute occupation/overlap only once for all nets?
            # TODO: spatial reuse is not measured in this metric (overlap does not watch reception conditions, i.e. an overlap in channel log does not necessarily implies a collision)



        ##################### powerCaptures
        # TODO: rename and check metric consistency

        powerChecks=0#len(powerCaptures)
        
        nb_caps=0
        sum_in_ears=0
        max_gw_in_ears=0
        sum_in_ears_with_capture=0

        for n in this_nets_nodes:
            res["nodes"][n.node_id]["sum_in_ears"]=0
            res["nodes"][n.node_id]["nb_caps"]=0
            res["nodes"][n.node_id]["sum_in_ears_with_capture"]=0
            res["nodes"][n.node_id]["powerChecks"]=0

            res["nodes"][n.node_id]["max_overlap_degree"]=0
            res["nodes"][n.node_id]["max_capture_overlap_degree"]=0


        for pcheck in powerCaptures:
            if pcheck[3]=="gw":
                powerChecks+=1
                sum_in_ears+=pcheck[0]
                max_gw_in_ears=max(max_gw_in_ears,pcheck[0])
                if pcheck[1]:
                    nb_caps+=1
                    sum_in_ears_with_capture+=pcheck[0]

            else:
                if pcheck[2][0] == net_id:
                    res["nodes"][pcheck[2][1]]["sum_in_ears"]+=pcheck[0]
                    res["nodes"][pcheck[2][1]]["max_overlap_degree"]=max(res["nodes"][pcheck[2][1]]["max_overlap_degree"],pcheck[0])
                    res["nodes"][pcheck[2][1]]["powerChecks"]+=1
                    if pcheck[1]:
                        res["nodes"][pcheck[2][1]]["nb_caps"]+=1
                        res["nodes"][pcheck[2][1]]["sum_in_ears_with_capture"]+=pcheck[0]
                        res["nodes"][pcheck[2][1]]["max_capture_overlap_degree"]=max(res["nodes"][pcheck[2][1]]["max_capture_overlap_degree"],pcheck[0])
        

        res["TOTAL"]["GW_power_capture_ratio"]=nb_caps/powerChecks if powerChecks>0 else 0
        res["TOTAL"]["GW_overlap_degree"]=sum_in_ears/powerChecks if powerChecks>0 else 0
        res["TOTAL"]["GW_max_overlap_degree"]=int(max_gw_in_ears) # convert numpy.int64 to int for jsonify
        res["TOTAL"]["GW_capture_overlap_degree"]=sum_in_ears_with_capture/nb_caps if nb_caps>0 else 0

        for n in this_nets_nodes:
            res["nodes"][n.node_id]["power_capture_ratio"]=res["nodes"][n.node_id]["nb_caps"]/res["nodes"][n.node_id]["powerChecks"] if res["nodes"][n.node_id]["powerChecks"]>0 else 0
            res["nodes"][n.node_id]["mean_overlap_degree"]=res["nodes"][n.node_id]["sum_in_ears"]/res["nodes"][n.node_id]["powerChecks"] if res["nodes"][n.node_id]["powerChecks"]>0 else 0
            res["nodes"][n.node_id]["mean_capture_overlap_degree"]=res["nodes"][n.node_id]["sum_in_ears_with_capture"]/res["nodes"][n.node_id]["nb_caps"] if res["nodes"][n.node_id]["nb_caps"]>0 else 0

        res["TOTAL"]["power_capture_ratio"]=sum(res["nodes"][n.node_id]["power_capture_ratio"] for n in this_nets_nodes)/netp["nrNodes"]
        res["TOTAL"]["mean_overlap_degree"]=sum(res["nodes"][n.node_id]["mean_overlap_degree"] for n in this_nets_nodes)/netp["nrNodes"]
        res["TOTAL"]["mean_capture_overlap_degree"]=sum(res["nodes"][n.node_id]["mean_capture_overlap_degree"] for n in this_nets_nodes)/netp["nrNodes"]
        res["TOTAL"]["max_overlap_degree"]=sum(res["nodes"][n.node_id]["max_overlap_degree"] for n in this_nets_nodes)/netp["nrNodes"]
        res["TOTAL"]["max_capture_overlap_degree"]=sum(res["nodes"][n.node_id]["max_capture_overlap_degree"] for n in this_nets_nodes)/netp["nrNodes"]


            
        
        # print("-- END ----------------------------------------------------------------------"    )


        return res
        


#
## Build a numpy array of distances inter devices
# scaled to the given topo_scale
# multinetized
def build_dist_mat(topos, topo_scale):
    global distance_matrix
 
    nrNodes = sum(active_nodes) # all the (active for this simu) devices

    distance_matrix = np.zeros((nrNodes,nrNodes))

    # distribute node and net ids
    for net_id in range(len(active_nodes)):
        net_pop = active_nodes[net_id]

        for local_id in range(net_pop):
            flat_ndcs.append(local_id)
            flat_net_ids.append(net_id)

    # get coordinate in topos and compute distances
    for i in range(nrNodes):
        for j in range(nrNodes):
            # .001 => avoid log10(0) (aka I am 1mm away from myself)
            distance_matrix[i][j] = max(.001,(
                (topos[flat_net_ids[i]]['nodes'][flat_ndcs[i]]['x']-topos[flat_net_ids[j]]['nodes'][flat_ndcs[j]]['x'])**2+
                (topos[flat_net_ids[i]]['nodes'][flat_ndcs[i]]['y']-topos[flat_net_ids[j]]['nodes'][flat_ndcs[j]]['y'])**2
                )**(1/2))

    # scale it
    distance_matrix *= topo_scale



# insert parameters into loop
def main_with_params(params):

    global distance_matrix  # numpy array with distances between devs

    # declare global lists 
    # yep, list are mutables, global would not be necessary if not assigned to [] at the end of this func (re-initialization is necessary for repeated use of this function in a multiprocessing pool)
    # without these globals => UnboundLocalError: local variable 'gateways' referenced before assignment
    global nodes,gateways,active_nodes,flat_ndcs,flat_net_ids,OnAir,channel_log,listen_log,powerCaptures

    env = simpy.Environment()
    simu = Simulator(env)


    #############################################
    # Translation of Parameters
    #############################################

        #############################################
        # Parameters for this scenario
        #############################################

    # build default nets
    for net in params["networks"]:
        simu.networks.append(config_networks.prepare_default_network())


    # build specific items for each net
    for net_id in range(len(params["networks"])):
        config_networks.update_network(simu.networks[net_id], params["networks"][net_id])

        # translate dev-wise target sent packets to net-wise; 
        # "targetSentPacket" is not in use in current version (2026-05-12) but see targetSchedPacket
        simu.networks[net_id]["targetSentPacket"] *= simu.networks[net_id]["nrNodes"]
        # net nrNodes has precedence over len(topos["nodes"]
        active_nodes.append(simu.networks[net_id]["nrNodes"])

        # traffic distribution 
        simu.networks[net_id]["distribType"] = constants.expoDistribType if simu.networks[net_id]["distrib"] == "expo" else constants.uniformDistribType if simu.networks[net_id]["distrib"] == "unif" else constants.perioDistribType
        ### Alias "distrib" == "distribType"

        # remaining definition from obsolete design: SF for LoRa-CSS in scenario "experiment 4" is 12
        # TODO: remove obsolete declarations and check multi-SF scenarios 
        simu.networks[net_id]["exp4SF"] = 12

    # Simulation parameters: 
    # Do we log events (an old verbose debug)? 
    simu.log_events = params["log_events"]
    # What is simu id? 
    simu.run_index = params["run_index"]
    # Do we keep channel logs on disk? 
    simu.keep_chan_log = params["keep_chan_log"] if "keep_chan_log" in params else False

    #event logs
    if simu.log_events:
        simu.MainLogger = load_main_logger(params["start_time"])
        simu.MainLogger.info("Started")
        simu.MainLogger.info('start_time:{0}'.format(params["start_time"]))

    # Do we store inter generation times ?
    simu.keep_Global_TT_IGTs = params["keep_Global_TT_IGTs"] if "keep_Global_TT_IGTs" in params else False

    # get devices' coordinates
    topos = pickle.load(open('results/{0}_topos.dat'.format(params["start_time"]), 'rb'))[params["topo"]]

    # compute inter devices' distances
    build_dist_mat(topos, params["topo_scale"])

    # net-wise topological parameters
    for net_id in range(len(params["networks"])):
        this_topo = topos[net_id]

        simu.maxDist = max(simu.maxDist, this_topo['maxDist']*params["topo_scale"]) # from a net node to "a" net center

        # if no full distance with multiGW, then mono eval of CAD at center of topo; besides, eval gw collisions as with full dist, no dev collision eval  
        center_x = this_topo['center']['center_x']*params["topo_scale"]
        center_y = this_topo['center']['center_y']*params["topo_scale"]
        simu.networks[net_id]["center_x"] = center_x
        simu.networks[net_id]["center_y"] = center_y
        
        # add a margin of 20m 
        simu.xmax = max(simu.xmax, center_x + this_topo['maxDist']*params["topo_scale"] + 20*params["topo_scale"])
        simu.ymax = max(simu.ymax, center_y + this_topo['maxDist']*params["topo_scale"] + 20*params["topo_scale"])

    # simu max dist to center  
    simu.dist_max_overall = (simu.xmax**2+simu.ymax**2)**(1/2)

    # radio propagation parameters 
    simu.radio_params = params["radio_params"]



    ###########
    # adjust clustering
    ###########
    for net_id in range(len(params["networks"])):
        this_topo = topos[net_id]
        nrNodes = active_nodes[net_id]
        externals_prop = simu.networks[net_id]["externals_prop"] if "externals_prop" in simu.networks[net_id] else 0.
        node_profiles = simu.networks[net_id]["node_profiles"]
        # profiles = simu.networks[net_id]["profiles"]

        # list ids of devices outside clusters (-1)
        nonclusts=[]
        for n in range(nrNodes):
            if this_topo['nodes'][n]['cluster'] == -1:
                nonclusts.append(n)

        # in the nonclusts, select the first ones, in given proportion, and set them as external interferers
        to_externalize=round(len(nonclusts)*externals_prop)
        for ext_id in range(to_externalize):
            this_topo['nodes'][nonclusts[ext_id]]['cluster'] = -2


        # now each node has cluster, let's profile them all
        # first recount
        clusters_recount={}
        for n in range(nrNodes):
            if this_topo['nodes'][n]['cluster'] in clusters_recount:
                clusters_recount[this_topo['nodes'][n]['cluster']].append(n)
            else:
                clusters_recount[this_topo['nodes'][n]['cluster']]=[n]

        # apply profile to interferers and recount non interferers (contributors)
        contributors = nrNodes
        if -2 in clusters_recount: # externals
            population = len(clusters_recount[-2])
            contributors -= population
            subgroups = []
            subgroups_names = []
            for prof_distrib in node_profiles["externals"]["distrib"]:
                subgroups.append(round(node_profiles["externals"]["distrib"][prof_distrib]*population))
                subgroups_names.append(prof_distrib)
            turno = 0
            while sum(subgroups)<population:
                subgroups[turno]+=1
                turno+=1
                turno%=len(subgroups)


            it = iter(clusters_recount[-2])
            populated_subgroups = [[next(it) for _ in range(size)] for size in subgroups]

            for popsub_id in range(len(populated_subgroups)):
                popsub = populated_subgroups[popsub_id]
                prof_to_apply = subgroups_names[popsub_id]
                for nid in popsub:
                    this_topo['nodes'][nid]['profile'] = prof_to_apply
        

        # translate dev-wise target sched packets to net-wise; 
        ######### the global packet schedule target consists in packets generated by legit contributors
        simu.networks[net_id]["targetSchedPacket"] *= contributors

        # unclustered contributors
        if -1 in clusters_recount: # unclustered
            population = len(clusters_recount[-1])
            subgroups = []
            subgroups_names = []
            for prof_distrib in node_profiles["unclustered"]["distrib"]:
                subgroups.append(round(node_profiles["unclustered"]["distrib"][prof_distrib]*population))
                subgroups_names.append(prof_distrib)
            turno = 0
            while sum(subgroups)<population:
                subgroups[turno]+=1
                turno+=1
                turno%=len(subgroups)


            it = iter(clusters_recount[-1])
            populated_subgroups = [[next(it) for _ in range(size)] for size in subgroups]

            for popsub_id in range(len(populated_subgroups)):
                popsub = populated_subgroups[popsub_id]
                prof_to_apply = subgroups_names[popsub_id]
                for nid in popsub:
                    this_topo['nodes'][nid]['profile'] = prof_to_apply

        
        # clustered devices
        nb_clusters = 0
        nb_clustered = 0
        clust_ids = []
        for clust_id in clusters_recount:
            if clust_id in [-2,-1]:
                continue
            nb_clusters +=1
            nb_clustered += len(clusters_recount[clust_id])
            clust_ids.append(clust_id)


        if nb_clusters>0:
            # profile distribution is cluster wise (in each cluster all devices have the same profile)
            if node_profiles["clustered"]["distribType"] == "full_cluster":

                subgroups = []
                subgroups_names = []
                for prof_distrib in node_profiles["clustered"]["distrib"]:
                    subgroups.append(round(node_profiles["clustered"]["distrib"][prof_distrib]*nb_clusters))
                    subgroups_names.append(prof_distrib)
                turno = 0
                while sum(subgroups)<nb_clusters:
                    subgroups[turno]+=1
                    turno+=1
                    turno%=len(subgroups)

                it = iter(clust_ids)
                populated_subgroups = [[next(it) for _ in range(size)] for size in subgroups]


                for popsub_id in range(len(populated_subgroups)):
                    popsub = populated_subgroups[popsub_id]
                    prof_to_apply = subgroups_names[popsub_id]

                    for clust_id in popsub:
                        for nid in clusters_recount[clust_id]:
                            this_topo['nodes'][nid]['profile'] = prof_to_apply


            # profile distribution is made at node level, network-wise (in each cluster devices may have different profiles)
            elif node_profiles["clustered"]["distribType"] == "node_wise":

                population = nb_clustered
                subgroups = []
                subgroups_names = []
                for prof_distrib in node_profiles["clustered"]["distrib"]:
                    subgroups.append(round(node_profiles["clustered"]["distrib"][prof_distrib]*population))
                    subgroups_names.append(prof_distrib)
                turno = 0
                while sum(subgroups)<population:
                    subgroups[turno]+=1
                    turno+=1
                    turno%=len(subgroups)



                subgroup_id = 0
                
                for nid in range(nrNodes):
                # for nid in this_topo['nodes']:
                    

                    if this_topo['nodes'][nid]['cluster'] in [-2,-1]:
                        continue
                    
                    while subgroups[subgroup_id] == 0:
                        subgroup_id +=1 

                    prof_to_apply = subgroups_names[subgroup_id]
                    this_topo['nodes'][nid]['profile'] = prof_to_apply

                    subgroups[subgroup_id] -= 1


    ###########
    # Prepare CADs (Channel Activity Detection for LoRa-CSS devices)
    # CAD model depends on radio parameters and transmission/reception parameters (SF, BW, TX power...)
    ###########

    # check if it is necessary
    must_prepare_cads = False    
    for net_id in range(len(params["networks"])):
        if simu.networks[net_id]["modulation"] == "LoRa-CSS":
            if "use_CADs" in simu.networks[net_id]:
                if simu.networks[net_id]["use_CADs"]:
                    must_prepare_cads =True
                    break
            else:
                must_prepare_cads =True
                break

    # preparation in LoRa_CSS.py
    if must_prepare_cads:
        LoRa_CSS.prepare_CAD_model(simu.dist_max_overall, simu.radio_params, simu.networks)



    ###########
    # numpyfy GWs and gather their params, ids, etc.
    ###########
    
    # TODO: add specific (net) gw params, e.g. for get_rx_sensitivity

    gw_id = 0
    for net_id in range(len(params["networks"])):
        simu.networks[net_id]["gateways"] = {"xs":[],"ys":[], "gw_ids":[]}
        this_topo = topos[net_id]

        for gw in this_topo['GWs']:
            simu.networks[net_id]["gateways"]["xs"].append(gw["x"]*params["topo_scale"])
            simu.networks[net_id]["gateways"]["ys"].append(gw["y"]*params["topo_scale"])
            simu.networks[net_id]["gateways"]["gw_ids"].append(gw_id)
            
            simu.gateways["xs"].append(gw["x"]*params["topo_scale"])
            simu.gateways["ys"].append(gw["y"]*params["topo_scale"])
            simu.gateways["net_ids"].append(net_id)

            # Create GW object
            if simu.networks[net_id]["modulation"] == "LR-FHSS":
                gateway = LR_FHSS.LR_FHSS_Gateway( 
                    simu,
                    net_id,
                    gw_id, 
                    gw["x"]*params["topo_scale"], 
                    gw["y"]*params["topo_scale"]
                    )
            elif simu.networks[net_id]["modulation"] == "Mioty":
                gateway = Mioty.Mioty_Gateway( 
                    simu,
                    net_id,
                    gw_id, 
                    gw["x"]*params["topo_scale"], 
                    gw["y"]*params["topo_scale"]
                    )
            else:
                gateway = LoRa_CSS.LoRa_CSS_Gateway( 
                    simu,
                    net_id,
                    gw_id, 
                    gw["x"]*params["topo_scale"], 
                    gw["y"]*params["topo_scale"]
                    )

            gateways.append(gateway)

            gw_id += 1
        
        simu.networks[net_id]["gateways"]["xs"]=np.array(simu.networks[net_id]["gateways"]["xs"])
        simu.networks[net_id]["gateways"]["ys"]=np.array(simu.networks[net_id]["gateways"]["ys"])
        # simu.networks[net_id]["gateways"]["gw_ids"]=np.array(simu.networks[net_id]["gateways"]["gw_ids"])

    simu.gateways["xs"]=np.array(simu.gateways["xs"])
    simu.gateways["ys"]=np.array(simu.gateways["ys"])
    simu.gateways["net_ids"]=np.array(simu.gateways["net_ids"])


    ### Set live nets (where net variables are updated during simulation)
    simu.init_network_variables()

    simu.nrNodes = sum(active_nodes)


    ###########
    # Devices
    ###########
    for net_id in range(len(params["networks"])):
        nrNodes = active_nodes[net_id]
        this_topo = topos[net_id]


        for i in range(0,nrNodes):
            if simu.networks[net_id]["modulation"] == "LR-FHSS":
                node = LR_FHSS.LR_FHSS_Node(
                    simu,
                    net_id,
                    i, 
                    this_topo['nodes'][i]['x']*params["topo_scale"], 
                    this_topo['nodes'][i]['y']*params["topo_scale"], 
                    this_topo['nodes'][i]['cluster'],
                    this_topo['nodes'][i]['profile'],
                    constants.endDeviceType, 
                    simu.networks[net_id]["avgSendTime"], 
                    simu.networks[net_id]["distribType"], 
                    simu.networks[net_id]["packetLength"], 
                    obw = 35, # TODO: move to constants. 35*8 = 280 subchans in 8 groups
                    nb_headers = 3, # TODO: move to constants
                    header_duration = 233.472, # TODO: move to constants
                    payload_duration = 102.4, # TODO: move to constants
                    transceiver_wait = 6.472, # TODO: move to constants
                    code = simu.networks[net_id]["LR-FHSS-CR"]
                    )

            elif simu.networks[net_id]["modulation"] == "Mioty":
                node = Mioty.Mioty_Node(
                    simu,
                    net_id,
                    i, 
                    this_topo['nodes'][i]['x']*params["topo_scale"], 
                    this_topo['nodes'][i]['y']*params["topo_scale"], 
                    this_topo['nodes'][i]['cluster'],
                    this_topo['nodes'][i]['profile'],
                    constants.endDeviceType, 
                    simu.networks[net_id]["avgSendTime"], 
                    simu.networks[net_id]["distribType"], 
                    simu.networks[net_id]["packetLength"], 
                    symbol_rate = constants.Mioty_symbol_rate, # 2380.371, 
                    split_duration = constants.Mioty_split_duration #36
                    )
            else:
                node = LoRa_CSS.LoRa_CSS_Node(
                    simu,
                    net_id,
                    i, 
                    this_topo['nodes'][i]['x']*params["topo_scale"], 
                    this_topo['nodes'][i]['y']*params["topo_scale"], 
                    this_topo['nodes'][i]['cluster'],
                    this_topo['nodes'][i]['profile'],
                    constants.endDeviceType, 
                    simu.networks[net_id]["avgSendTime"], 
                    simu.networks[net_id]["distribType"], 
                    simu.networks[net_id]["packetLength"]
                )


            nodes.append(node)

    
    # print(len(nodes),simu.nrNodes, file=sys.stderr)

    # initiate packets propagation conditions
    for node in nodes:
        # Path loss exponents to neighs
        node.packet.gamma_array = np.zeros((simu.nrNodes))
        if simu.networks[node.net_id]["normal_gamma_ED"]:
            node.packet.gamma_array = rng.normal(simu.networks[node.net_id]["gamma_ED"],simu.networks[node.net_id]["sigma_gamma_ED"],node.packet.gamma_array.shape)
        else:
            node.packet.gamma_array += simu.networks[node.net_id]["gamma_ED"]


    ############# cluster gains
    for node in nodes:
        if node.cluster not in [-1,-2]:# cluster -1 is the no-cluster, -2 is the cluster of externals
            for other in nodes:
                if node.cluster==other.cluster and node.net_id == other.net_id:
                    node.packet.gamma_array[uid(other.net_id, other.node_id)] -= simu.networks[node.net_id]["intra_cluster_gamma_gain"]

        # update radio conditions (at first time, set radio conditions that are constants for this simu, e.g. building obstruction)
        node.packet.repropagate(first_time=True)






    ###########
    # SIMULATION PROCESS
    ###########

    # with this correlated_noise parameter, noise at reception is periodically updated, with geographical correlation, in the Radio_environment module
    # without this parameter, noise at reception is constant, or updated at each transmission following a normal distribution. 
    if params["radio_params"]["correlated_noise"]:
        radio_map = Radio_environment.Radio_environment(simu)
        env.process(radio_map.correlated_noise_map(params["radio_params"]["correlated_noise_update_period"]))    


    # process (set) the communicate() function of all the devices in the Simpy environment  
    for node in nodes:
        env.process(node.communicate())    
    
    # remove topo from RAM
    topos=None
        
    # start simulation
    # env.run(until=simtime)
    env.run()

    # close the event logs if any
    if simu.log_events:
        close_logger_handlers(MainLogger)

    # determine if results must be stored on disk (for this simu, or for the set of simus)
    pickle_each = True
    if "pickle_each" not in params or not params["pickle_each"]:
        pickle_each = False
    
    # prepare simu stats
    res = simu.compute_simu_stats(pickle_each = pickle_each, start_time = params["start_time"], run_index = params["run_index"])

    # Clean everything for the case of multiple multiprocessing runs
    del simu
    nodes = []
    gateways = []
    active_nodes = []
    flat_ndcs = []
    flat_net_ids = []
    OnAir = []
    channel_log = []
    listen_log = []
    powerCaptures = []
    distance_matrix = None


    return res








#
# "main" program as used as "python lpwan_sim.py"
#
if __name__ == '__main__':


    # module for the generation of the topology
    import topo_builder

    import time
    import json
    JSON_EXPORT = True

    raw_start=time.localtime()
    start_time=time.strftime("%Y-%m-%d-%H-%M-%S", raw_start)


    example = "LoRa-CSS"

    if len(sys.argv)>1:
        example=sys.argv[1]
        print(example,file=sys.stderr)


    # fetch parameters for examples as prepared by config_networks.py 
    params=pickle.load(open('results/{0}_example_params.dat'.format(example), 'rb'))
    # params=pickle.load(open('results/LR-FHSS_example_params.dat', 'rb'))

    # update start_time
    params["start_time"] = start_time
    pickle.dump(params, open('results/{0}_params.dat'.format(start_time), 'wb'))

    # non default clustering parameters for topologies, see topo_builder.py
    # cluster_params=('manual-uniform',3,250,(5,10))

    # topos is a dict of dicts: simulation can be repeated over several instances of topology, params["topo"]=0, 1, 2... In this main, just one instance (params["topo"]==0) 
    # and each network a, b, c has a topology: topos[0][a], topos[0][b], topos[0][c]...
    topos={params["topo"]:{}}

    
    # build topo for each net
    for net in params["networks"]:
        # topos[0]=topo_builder.build_topo(nrNodes,nrGWs,experiment,maxDist_dev_gw,cluster_params=cluster_params)
        topos[params["topo"]][len(topos[params["topo"]])]=topo_builder.build_topo(net["nrNodes"],net["nrGWs"],net["experiment"],net["maxDist_dev_gw"])


    # store topos so that they can be re-used for other simulations
    pickle.dump(topos, open('results/{0}_topos.dat'.format(start_time), 'wb'))


    # run the simulation
    res = main_with_params(params)

    # reactivate prints
    sys.stdout=stdout_print_target

    # rapidly output first net stats only 
    res3 = res[0]

    for key in res3["TOTAL"]:
        print("{0}:".format(key))
        if isinstance(res3["TOTAL"][key], list):
            if len(res3["TOTAL"][key])>12:
                print(res3["TOTAL"][key][:12])
            else:
                print(res3["TOTAL"][key])
        else:
            print(res3["TOTAL"][key])

    print("")
    print("")
    print("")
    print(start_time)

    # pickle the results (even if pickle_each is false in config_networks.py)
    pickle.dump(res, open('results/{0}_data.dat'.format(start_time), 'wb'))

    # a nicer JSON export of first network's stats
    if JSON_EXPORT:
        json_dict = json.dumps(res3, indent=4)
        with open('results/{0}_results.json'.format(start_time), "w") as outfile:
            outfile.write(json_dict)



