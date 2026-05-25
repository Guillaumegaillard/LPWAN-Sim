# Guidelines for LPWAN-Sim

```
######## https://github.com/Guillaumegaillard/LPWAN-Sim
######## 2026-04-28
######## License MIT: 
######## https://github.com/Guillaumegaillard/LPWAN-Sim/blob/main/LICENSE 
######## Inspired from LoRaSim https://mcbor.github.io/lorasim/ 
```

## Structure

LPWAN-Sim simulates networks of devices and gateways, distributed on a 2D map. The devices of a network transmit frames/packets using a given technique/modulation (LoRa-CSS, LR-FHSS, or Mioty) and running one of a list of (MAC) protocols/profiles.
The radio condition parameters are shared among the networks of each simulation scenario.

The main loop of execution of the simulation is developed in `lpwan_simulator.py`. Its function `main_with_params(params)` generates and triggers the simulation based on a full dictionary of parameters. Such a setup can be written manually or using the helper `config_networks.py`. The distribution of devices and gateways in the 2D map is ensured by `topo_builder.py`. The radio conditions are updated along the simulation in `Radio_environment.py`.

The components of the simulation (devices/nodes, gateways, packets/frames) have attributes that are identical regardless of the technique (e.g. coordinates, identifiers), and others are specific (e.g. spreading factor). The common class functions and attributes are specified in `Generic_Elements.py`. These generic elements are inherited and differentiated in the independent technique-specific modules: `LoRa_CSS.py`, `LR_FHSS.py` and `Mioty.py`.

Finally, the constants, that should not be modified in the scenarios (or not meant to at the time of writing) are specified in `constants.py`.

---

## `lpwan_simulator.py`
This main module translates parameters into an instance of simulation, and controls all the structures of shared information (current transmissions, channel log, radio conditions, interference) among the components of the simulation (networks, nodes, gateways, etc.).

Node objects are listed and referenced by a global id in `lpwan_simulator.py` but also get a pair id comprising their network id and their device id in this network.

Current ongoing transmissions are also maintained in a list of pair ids of their transmitters. All the details about each transmission (e.g. time on air, collision history) are maintained and retrieved from the node or packet objects.

For each network the devices are assigned coordinates, cluster, traffic pattern, protocol profile, etc. 

At the end of a simulation (when all the targeted frames have been generated for each network), statistics are build from the simulation variables and gathered in a structured output that can be stored on disk for a further analysis. The metrics highlight transmissions, collisions, losses, energy consumption, delays, channel occupation, etc. and are computed for each device (e.g. PDR of node 1, PDR of node 2), each channel, and each network.

When run as the main script, `lpwan_simulator.py` runs a simulation with a scenario that is prepared in `config_networks.py` and can be given as argument, e.g. `python lpwan_simulator.py Mioty`. The `__main__` portion of the module exemplifies the construction of parameters, topology and the use of the main function `main_with_params()` to trigger the simulation. A JSON output of the simulation results is also proposed as an example.

When called from other scripts, `lpwan_simulator.py` runs independent simulations (global variables are re-initiated at each call to `main_with_params()`) but may use shared resources (Numpy has intrinsic parallel behavior).
For example:
```
import lpwan_simulator
from multiprocessing import Pool

simulations_params_list = [params1,params2,params3...]
with Pool(processes = num_cpus) as p:
    p.map(lpwan_simulator.main_with_params, simulations_params_list)
```


## `config_networks.py`
This is a helper that prepares simulation scenarios, with networks, radio parameters, topologies etc. A basic example for each technique is given, and two examples of coexistent scenarios (various techniques simultaneously). 

## `constants.py`
Numerical values, comprising sensitivities, power consumption values, impact of interference, noise, etc. for the different techniques. It also includes state definitions used in the communication loop of the devices. 

## `Generic_Elements.py`
This module defines core components of the simulator: Nodes, Packets, Gateways. These classes are then inherited by tech-dependent modules that specialize them.

Common functions include `setDataPayloadSize()` that sets the payload size of a frame according to the distribution of probability defined in the parameters. The radio environment of the components is initiated and updated after each transmission in the `repropagate()` function.

The channel hopping mechanism (the way a device switches from one channel to another to transmit frames) is defined in the Node's `update_channel()` function. 

The scheduling of frame generations is dealt with in the `generate_new_frames()` function: new frames are queued according to their distribution of arrival times until the future, and the oldest waiting frame is chosen for transmission. 


## `Radio_environment.py`
In this current version, the radio environment defined in `Radio_environment.py` is limited to an periodical update of a spatially correlated noise map, using a Gaussian filter.

Other updates of the radio conditions are defined in `Generic_Elements.py`, in particular in the  `repropagate()` function.

## `topo_builder.py`
This module gives to all the nodes and gateways of a network their coordinates in a 2D map. The typical topology is a circular one on which nodes are spread uniformly, and GWs at specific, homogeneous, positions. Clusters of nodes can be formed according to various construction rules, nodes are given a cluster id in that case. Since other nodes can be placed outside existing clusters, an empty cluster can serve as a (circular) obstacle in the design of a topology. 

## `LoRa_CSS.py`
Specific definitions and behavior for LoRa-CSS networks. In particular, there are given CAD success models and behavior, airtime according to the CSS configuration, sensitivities, RX mode of the devices, etc.

Several MAC protocols are defined in the main `communicate()` loop, distributed under four families:

- `CANL22`: protocols involving LISTEN phases using the RX mode for devices;
- `ideal_FIFO`: a protocol where devices schedule idealistically all their transmissions without overlapping any other;
- `CAD+Backoff`: protocols using CADs in combination with different implementations of back off mechanisms;
- `ALOHA`: the basic behavior -transmit directly without any Carrier Sense.


## `LR_FHSS.py`
Specific definitions and behavior for LR-FHSS networks. In particular, time frequency fragments sequence definitions and decoding procedure on gateways are given.

## `Mioty.py`
Specific definitions and behavior for Mioty networks. In particular, time frequency split (burst) sequence definitions and decoding procedure on gateways are given.

---

## Todo list:

- complete the LoRa code for multi-SF multi-BW scenarios;
- implement relays, i.e. full RX capability on nodes;
- complete and refine the sensitivities for each technique;
- complete and refine the energy consumption values for each technique;
- improve taxonomy, i.e. choose names unambiguous and technique independent;
- improve the topology builder (heterogeneous building/obstacle densities, real world scenarios, etc.);
- design a multi-technique symbol-level interference analyzer;
- improve the global structure of the simulator: remove obsolete features, misplaced variables and functions, etc.
- complete the Mioty code for down-link and other modes;
- improve the design of device clusters in the simulation.
- better define noise models, better quantify Rayleigh values


