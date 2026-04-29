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

Finally, the constants, that should not be modified in the scenarii (or not meant to at the time of writing) are specified in `constants.py`.


## `lpwan_simulator.py`

## `config_networks.py`
This is a helper that prepares simulation scenarios, with networks, radio parameters, topologies etc. A basic example for each technique is given, and two examples of coexistent scenarios (various techniques simultaneously). 

## `constants.py`
Numerical values, comprising sensitivities, power consumption values, impact of interference, noise, etc. for the different techniques. It also includes state definitions used in the communication loop of the devices. 

## `Generic_Elements.py`


## `Radio_environment.py`

## `topo_builder.py`

## `LoRa_CSS.py`

## `LR_FHSS.py`

## `Mioty.py`



## Todo list:

- complete the LoRa code for multi-SF multi-BW scenarii;
- implement relays, i.e. full RX capability on nodes;
- complete and refine the sensitivities for each technique;
- complete and refine the energy consumption values for each technique;
- improve taxonomy, i.e. choose names unambiguous and technique independent;
- improve the topology builder (heterogeneous building/obstacle densities, real world scenarii, etc.);
- design a multi-technique symbol-level interference analyzer;
- improve the global structure of the simulator: remove obsolete features, misplaced variables and functions, etc.
- improve the design of device clusters in the simulation.


