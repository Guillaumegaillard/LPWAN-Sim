# LPWAN-Sim
A simulation framework using Python SimPy and NumPy to compare LPWAN technologies (such as LoRa, LR-FHSS, Mioty, etc.).

```
######## https://github.com/Guillaumegaillard/LPWAN-Sim
######## 2026-04-28
######## License MIT: 
######## https://github.com/Guillaumegaillard/LPWAN-Sim/blob/main/LICENSE 
######## Inspired from LoRaSim https://mcbor.github.io/lorasim/ 
```
## Features:
- a time-discrete event-based simulation framework (relying on [SimPy](https://simpy.readthedocs.io)) modelling networks of LPWAN devices transmitting frames to multiple gateways on multiple channels;
- a structured simulation output of 30 metrics allowing comparative analyses;
- a frame/preamble/fragment-level collision and interference model on coexistent multi-technology scenarios;
- a log-distance radio propagation model with random variables for correlated noise, fast fading and topological obstructions;
- an independent implementation of LoRa-CSS, Mioty, LR-FHSS uplink transmission schemes;
- an energy consumption model for devices;
- a Channel Activity Detection (CAD) probabilistic model for LoRa-CSS devices, coherent with the log-distance radio propagation model;
- a set of CSMA protocols for LoRa-CSS devices based on CAD.


## Developed and tested with:

Python 3.9 within Anaconda on Linux (Xubuntu)

## Requires numpy simpy scipy
`conda create -n LPWAN-Sim python=3.9 numpy=1.26.4 simpy=4.0.1 scipy=1.13.1`

`conda --version`: `23.5.2`

## Main scripts:
```
python config_networks.py
python lpwan_simulator.py
python lpwan_simulator.py LR-FHSS
```

## Documentation:
See the [Guidelines](https://github.com/Guillaumegaillard/LPWAN-Sim/blob/main/Guidelines.md)


## References
Earlier versions of this simulator have been the support for our research on Carrier Sense for LoRa. Without impacting its generality, some options and parameters in the simulator refer to specific notions we have been working with, e.g. the CANL protocol, RTS packets, xCANL_CAD, etc.

Guillaume Gaillard and Congduc Pham. "[Performance of LoRaWAN CSMA in Dense LoRa Networks.](https://hal.science/hal-05411772/)" 2025 International Conference on Modeling, Analysis and Simulation of Wireless and Mobile Systems (MSWiM), Barcelona, Spain, 2025, pp. 324-332, IEEE, doi: 10.1109/MSWiM67937.2025.11309049.

Guillaume Gaillard and Congduc Pham. "[CANL LoRa: Collision Avoidance by Neighbor Listening for Dense LoRa Networks.](https://hal.science/hal-04191330/)" 2023 IEEE Symposium on Computers and Communications (ISCC), Gammarth, Tunisia, 2023, pp. 1293-1298, IEEE, doi: 10.1109/ISCC58397.2023.10218282.

## Contact
All contact about LPWAN-Sim are very welcome :)! 
Our email addresses are indicated in the papers.


