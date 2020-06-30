[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# Multi-Modal Route Planning with Bike Sharing
This C++ framework contains several algorithms for the efficient computation of shortest paths in multi-modal transportation networks.
All algorithms and data structures have a special focus on handling bike-sharing services and bike sharing stations present in the network.
The framework was developed at [KIT](https://www.kit.edu) in the [group of Prof. Dorothea Wagner](https://i11www.iti.kit.edu/).

## Usage

This framework contains code for generating operator-dependent and operator-expanded network representations. Code of additional preprocessing techniques allows for the computation of operator-hulls and ULTRA shortcuts. Finally, the framework contains code of variations of the RAPTOR algorithm, which use the networks and preprocessed data to compute shortest paths. All components of the framework can be compiled into a single console application, using the ``Makefile`` that is located in the ``Runnables`` folder. Within this application the following commands are available:

* ``SanitizeBikeSharingStations`` Unifies different spellings of the names of bike-sharing operators
* ``BuildBikeSharingData`` takes public transit and bike sharing data and builds an operator-dependent network
* ``BuildExtendedBikeSharingData`` takes an operator-dependent network and builds an operator-expanded network
* ``BuildPartialBikeSharingData`` takes an operator-dependent network and removes some bike-sharing operators
* ``GenerateBikeSharingQueries`` generates random triples of source location, target location, and departure time
* ``RunBikeSharingQueries`` evaluates a query algorithm on queries generated with the command above

All of the above commands use custom data formats for loading the public transit network, the transfer graph, and the bike sharing stations. As an example we provide the public transit network of Switzerland together with a transfer graph extracted from OpenStreetMap in the appropriate binary format at [https://i11www.iti.kit.edu/PublicTransitData/Switzerland/binaryFiles/](https://i11www.iti.kit.edu/PublicTransitData/Switzerland/binaryFiles/). Additional files containing the location of bike-sharing stations and their operators are available at [https://i11www.iti.kit.edu/PublicTransitData/BikeSharing/](https://i11www.iti.kit.edu/PublicTransitData/BikeSharing/)
 
## Publications

A detailed introduction to the ULTRA-Bike-Sharing algorithm as well as an in-depth evaluation of its performance is given in:

* *Faster Multi-Modal Route Planning With Bike Sharing Using ULTRA*  
  Jonas Sauer, Dorothea Wagner, Tobias Zündorf  
  In: Proceedings of the 18th International Symposium on Experimental Algorithms (SEA'20), Leibniz International Proceedings in Informatics, pages 16:1--16:14, 2020  
  [pdf](https://drops.dagstuhl.de/opus/volltexte/2020/12090/pdf/LIPIcs-SEA-2020-16.pdf)
