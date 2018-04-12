#!/bin/bash
#pylint --rcfile=pylint.rc src/test/pymunk/simulator.py src/test/pymunk/sim_leg.py src/test/gym/carttable.py
pylint --rcfile=pylint.rc src/test/gym/carttable.py src/test/gym/carttableangular.py
