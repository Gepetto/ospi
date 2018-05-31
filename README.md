# ospi
This library contains scripts for working with OpenSim files and pinocchio software. 

## Required Dependencies:
- Python 2.7. Additional libraries are directly handled by setup.py (numpy, math and elementtree)
- Pinocchio: library for rigid multi-body dynamics. Can be download from here: http://stack-of-tasks.github.io/pinocchio/ 

## Extra Dependencies:
If you want to visualize your pinocchio model:
- Gepetto-viewer: A graphical interface for pinocchio. Can be downloaded from here:
    https://github.com/humanoid-path-planner/gepetto-viewer.git
- Gepetto-viewer-corba: CORBA server/client for the Graphical Interface of Pinocchio. Can be downloaded from here:
    https://github.com/humanoid-path-planner/gepetto-viewer-corba.git
    
    
## Installing
```json
   $ python setup.py install --prefix='your install path'
```
