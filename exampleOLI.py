import sys
sys.path.append('ospi')

import subprocess
import time
#from ospi 
import viewer_utils as vw
import wrapper as wr
import motion_parser as mtp



# The path to the model meshes
mesh_path='/home/mboukhed/devel/GAIT_SOT_OLI/ospi/models/whole_body/obj'
# The path to the model and the filename
filename='/home/mboukhed/devel/GAIT_SOT_OLI/ospi/models/whole_body/wholebodyOLI.osim'
# Create a wrapper specific to the whole-body model 
# The wrapper parse the OpenSim model and builds pinocchio model and data
wb_model = wr.Wrapper(filename, mesh_path, name='whole-body_model10')


# call the gepetto viewer server
gvs = subprocess.Popen(["./gepetto-viewer.sh","&"])
print 'Loading the viewer ...'
time.sleep(2)

# Init the viewer and add the model to it 
viewer = vw.Viewer('viewer',wb_model)
viewer.setVisibility(wb_model.name+"/floor", "ON")
viewer.display(wb_model.q0, wb_model.name)



#See axis
#viewer.JointFrames(wb_model.name)

# parse motion:
time, q, colheaders, qOsim = mtp.parseMotion(wb_model.model, wb_model.joint_transformations, '/home/mboukhed/devel/GAIT_SOT_OLI/ospi/OLI_F_3.mot','quat')



t  = 0.0
def playMotions(first=0, last=1, step=3, t=0):
    for i in range(first, last, step):
        viewer.setVisibility("OLI", "ON")
        viewer.display(q[i].T, wb_model.name)
       

#time.sleep(4)
playMotions(0,396,1,0.0025) 
