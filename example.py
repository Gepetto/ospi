import sys
sys.path.append('src')
import viewer_utils as vw
import wrapper as wr
import motion_parser as mtp

# The path to the model meshes
mesh_path='/local/gmaldona/devel/Models/data/whole_body/obj'
# The path to the model and the filename
filename='/local/gmaldona/devel/Models/data/whole_body/wholebody.osim'
# Create a wrapper specific to the whole-body model 
# The wrapper parse the OpenSim model and builds pinocchio model and data
wb_model = wr.Wrapper(filename, mesh_path, name='whole-body_model1')
# Init the viewer and add the model to it 
viewer = vw.Viewer('viewer',wb_model)
viewer.setVisibility(wb_model.name+"/floor", "OFF")
viewer.display(wb_model.q0, wb_model.name)

# See kinematic ranges of motion

# parse motion:
time, q, colheaders, qOsim = mtp.parseMotion(wb_model.model, wb_model.joint_transformations, '/home/gmaldona/Dropbox/kv03_filtered_ik.mot','quat')

