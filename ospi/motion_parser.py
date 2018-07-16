"""
.. module:: opensim parser compatible with numpy
       :platform: ubuntu
       :synopsis: parse OpenSim motion files into numpy matrices
.. moduleauthor:: Galo Maldonado <galo_xav@hotmail.com>
"""
import utils 
import os 
import xml.etree.ElementTree as xml
import numpy as np
import pinocchio as se3
from IPython import embed


def _readSto(filename, verbose=False):
    """This parser can be used with osim motion or force files 
    Note:
       Does the same as parseMot
    """
    return parseMot(filename, verbose)

def _readMot(filename, verbose=False):
    """This parser can be used with osim motion or force files 
    param filename: the complete filename. Accepted extensions are 
    ".mot" and "sto".
    type filename: str.
    param verbose: false by default.
    type: boolean.
    returns: time numpy array, data numpy matrix and column headers 
        list of data
    """
    data = []
    time = []
    file_extension = os.path.splitext(filename)[1][1:]
    
    # Verify extension is correct
    if file_extension not in ['mot', 'sto']:
        print('File extension is not recognized. Only OpenSim .mot and .sto files can be parsed')
        if (verbose): print 'File extension given is .'+file_extension
        return
    
    # Try to open the file
    try:
        f = open(filename, 'r')
    except IOError:
        print('cannot open', filename)
    
    # Read the file    
    with open(filename, 'r') as f:
        filename = f.readline().split()[0]
        if (verbose): print('Reading file: '+filename)

        # The header is not really informative
        while True:
            try:
                line = f.readline().split()[0]
            except IndexjError:
                line = f.readline()
                    
            if line[0:9] == 'endheader':
                break        

        # Get the colheaders
        if (verbose): print("Reading the colheaders")
        col_headers = f.readline().split()[1:]

        
        # Read time and data from file
        for rows in f:
            data.append(rows.split()[1:])
            time.append(float(rows.split()[0]))
        
        for rows in range (0,len(data[:])):
            for cols in range (0,len(data[0])):
                data[rows][cols] = float(data[rows][cols])
        
        return np.array(time), np.matrix(data), col_headers



def parseMotion(model, joint_transformations, filename, representation, verbose=False):
    time, qOsim, col_headers = _readMot(filename, verbose)
    # store rotations in radians, defualt in opensim is degrees
    qOsim_radians = np.hstack([np.deg2rad(qOsim[:,:3]), qOsim[:,3:6], np.deg2rad(qOsim[:,6:])])
    
    q=[]
    for t in xrange(0,time.size):
        q.append(utils.pinocchioCoordinates(model, joint_transformations, qOsim_radians[t,:].T, representation))
    return time, np.matrix(np.array(q).squeeze()), col_headers, qOsim

    

def parseForces(filename, verbose=False):
    '''
    Returned data has the shape : fse3[nsensors,nframes]
    input Opensim format must be [Fx, Fy, Fz, Px, Py, Pz, Tx, Ty, Tz]
    '''
    osMpi = se3.utils.rotate('z', np.pi/2) * se3.utils.rotate('x', np.pi/2)
    time, fOsim, colheaders = _readMot(filename, verbose)
    _, ncols = fOsim.shape
    counter = 0
    f = []
    p = []
    for i in xrange(0, ncols/9):
        ftime = []
        ptime = []
        for t in xrange(0,time.size):
            ptime.append(osMpi*fOsim[t,3+counter:6+counter].T)
            fse3 = se3.Force.Zero()
            fse3.linear = fOsim[t,0+counter:3+counter].T
            fse3.angular = fOsim[t,6+counter:9+counter].T
            ftime.append(fse3)
        f.append(ftime)
        p.append(ptime)
        counter += 9    
    return time, np.matrix(f), np.array(p).squeeze(), colheaders, fOsim 

