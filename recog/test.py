import cv2
import numpy as np
import os,sys
from scipy.io.matlab.mio import loadmat
from scipy.misc import imresize
from scipy.spatial import distance
import glob



U = loadmat('u.mat');U = U['U']
V = loadmat('v.mat');V = V['V']
Ms = loadmat('ms.mat');Ms = Ms['Ms']
Mu = loadmat('mu.mat');Mu = Mu['Mu']
MM = loadmat('mm.mat');MM = MM['MM']

osebe = {0:'harry', 1:'ellen',2:'kim',3:'matt',4:'filip',5:'scarlett',6:'tina',7:'prevc'}

for sl in glob.glob("proper/*.jpg"):
    im = cv2.imread(sl)
    im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    
    #cv2.imshow("test",im)
    #cv2.waitKey()
    #print im[122,24]
    
    im = np.asarray(im)
    #print im[122,24]
    
    im = imresize(im, (128,128),'bilinear')
    #print im[122,24]
    im = np.reshape(im, (128*128,1),'F')
    #print im[3194]
    im = im.astype(np.float32)
    
    #print im.dtype   
    
    ty = np.dot(np.matrix.transpose(U),(im-Mu))
    #print ty
    ty = np.dot(np.matrix.transpose(V),(ty-MM))
    
    mi = 1e10
    
    #print ty
    
    for i in xrange(0,8):
        cr = Ms[:,i]
        #print cr
        dist = distance.euclidean(ty,cr)
        #print dist
        if dist<mi:
            mind = i
            mi = dist
    
    print sl, osebe[mind]

