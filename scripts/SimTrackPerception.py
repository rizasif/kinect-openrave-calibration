#!/usr/bin/env python

# Import required Python code.
import roslib, rospy, tf, tf_conversions, threading
import numpy as np
import yaml
import sys
from scipy.spatial.transform import Rotation as R


# cameraPose = np.array([
#        [ np.cos(np.pi),  -np.sin(np.pi), 0.0,  0.0   ],
#        [ np.sin(np.pi),  np.cos(np.pi), 0.0,  0.0   ],
#        [ 0.0,  0.0, 1.0,  0.0   ],
#        [ 0.0,  0.0, 0.0,  1.0   ]])

cameraPose = np.array([
       [ 1.0,  0.0, 0.0,  0.0   ],
       [ 0.0,  1.0, 0.0,  0.0   ],
       [ 0.0,  0.0, 1.0,  0.0   ],
       [ 0.0,  0.0, 0.0,  1.0   ]])


# cameraPose = np.array([[-0.855453,  0.059819, -0.514415,  0.580475],
#        [ 0.506901,  0.300174, -0.808052,  1.214   ],
#        [ 0.106077, -0.952007, -0.287107,  0.3435  ],
#        [ 0.      ,  0.      ,  0.      ,  1.      ]])


# cameraPose = numpy.array([[-0.855453,  0.059819, -0.514415,  0.573975],
#        [ 0.506901,  0.300174, -0.808052,  1.215   ],
#        [ 0.106077, -0.952007, -0.287107,  0.345   ],
#        [ 0.      ,  0.      ,  0.      ,  1.      ]])

# cameraPose = numpy.array([[-0.855453,  0.059819, -0.514415,  0.5885  ],
#        [ 0.506901,  0.300174, -0.808052,  1.2062  ],
#        [ 0.106077, -0.952007, -0.287107,  0.321   ],
#        [ 0.      ,  0.      ,  0.      ,  1.      ]])


# cameraPose = numpy.array([[-0.855453,  0.059819, -0.514415,  0.56    ],
#        [ 0.506901,  0.300174, -0.808052,  1.195   ],
#        [ 0.106077, -0.952007, -0.287107,  0.334   ],
#        [ 0.      ,  0.      ,  0.      ,  1.      ]])

#Virgile's matrix:
# cameraPose = numpy.array([[-0.855453, 0.059819, -0.514415, 0.573975], [0.506901, 0.300174, -0.808052, 1.22227], [0.106077, -0.952007, -0.287107, 0.304756], [0, 0, 0, 1]])

class SimTrackPerception:
    def __init__(self, orEnv, objectNames, dataRootPath, objectParams, rootFrame = '/kinect2_rgb_optical_frame', transform =  cameraPose):
        self._orEnv = orEnv
        self._objectNames = objectNames
        self._dataPath = dataRootPath
        self._isRunning = False
        self._listener = None
        self._rootFrame = rootFrame
        self._thread = None
        self._transform = transform
        self.ignoreList = []
        self.objectParams = objectParams
        self.frames, self.frame_update, self.frame_transform = self._makeFrames()
        # self._transform[0,0:3] = numpy.cross(transform[2,:3], transform[1,:3])
        # self._transform = numpy.eye(4,4)
        # self._transform[:3, 3] = numpy.array([0.573975, 1.22227, -0.287107])
        # self.drawer = utils.OpenRAVEUtils()

    def _makeFrames(self):
        frames = {}
        last_updated = {}
        transform = {}
        for obj in self._objectNames:
            markers = self.objectParams[obj]['markers']
            frames[obj] = []
            for i in range(len(markers)):
                m = markers[i]
                id = obj + "_" + str(m)
                frames[obj].append(id)
                last_updated[id] = rospy.Time.from_sec(0.0)

                # dimentions  = np.array(self.objectParams[obj]['dimentions'])
                marker_trans = np.array(self.objectParams[obj]['marker_trans'][i])
                marker_rot = np.array(self.objectParams[obj]['marker_rot'][i])*(np.pi/180.0)
                
                quat = tf.transformations.quaternion_from_euler(marker_rot[0], marker_rot[1], marker_rot[2])
                marker_rot = R.from_quat(quat).as_dcm()

                transform[id] = [marker_trans, marker_rot]
        return frames, last_updated, transform


    def start(self):
        if self._isRunning:
            print 'Perception is already running'
            return
        rospy.init_node('simtrackListener', anonymous = True)
        self._listener = tf.TransformListener()
        self._isRunning = True
        self._thread = threading.Thread(target=self._run)
        self._thread.start()

    def _run(self):
        rate = rospy.Rate(120.0)
        while not rospy.is_shutdown() and self._isRunning:
            # Do shit
            for obj in self._objectNames:
                if obj in self.ignoreList:
                    continue
                
                frame_ids = []
                for frame in self.frames[obj]:
                    try:
                        last_update = self._listener.getLatestCommonTime(self._rootFrame, frame)
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue
                    if self.frame_update[frame] < last_update and not last_update is None:
                        if len(frame_ids) > 0 and last_update > self.frame_update[frame_ids[0]]:
                            frame_ids.insert(0, frame)
                            self.frame_update[frame] = last_update
                        else:
                            frame_ids.append(frame)
                            self.frame_update[frame] = last_update
                            # break
                if len(frame_ids) == 0:
                    continue
                
                with self._orEnv:
                    body = self._orEnv.GetKinBody(self.objectParams[obj]['name'])
                    if body is None:
                        bodyLoaded = self._orEnv.Load(self._dataPath + "/" + obj + "/" + obj + ".kinbody.xml")
                        if bodyLoaded:
                            body = self._orEnv.GetKinBody(self.objectParams[obj]['name'])
                        else:
                            print("Could not load %s." % self._dataPath + "/" + obj + "/" + obj + ".kinbody.xml")
                            break
                
                frame_trans = np.array([0.0,0.0,0.0])
                frame_rot = None
                marker_trans = np.array([0.0,0.0,0.0])
                marker_rot = []
                divider = 0.0
                if self.objectParams[obj]['sync'] == True:
                    for frame_id in frame_ids:
                        (trans, rot) = self._listener.lookupTransform(self._rootFrame, frame_id, rospy.Time(0))
                        frame_trans = frame_trans + np.array(trans)
                        frame_rot = np.array(rot)
                        marker_trans = marker_trans + np.array(self.frame_transform[frame_id][0])
                        marker_rot = np.array(self.frame_transform[frame_id][1])
                        divider += 1.0
                else:
                    frame_id = frame_ids[0]
                    (trans, rot) = self._listener.lookupTransform(self._rootFrame, frame_id, rospy.Time(0))
                    frame_trans = frame_trans + np.array(trans)
                    frame_rot = np.array(rot)
                    marker_trans = marker_trans + np.array(self.frame_transform[frame_id][0])
                    marker_rot = np.array(self.frame_transform[frame_id][1])
                    divider += 1.0
                    
                assert(not frame_rot is None)

                marker_trans = marker_trans/divider
                frame_trans = frame_trans/divider

                # dimentions  = np.array(self.objectParams[obj]['dimentions'])
                
                rot =  R.from_quat(frame_rot).as_dcm()
                rot =  np.dot(rot, marker_rot)
                # trans = frame_trans + np.multiply(marker_trans, dimentions)
                trans = frame_trans + marker_trans
                trans[1] = trans[1]*-1
                trans[0] = trans[0]*-1
                # if self.objectParams[obj]['invert_x'] == False:
                    # trans[0] = trans[0]*-1    
                

                # matrix = tf_conversions.transformations.quaternion_matrix([rot[0], rot[1], rot[2], rot[3]])
                matrix = np.zeros(16).reshape(4,4)
                matrix[3][3] = 1
                matrix[:3,:3] = rot
                matrix[:3, 3] = trans

                matrix = np.dot(self._transform, matrix)

                body.SetTransform(matrix)
            rate.sleep()

    def enableObject(self, objName, enable = True):
        if enable:
            if objName in self.ignoreList:
                self.ignoreList.remove(objName)
        else:
            if objName not in self.ignoreList:
                self.ignoreList.append(objName)

    def end(self):
        if self._isRunning:
            self._isRunning = False
            self._thread.join()

# Main function.
if __name__ == '__main__':
    import openravepy, IPython
    env = openravepy.Environment()
    env.SetViewer('qtcoin')
    objectNames = ['elmers_glue', 'cabinet', 'cracker_box', 'sugar_box', 'mustard']
    dataRootPath = '/home/rizwan/projects/placement_ws/src/hfts_grasp_planner/data'
    
    #Load Yaml files
    yamls = {}
    for obj in objectNames:
        try:
            with open('../data/' + obj +".yaml", 'r') as f:
                y = yaml.load(f)
        except:
            print("Error loading yaml")
            sys.exit(0)
        finally:
            yamls[obj] = y
    # print yamls
    perception = SimTrackPerception(env, objectNames, dataRootPath, yamls)
    perception.start()
    IPython.embed()
    perception.end()



