# 08-20-12 OpenRave Tutorial Test Code 
# Test Puma Robot exmaples

from openravepy import *
import time
from numpy import *

select = 1
turnon = 1
print 'start'
if  select == 1:     

    try:
	
	env = Environment()
	env.SetViewer('qtcoin')
	env.Load('data/puma_tabletop.env.xml') # load a simple scene
	robot = env.GetRobots()[0] # get the first robot
        robot.SetDOFValues(0*array([1,1,1,1,1,1,1]),[0,1,2,3,4,5,6])
	T0 = robot.GetLinks()[0].GetTransform() # get the transform of link 1
    	T1 = robot.GetLinks()[1].GetTransform() # get the transform of link 2
    	T2 = robot.GetLinks()[2].GetTransform() # get the transform of link 3
    	T3 = robot.GetLinks()[3].GetTransform() # get the transform of link 4
    	T4 = robot.GetLinks()[4].GetTransform() # get the transform of link 5
    	T5 = robot.GetLinks()[5].GetTransform() # get the transform of link 6
    	handles=[]
    	handles.append(misc.DrawAxes(env,T0,0.3,3))
    	handles.append(misc.DrawAxes(env,T1,0.3,3))
    	handles.append(misc.DrawAxes(env,T2,0.3,3))
    	handles.append(misc.DrawAxes(env,T3,0.3,3))
    	handles.append(misc.DrawAxes(env,T4,0.3,3))
    	handles.append(misc.DrawAxes(env,T5,0.3,3))
	time.sleep(10)
	
	if turnon == 1:
		for k in range(0,5):	
			for i in range(0,180):
			    t = -pi+(i*pi/180)
			    robot.SetDOFValues([t],[k]) # set joint 0 to value 0.5
			    T0 = robot.GetLinks()[0].GetTransform() # get the transform of link 1
		            T1 = robot.GetLinks()[1].GetTransform() # get the transform of link 2
			    T2 = robot.GetLinks()[2].GetTransform() # get the transform of link 3
		            T3 = robot.GetLinks()[3].GetTransform() # get the transform of link 4
			    T4 = robot.GetLinks()[4].GetTransform() # get the transform of link 5
			    T5 = robot.GetLinks()[5].GetTransform() # get the transform of link 6
			    handles=[]
			    handles.append(misc.DrawAxes(env,T0,0.3,3))
			    handles.append(misc.DrawAxes(env,T1,0.3,3))
		            handles.append(misc.DrawAxes(env,T2,0.3,3))
			    handles.append(misc.DrawAxes(env,T3,0.3,3))
			    handles.append(misc.DrawAxes(env,T4,0.3,3))
			    handles.append(misc.DrawAxes(env,T5,0.3,3))
			    time.sleep(0.05) 
			    print T5
 
		for i in range(0,180):
		    t = -pi+(i*pi/180)
		    robot.SetDOFValues([t],[6]) # set joint 0 to value 0.5
		    time.sleep(0.05) 
		    print T5 	

	body = env.ReadKinBodyXMLFile(filename='data/puma_tabletop.env.xml')
        env.AddKinBody(body)
        body.SetTransform([1,0,0,0,0,0,0])
        pose = poseFromMatrix(body.GetTransform())
  	
	robot.drawarrow(p1=[0.0,0.0,0.0],p2=[500,0.0,0.0],linewidth=0.01,color=[1.0,0.0,0.0])
        handles=[]
        handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[500,0.0,0.0],linewidth=0.01,color=[1.0,0.0,0.0]))
        handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.0,500,0.0],linewidth=0.01,color=[0.0,1.0,0.0]))
        handles.append(env.drawarrow(p1=[0.0,0.0,0.0],p2=[0.0,0.0,500],linewidth=0.01,color=[0.0,0.0,0.1]))

	raw_input("")
	with env: # lock the environment since robot will be used
	    raveLogInfo("Robot "+robot.GetName()+" has "+repr(robot.GetDOF())+" joints with values:\n"+repr(robot.GetDOFValues()))
	    robot.SetDOFValues([0.5],[0]) # set joint 0 to value 0.5
	    T = robot.GetLinks()[1].GetTransform() # get the transform of link 1
	    raveLogInfo("The transformation of link 1 is:\n"+repr(T))
            time.sleep(10)
    finally:
	env.Destroy()
