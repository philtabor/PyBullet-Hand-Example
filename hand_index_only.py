import pybullet as p
p.connect(p.GUI)

obj_to_classify = p.loadURDF("loader.stl.urdf",(0,-1,0), useFixedBase=1)

move = 0.00015

p.setGravity(0,0,0)

objects = p.loadMJCF("MPL_index_only.xml",flags=0)
hand=objects[0]  #1 total
obj_po = p.getBasePositionAndOrientation(obj_to_classify)
hand_cid = p.createConstraint(hand,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])

hand_po = p.getBasePositionAndOrientation(hand)
ho = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
pi = 3.14159
# keeps it from moving back go origin
p.changeConstraint(hand_cid,(hand_po[0][0],hand_po[0][1],hand_po[0][2]),ho, maxForce=200)

p.setRealTimeSimulation(0)

# Main loop for controlling the hand
while (1):
    key = p.getKeyboardEvents()

    for k in key.keys():
        hand_po = p.getBasePositionAndOrientation(hand)

        if k == 48: #down - 0 key on keyboard
            p.changeConstraint(hand_cid,(hand_po[0][0]+move,hand_po[0][1],hand_po[0][2]),ho, maxForce=50)
        elif k == 49: #up   1 key         
            p.changeConstraint(hand_cid,(hand_po[0][0]-move,hand_po[0][1],hand_po[0][2]),ho, maxForce=50)
        elif k == 50: #left 2 key           
            p.changeConstraint(hand_cid,(hand_po[0][0],hand_po[0][1]+move,hand_po[0][2]),ho, maxForce=50)
        elif k == 51: #right 3 key           
            p.changeConstraint(hand_cid,(hand_po[0][0],hand_po[0][1]-move,hand_po[0][2]),ho, maxForce=50)
        elif k == 52: #< 4 key       
            p.changeConstraint(hand_cid,(hand_po[0][0],hand_po[0][1],hand_po[0][2]+move), ho, maxForce=50)            
        elif k == 53: #> 5 key           
            p.changeConstraint(hand_cid,(hand_po[0][0],hand_po[0][1],hand_po[0][2]-move), ho, maxForce=50)
  
    p.stepSimulation()  

p.disconnect()