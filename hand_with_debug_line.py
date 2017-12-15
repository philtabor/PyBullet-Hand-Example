import pybullet as p
p.connect(p.GUI)

obj_to_classify = p.loadURDF("loader.stl.urdf",(0,-1,0), useFixedBase=1)

move = 0.1

p.setGravity(0,0,0)

objects = p.loadMJCF("MPL.xml",flags=0)
hand=objects[0]  #1 total
obj_po = p.getBasePositionAndOrientation(obj_to_classify)
hand_cid = p.createConstraint(hand,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])

hand_po = p.getBasePositionAndOrientation(hand)
ho = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
pi = 3.14159

# keeps it from moving back go origin
p.changeConstraint(hand_cid,(hand_po[0][0],hand_po[0][1],hand_po[0][2]),ho, maxForce=200)

p.setRealTimeSimulation(0)

offset = 0.02
indexEndID = 21 # Need get position and orientation from index finger parts
def ahead_view():
    link_state = p.getLinkState(hand,indexEndID)
    link_p = link_state[0]
    link_o = link_state[1]
    handmat = p.getMatrixFromQuaternion(link_o)

    axisX = [handmat[0],handmat[3],handmat[6]]
    axisY = [-handmat[1],-handmat[4],-handmat[7]] # Negative Y axis
    axisZ = [handmat[2],handmat[5],handmat[8]]

    eye_pos    = [link_p[0]+offset*axisY[0],link_p[1]+offset*axisY[1],link_p[2]+offset*axisY[2]]
    target_pos = [link_p[0]+axisY[0],link_p[1]+axisY[1],link_p[2]+axisY[2]] # target position based by axisY, not X
    up = axisZ # Up is Z axis
    viewMatrix = p.computeViewMatrix(eye_pos,target_pos,up)

    p.addUserDebugLine(link_p,[link_p[0]+0.1*axisY[0],link_p[1]+0.1*axisY[1],link_p[2]+0.1*axisY[2]],[1,0,0],2,0.05) # Debug line in camera direction    

    return viewMatrix

# Curl up the digits to make a pointing gesture. To remove other fingers from the equation
joint7Pos = p.getJointState(hand, 7)[0]
joint24Pos = p.getJointState(hand, 24)[0]
joint32Pos = p.getJointState(hand, 32)[0]
joint40Pos = p.getJointState(hand, 40)[0]
        
# thumb
joint7tgt, joint9tgt, joint11tgt, joint13tgt = pi/4, pi/2, pi/2, pi/2

# middle finger
joint24tgt, joint26tgt, joint28tgt = pi/2.25, pi/2, pi/2

 # ring finger
joint32tgt, joint34tgt, joint36tgt = pi/2.5, pi/2, pi/2

# pinky
joint40tgt, joint42tgt, joint44tgt = pi/3, pi/2, pi/2 

#This is to curl the hand into a pointing gesture
while (joint7Pos < joint7tgt) and (joint24Pos < joint24tgt) and (joint32Pos < joint32tgt) and (joint40Pos < joint40tgt):

    p.setJointMotorControl2(hand,7,p.POSITION_CONTROL,joint7tgt) 
    p.setJointMotorControl2(hand,9,p.POSITION_CONTROL,joint9tgt)
    p.setJointMotorControl2(hand,11,p.POSITION_CONTROL,joint11tgt)
    p.setJointMotorControl2(hand,13,p.POSITION_CONTROL,joint13tgt)

    p.setJointMotorControl2(hand,24,p.POSITION_CONTROL,joint24tgt)
    p.setJointMotorControl2(hand,26,p.POSITION_CONTROL,joint26tgt)
    p.setJointMotorControl2(hand,28,p.POSITION_CONTROL,joint28tgt)

    p.setJointMotorControl2(hand,32,p.POSITION_CONTROL,joint32tgt)
    p.setJointMotorControl2(hand,34,p.POSITION_CONTROL,joint34tgt)
    p.setJointMotorControl2(hand,36,p.POSITION_CONTROL,joint36tgt)
            
    p.setJointMotorControl2(hand,40,p.POSITION_CONTROL,joint40tgt)
    p.setJointMotorControl2(hand,42,p.POSITION_CONTROL,joint42tgt)
    p.setJointMotorControl2(hand,44,p.POSITION_CONTROL,joint44tgt)
 
    p.stepSimulation()

    joint7Pos = p.getJointState(hand, 7)[0]
    joint24Pos = p.getJointState(hand, 24)[0]
    joint32Pos = p.getJointState(hand, 32)[0]
    joint40Pos = p.getJointState(hand, 40)[0]

# need this to keep the hand from rotating when you move it up and down.
p.changeDynamics(hand, linkIndex=-1, angularDamping=35.0)

# Main loop for controlling the hand
while (1):
    key = p.getKeyboardEvents()

    for k in key.keys():
        hand_po = p.getBasePositionAndOrientation(hand)
        print(k)
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
    viewMatrix = ahead_view()
    p.stepSimulation()  

p.disconnect()