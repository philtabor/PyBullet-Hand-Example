import pybullet as pb

pb.connect(pb.GUI)
pb.setGravity(0,0,0)
pb.setRealTimeSimulation(0)
xyz=[0.25, 0.25, 0]
wandLength = 0.3
wandSide = 0.005
orientation = [0,1,0,1]
move = 10e-3
#agent = pb.createCollisionShape(pb.GEOM_BOX,
#                                halfExtents=[wandSide,
#                                             wandSide,
#                                             wandLength])

agent_finger = pb.createCollisionShape(pb.GEOM_BOX,
                                       halfExtents=[wandSide,
                                       wandSide,
                                       0.1])

agent_joint = pb.createCollisionShape(pb.GEOM_SPHERE,
                                      radius=0.025)


#agent_mb = pb.createMultiBody(baseMass=10,
#                              baseCollisionShapeIndex=agent,
#                              baseVisualShapeIndex=-1,
#                              basePosition=xyz,
#                              baseOrientation=orientation)

agent_joint_mb = pb.createMultiBody(baseMass=10,
                                    baseCollisionShapeIndex=agent_joint,
                                    baseVisualShapeIndex=-1,
                                    basePosition=[0.525,0.25,0],
                                    baseOrientation=orientation)


agent_finger_mb = pb.createMultiBody(baseMass=10,
                                     baseCollisionShapeIndex=agent_finger,
                                     baseVisualShapeIndex=-1,
                                     basePosition=[0.625, 0.25, 0],
                                     baseOrientation=orientation)

"""
agent_mb = pb.createMultiBody(baseMass=10,
                              baseCollisionShapeIndex=agent,
                              baseVisualShapeIndex=-1,
                              basePosition=xyz,
                              baseOrientation=orientation,
                              linkMasses=[0.5, 0.5],
                              linkCollisionShapeIndices=[agent_joint, agent_finger],
                              linkVisualShapeIndices=[-1, -1],
                              linkPositions=[[0, 0, 0.305], [0, 0, 0.405]],
                              linkOrientations=[[0,1,0,0], [0,1,0,0]],
                              linkInertialFramePositions=[[0,0,0], [0,0,0]],
                              linkInertialFrameOrientations=[[1,0,0,0], [0,0,0,1]],
                              linkParentIndices=[0, 0],
                              linkJointTypes=[pb.JOINT_REVOLUTE, pb.JOINT_FIXED],
                              linkJointAxis=[[0,1,0], [0,1,0]])
"""
"""
agent_mb = pb.createMultiBody(baseMass=10,
                              baseCollisionShapeIndex=agent,
                              baseVisualShapeIndex=-1,
                              basePosition=xyz,
                              baseOrientation=orientation,
                              linkMasses=[0.5],
                              linkCollisionShapeIndices=[agent_finger],
                              linkVisualShapeIndices=[-1],
                              linkPositions=[ [0, 0, 0.405]],
                              linkOrientations=[[0,1,0,0]],
                              linkInertialFramePositions=[[0,0,0]],
                              linkInertialFrameOrientations=[[0,0,0,1]],
                              linkParentIndices=[ 0],
                              linkJointTypes=[pb.JOINT_REVOLUTE],
                              linkJointAxis=[[0,1,0]])
"""
"""
agent_mb = pb.createMultiBody(baseMass=10,
                              baseCollisionShapeIndex=agent_joint,
                              baseVisualShapeIndex=-1,
                              basePosition=xyz,
                              baseOrientation=orientation,
                              linkMasses=[0.5],
                              linkCollisionShapeIndices=[agent_finger],
                              linkVisualShapeIndices=[-1],
                              linkPositions=[ [0, 0, 0.1]],
                              linkOrientations=[[0,1,0,0]],
                              linkInertialFramePositions=[[0,0,0]],
                              linkInertialFrameOrientations=[[0,0,0,1]],
                              linkParentIndices=[ 0],
                              linkJointTypes=[pb.JOINT_REVOLUTE],
                              linkJointAxis=[[0,1,0]])
"""
pb.changeVisualShape(agent_joint_mb,-1,rgbaColor=[1,1,0,1])
"""
agent_cid = pb.createConstraint(agent_mb,-1,-1,-1,pb.JOINT_FIXED,
                                [0,0,0],[0,0,0],xyz,
                                childFrameOrientation=orientation)
"""
joint_cid = pb.createConstraint(agent_joint_mb, -1, agent_finger_mb, -1, pb.JOINT_FIXED,
                                [0,0,0], [0,0,0], [0,0,0],#[0.525,0.25,0],
                                childFrameOrientation=orientation)

"""

c = p.createConstraint(wheelA,1,wheelA,3,jointType=p.JOINT_GEAR,
                       jointAxis =[0,1,0],parentFramePosition=[0,0,0],
                       childFramePosition=[0,0,0])

p.changeConstraint(c,gearRatio=1, maxForce=10000)
"""
def step(action):
        x, y, z = pb.getBasePositionAndOrientation(agent_joint_mb)[0]

        if action == 0: #down
            x += move
        elif action == 1: #up
            x -= move
        elif action == 2: #left
            y += move
        elif action == 3: #right
            y -= move
        elif action == 4: #<
            z += move
        elif action == 5: #>
            z -= move

        elif action == 6: # move finger
            joint1Pos = pb.getJointState(agent_joint_mb, 0)[0]
            newJoint1Pos = joint1Pos - 0.001
            pb.setJointMotorControl2(agent_joint_mb,0,pb.POSITION_CONTROL,newJoint1Pos)

        elif action == 7: # move finger
            joint1Pos = pb.getJointState(agent_joint_mb, 0)[0]
            newJoint1Pos = joint1Pos + 0.001
            pb.setJointMotorControl2(agent_joint_mb,0,pb.POSITION_CONTROL,newJoint1Pos)

        pivot = [x,y,z]
        orn = pb.getQuaternionFromEuler([0,0,0])
        pb.changeConstraint(joint_cid,pivot,
                            jointChildFrameOrientation=[0,1,0,1],
                            maxForce=0.005)
        pb.stepSimulation()

while(1):
    key = pb.getKeyboardEvents()
    for key in key.keys():
        action = key - 48
        step(action)
