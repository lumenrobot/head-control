from naoqi import ALProxy
import time
import almath

posture = ALProxy("ALRobotPosture","169.254.108.110",9559)
motion = ALProxy("ALMotion","169.254.108.110",9559)
tts = ALProxy("ALTextToSpeech","169.254.108.110",9559)
posture.goToPosture("Stand",0.5)
motion.setAngles(['RShoulderPitch','RShoulderRoll'],[-60*almath.TO_RAD,-30*almath.TO_RAD],0.3)
time.sleep(1.0)
tts.say("hello")
motion.setAngles('RElbowRoll',88*almath.TO_RAD,0.3)
time.sleep(1.0)
motion.setAngles('RElbowRoll',0*almath.TO_RAD,0.3)
time.sleep(1.0)
motion.setAngles('RElbowRoll',88*almath.TO_RAD,0.3)
time.sleep(1.0)
motion.setAngles('RElbowRoll',0*almath.TO_RAD,0.3)
time.sleep(1.0)
motion.setAngles('RElbowRoll',88*almath.TO_RAD,0.3)
time.sleep(1.0)
motion.setAngles('RElbowRoll',0*almath.TO_RAD,0.3)
time.sleep(1.0)
motion.setAngles('RElbowRoll',88*almath.TO_RAD,0.3)
time.sleep(1.0)
motion.setAngles('RElbowRoll',0*almath.TO_RAD,0.3)
time.sleep(1.0)
motion.setAngles('RElbowRoll',88*almath.TO_RAD,0.3)
time.sleep(1.0)
motion.setAngles('RElbowRoll',0*almath.TO_RAD,0.3)
time.sleep(1.0)
motion.rest()