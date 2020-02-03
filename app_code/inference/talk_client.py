import rospy
from roboy_cognition_msgs.srv import Talk

def talk_client(talk_str):
    rospy.wait_for_service('/roboy/cognition/speech/synthesis/talk', timeout=5)
    try:
        tts = rospy.ServiceProxy('/roboy/cognition/speech/synthesis/talk', Talk)
        resp = tts(talk_str)
        return resp
    except rospy.ServiceException as e:
        print ("Service call failed:")
        print (e)