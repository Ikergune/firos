import rospy

from include.ros import topicData

class TopicHandler:
    @staticmethod
    def publish(topic, datatype, value):
        pub = rospy.Publisher(topic, topicData)
        print pub