

regex = re.compile(ur'^\/([\w]+)\/*([\/\-\w]*)$')
robots = {}

class RosConfigurator:

    @staticmethod
    def topic_type(t, topic_types):
        matches = [t_type for t_name, t_type in topic_types if t_name == t]
        if matches:
            return matches[0].replace("msgs/", "msgs.msg.").replace("/", ".msg.")
        return 'unknown type'

    @staticmethod
    def setRobot(topic, t_type, pubsub):
        matching = re.search(regex, topic)
        robot_topic = matching.group(2)
        if robot_topic != '':
            robot_name = matching.group(1)
            if robot_name not in robots:
                robots[robot_name] = {"topics": []}
            robots[robot_name]["topics"].append({
                "name": robot_topic,
                "msg": t_type,
                "type": pubsub
            })

    @staticmethod
    def systemTopics():
        master = rosgraph.Master('/rostopic')
        try:
            state = master.getSystemState()
            topic_types = rostopic._master_get_topic_types(master)

            pubs, subs, _ = state

            # ROS publisher --> firos subscribes to listen data published
            for t, l in pubs:
                _type = RosConfigurator.topic_type(t, topic_types)
                RosConfigurator.setRobot(t,_type, "subscriber")

            # ROS subscriber --> firos publishes data to them
            for t, l in subs:
                _type = RosConfigurator.topic_type(t, topic_types)
                RosConfigurator.setRobot(t,_type, "publisher")

            # print robots
            print json.dumps(robots)

        except socket.error:
            raise rostopic.ROSTopicIOException("Unable to communicate with master!")