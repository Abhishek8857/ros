import rospy

rospy.init_node('temp')

state = rospy.Time.now()
for i in range(100):
    print(rospy.Time.now())
    state = rospy.Time.now() + rospy.Duration(30)
    print(state)
    
