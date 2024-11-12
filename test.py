import rospy
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient

def perform_motion():

  motion_client = SimpleActionClient('/play_motion', PlayMotionAction)
  motion_client.wait_for_server(rospy.Duration(5.0))
  rospy.loginfo("Server connection successful")

  goal = PlayMotionGoal()

  # use here the desired motion ID from the table above
  goal.motion_name = "wave"
  goal.skip_planning = False
  motion_client.send_goal(goal)
  rospy.loginfo("Movement done")

if __name__ == "__main__":
    try:
        rospy.init_node('test', anonymous=True)
        perform_motion()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"Error: {e}")
