#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonInteface(object):
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)


    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0 # 0 + 1.5 = 0.4
    joint_goal[2] = 0 # BB_Front_Boom to BB -0.4
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0
    joint_goal[6] = 0
    joint_goal[7] = 0.2
    joint_goal[8] = 0
    joint_goal[9] = 0
    joint_goal[10] = 0
    joint_goal[11] = 0
    joint_goal[12] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self, pos_x, pos_y):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    print pose_goal
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = pos_x
    pose_goal.position.y = pos_y
    #pose_goal.position.z = 0.4
    #xyz=[130,40,0]

    #move_group.clear_pose_targets()
    #move_group.set_position_target(xyz, "bwe")
    #move_group.set_goal_position_tolerance(1)
    #move_group.set_planning_time(10)

    ## Now, we call the planner to compute the plan and execute it.
    move_group.set_planning_time(20)
    plan = move_group.go(pose_goal, wait=True)

    print self.move_group.get_current_pose().pose

    print move_group.get_end_effector_link()
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def go_to_position(self, pos_x, pos_y):
    waypoints = []
    wpose = self.move_group.get_current_pose().pose
    wpose.position.x = pos_x  # First move up (z)
    wpose.position.y = pos_y  # First move up (z)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 1, 0.0) # waypoints to follow # 0.1  # eef_step # 0.0  # jump_threshold

    self.move_group.set_goal_tolerance(1)
    self.move_group.execute(plan, wait=True)

  def go_to_rel_position(self, inc_x, inc_y):   # inc = increment
    waypoints = []
    wpose = self.move_group.get_current_pose().pose
    wpose.position.x += inc_x  # First move up (z)
    wpose.position.y += inc_y  # First move up (z)
    #waypoints.append(copy.deepcopy(wpose))

    #self.go_to_pose_goal(wpose.position.x, wpose.position.y)
    self.go_to_pose_goal(inc_x, inc_y)

    #(plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.1, 0.0) # waypoints to follow # 0.1  # eef_step # 0.0  # jump_threshold

    #self.move_group.set_goal_tolerance(1)
    #self.move_group.execute(plan, wait=True)

  def plan_cartesian_path(self, scale=10):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.x += 50  # First move up (z)
    waypoints.append(copy.deepcopy(wpose))
    #wpose.position.y -= 30  # First move up (z)
    #waypoints.append(copy.deepcopy(wpose))
    #
    # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    # waypoints.append(copy.deepcopy(wpose))
    #
    # wpose.position.y -= scale * 0.1  # Third move sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.1,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL


  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.set_goal_tolerance(1)
    print str(move_group.get_end_effector_link())
    print str(move_group.get_goal_tolerance())
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Received
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def add_enviroment(self, timeout=4):

    print "Preparing to build belt"
    raw_input()
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = self.robot.get_planning_frame()
    p.pose.position.x = 100.
    p.pose.position.y = -24.
    p.pose.position.z = -2.
    self.scene.add_box("belt", p, (200, 50, 1))

    print "Preparing to build mountain 1"
    raw_input()
    p.pose.position.x = 35.
    p.pose.position.y = 92.
    p.pose.position.z = 0.
    self.scene.add_box("mountain1", p, (70, 100, 1))

    print "Preparing to build mountain 2"
    raw_input()
    p.pose.position.x = 137.5
    p.pose.position.y = 92.
    p.pose.position.z = 0.
    self.scene.add_box("mountain2", p, (105, 100, 1))

    print "Preparing to build mountain 3"
    raw_input()
    p.pose.position.x = 77.5
    p.pose.position.y = 97.5
    p.pose.position.z = 0.
    self.scene.add_box("mountain3", p, (15, 85, 1))

    print "Box should be there"

    return

def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to IRMS"
    print "The Intelligent Resource Management System"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin setting up the transfer point allignment and the enviroment..."
    raw_input()
    tpa = MoveGroupPythonInteface()
    print "Created TPA object"
    tpa.go_to_joint_state()
    print "Initial joint state"
    tpa.add_enviroment()
    print "Finished adding environment"

    print ""
    print "=========================================="
    print "+  Transfer point allignment SUCCESFULL  +"
    print "=========================================="
    print ""

    print "============ Press `Enter` to move the BWE to the rib ..."
    #tpa.go_to_position(40, 40)
    print "tpa.go_to_pose_goal(50, 40)"
    raw_input()
    tpa.go_to_pose_goal(50, 40)  # Motion Planner
    #tpa.go_to_pose_goal(73, 50)

    print "============ Press `Enter` to start long travel working cycle ..."
    raw_input()
    #tpa.go_to_pose_goal(75, 40)
    tpa.go_to_position(75, 40)  # No Motion Planner => Cartesian Path
    #cartesian_plan, fraction = tpa.plan_cartesian_path()
    #tpa.execute_plan(cartesian_plan)

    # while schleife boxcut
    print "============ Press `Enter` to start manual box cut ..."
    raw_input()

    print "============ Press `x` and `Enter` to end manual box cut ..."
    print "============ Press `w, a, s, d` to controll BWE followed by `Enter`"
    # raw_input()

    x = 75
    y = 40

    while True:
      print "Waiting for input..."
    
      key = raw_input()
    
      if key == "w":
        y += 2
      elif key == "a":
        x += -2
      elif key == "s":
        y += -2
      elif key == "d":
        x += 2
      elif key == "x":
        break
      else:
        print "Wrong input"
    
      #tpa.go_to_rel_position(x, y)
      tpa.go_to_position(x, y)

    # bei abbruchbeddingung zuruck zu ausgangsposizion
    # parallelbetrieb

    #tpa.scene.clear()

    print "============ Press `Enter` continuw long travel working cycle ..."
    raw_input()
    tpa.go_to_pose_goal(80, 40)
    tpa.go_to_position(180, 40)

    print "============ Python IRMS demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

