<?xml version="1.0"?>
<launch>
        <machine name="drc3" address="drc3" user="youbot" timeout="30"
                 env-loader="/home/youbot/rosenv.sh" />
        <machine name="drc1" address="drc1" user="youbot" timeout="30" 
                 env-loader="/home/youbot/rosenv.sh" />

	<rosparam file="$(find assembly_common)/config/parameters.yaml" command="load" />

	<include file="$(find fleet_control)/launch/gripper_only_joy.launch" />


	<node name="fleet_master" pkg="fleet_control"
              type="fleet_master.py" output="screen" />


	<group ns="drc3">

		<node name="fleet_controller" pkg="fleet_control"
                      type="fleet_controller.py" output="screen" />

                <node machine="drc3" name="arm_feedback" pkg="robot"  type="arm_feedback.py" output="screen" />

                <!--node name="mini_cartesian" pkg="robot"  type="mini_cartesian.py" output="screen" /-->
                <node machine="drc3" name="mini_cartesian" pkg="robot"  type="mini_cartesian.py" output="log" />

                <node machine="drc3" name="arm" pkg="robot"  type="arm.py" output="log" />

                <node machine="drc3" name="vicon" pkg="vicon"  type="vicon.py" output="screen" />

                <node machine="drc3" name="text_to_speech" pkg="text_to_speech"
                      type="text_to_speech.py"/>

                <node machine="drc3" name="youbot_ik" pkg="fleet_control" type="youbot_ik.py" args="/home/youbot/ros_stacks/drl-youbot/fleet_control/openrave/models" />
	</group>

	<group ns="drc1">

		<node name="fleet_controller" pkg="fleet_control"
                      type="fleet_controller.py" output="screen" />

                <node machine="drc1" name="arm_feedback" pkg="robot"  type="arm_feedback.py" output="screen" />

                <!--node name="mini_cartesian" pkg="robot"  type="mini_cartesian.py" output="screen" /-->
                <node machine="drc1" name="mini_cartesian" pkg="robot"  type="mini_cartesian.py" output="log" />

                <node machine="drc1" name="arm" pkg="robot"  type="arm.py" output="log" />

                <node machine="drc1" name="vicon" pkg="vicon"  type="vicon.py" output="screen" />

                <node machine="drc1" name="text_to_speech" pkg="text_to_speech"
                      type="text_to_speech.py"/>

                <node machine="drc1" name="youbot_ik" pkg="fleet_control" type="youbot_ik.py" args="/home/youbot/ros_stacks/drl-youbot/fleet_control/openrave/models" />
	</group>
</launch>
