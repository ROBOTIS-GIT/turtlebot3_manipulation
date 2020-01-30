<launch>
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [waffle, waffle_pi]"/>
  <arg name="use_robot_name" default="om_with_tb3"/>

  <param name="robot_description"
    command="$(find xacro)/xacro --inorder '$(find open_manipulator_with_tb3_description)/urdf/open_manipulator_with_tb3_$(arg model).urdf.xacro'"/>

  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen">
    <param name="publish_frequency" type="double" value="100.0" />
    <param name="tf_prefix" value="$(arg use_robot_name)" />
  </node>
</launch>
