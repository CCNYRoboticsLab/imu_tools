<!-- ComplementaryFilter launch file -->
<launch>
  #### Nodelet manager ######################################################

  <node pkg="nodelet" type="nodelet" name="imu_manager" 
    args="manager" output="screen" />

  #### IMU Driver ###########################################################

  <node pkg="nodelet" type="nodelet" name="PhidgetsImuNodelet" 
    args="load phidgets_imu/PhidgetsImuNodelet imu_manager" 
    output="screen">

    # supported data rates: 4 8 16 24 32 40 ... 1000 (in ms)
    <param name="period" value="4"/>

  </node>

  #### Complementary filter

  <node pkg="imu_complementary_filter" type="complementary_filter_node"
      name="complementary_filter_gain_node" output="screen">
    <param name="do_bias_estimation" value="true"/>
    <param name="do_adaptive_gain" value="true"/>
    <param name="use_mag" value="false"/>
    <param name="gain_acc" value="0.01"/>
    <param name="gain_mag" value="0.01"/>
 
  
  </node>
  


</launch>
