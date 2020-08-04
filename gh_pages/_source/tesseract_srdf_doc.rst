*********************
Tesseract SRDF Format
*********************

Background
==========
Tesseract has its own SRDF format which is similar to the one used through ROS, but includes features specific to Tesseract.

Example File
------------

.. code-block:: xml

   <robot name="abb_irb2400" version="1.0.0">
       <group name="manipulator_chain">
           <chain base_link="base_link" tip_link="tool0"/>
       </group>

       <group name="manipulator_joints">
           <joint name="joint_1"/>
           <joint name="joint_2"/>
           <joint name="joint_3"/>
           <joint name="joint_4"/>
           <joint name="joint_5"/>
           <joint name="joint_6"/>
       </group>

       <group_state name="zeros" group="manipulator_joints">
           <joint name="joint_6" value="0"/>
           <joint name="joint_4" value="0"/>
           <joint name="joint_5" value="0"/>
           <joint name="joint_3" value="0"/>
           <joint name="joint_1" value="0"/>
           <joint name="joint_2" value="0"/>
       </group_state>

       <group_state name="zeros" group="manipulator_chain">
           <joint name="joint_6" value="0"/>
           <joint name="joint_4" value="0"/>
           <joint name="joint_5" value="0"/>
           <joint name="joint_3" value="0"/>
           <joint name="joint_1" value="0"/>
           <joint name="joint_2" value="0"/>
       </group_state>

       <group_tcps group="manipulator_chain">
           <tcp name="scanner" xyz="  0   0 0.2" wxyz="1 0 0 0"/>
       </group_tcps>

       <group_tcps group="manipulator_joints">
           <tcp name="scanner" xyz="  0   0 0.2" wxyz="1 0 0 0"/>
       </group_tcps>

       <group_opw group="manipulator_chain" a1="0.10000000000000001" a2="-0.13500000000000001" b="0" c1="0.61499999999999999" c2="0.70499999999999996" c3="0.755" c4="0.085000000000000006" offsets="0.000000 0.000000 -1.570796 0.000000 0.000000 0.000000" sign_corrections="1 1 1 1 1 1"/>

       <disable_collisions link1="link_3" link2="link_5" reason="Never"/>
       <disable_collisions link1="link_3" link2="link_6" reason="Never"/>
       <disable_collisions link1="link_2" link2="link_5" reason="Never"/>
       <disable_collisions link1="link_2" link2="link_4" reason="Never"/>
       <disable_collisions link1="link_4" link2="link_6" reason="Allways"/>
       <disable_collisions link1="link_1" link2="link_5" reason="Never"/>
       <disable_collisions link1="link_3" link2="link_4" reason="Adjacent"/>
       <disable_collisions link1="link_2" link2="link_3" reason="Adjacent"/>
       <disable_collisions link1="base_link" link2="link_1" reason="Adjacent"/>
       <disable_collisions link1="link_1" link2="link_2" reason="Adjacent"/>
       <disable_collisions link1="link_1" link2="link_4" reason="Never"/>
       <disable_collisions link1="base_link" link2="link_4" reason="Never"/>
       <disable_collisions link1="link_1" link2="link_6" reason="Never"/>
       <disable_collisions link1="link_5" link2="link_6" reason="Adjacent"/>
       <disable_collisions link1="base_link" link2="link_5" reason="Never"/>
       <disable_collisions link1="link_1" link2="link_3" reason="Never"/>
       <disable_collisions link1="base_link" link2="link_2" reason="Never"/>
       <disable_collisions link1="link_2" link2="link_6" reason="Never"/>
       <disable_collisions link1="link_4" link2="link_5" reason="Adjacent"/>
       <disable_collisions link1="base_link" link2="link_6" reason="Never"/>
       <disable_collisions link1="base_link" link2="link_3" reason="Never"/>
   </robot>
