
# Collision

The collision properties of a link. Note that this can be different from the visual properties of a link, for example, simpler collision models are often used to reduce computation time. This is a child element of `<link>` and a link can have multiple collisions.



```python
# Import the element creator
from pcg_gazebo.parsers.sdf import create_sdf_element
```


```python
# The collision element is created with an empty geometry by default
collision = create_sdf_element('collision')
print(collision)
```

    <collision name="collision">
      <geometry>
        <empty></empty>
      </geometry>
    </collision>
    



```python
# To see the optional elements, use the method reset()
collision.reset(with_optional_elements=True)
print(collision)
```

    <collision name="collision">
      <max_contacts>10</max_contacts>
      <pose frame="">0 0 0 0 0 0</pose>
      <surface>
        <bounce>
          <threshold>100000</threshold>
          <restitution_coefficient>0</restitution_coefficient>
        </bounce>
        <contact>
          <ode>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
          </ode>
          <bullet>
            <kp>1000000000000.0</kp>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kd>1</kd>
            <split_impulse>1</split_impulse>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
          </bullet>
        </contact>
        <friction>
          <torsional>
            <ode>
              <slip>0</slip>
            </ode>
            <surface_radius>0</surface_radius>
            <coefficient>1</coefficient>
            <use_patch_radius>1</use_patch_radius>
            <patch_radius>0</patch_radius>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <mu>1</mu>
            <slip1>0</slip1>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <rolling_friction>1</rolling_friction>
            <friction>1</friction>
            <friction2>1</friction2>
          </bullet>
        </friction>
      </surface>
      <contact>
        <ode>
          <kp>1000000000000.0</kp>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <min_depth>0</min_depth>
          <soft_cfm>0</soft_cfm>
          <max_vel>0.01</max_vel>
        </ode>
        <bullet>
          <kp>1000000000000.0</kp>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kd>1</kd>
          <split_impulse>1</split_impulse>
          <soft_cfm>0</soft_cfm>
          <soft_erp>0.2</soft_erp>
        </bullet>
      </contact>
      <geometry>
        <empty></empty>
      </geometry>
      <laser_retro>0</laser_retro>
    </collision>
    



```python
# Setting the parameters of a collision element

# Maximum number of contacts allowed between two entities, this value
# will override the max_contacts element defined in physics
collision.max_contacts = 30
# Pose of the collision geometry with respect to a speficied frame
collision.pose = [0, 0, 1, 0, 0, 0]
collision.pose.frame = 'base_link'

print(collision)
```

    <collision name="collision">
      <max_contacts>30</max_contacts>
      <pose frame="base_link">0 0 1 0 0 0</pose>
      <surface>
        <bounce>
          <threshold>100000</threshold>
          <restitution_coefficient>0</restitution_coefficient>
        </bounce>
        <contact>
          <ode>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
          </ode>
          <bullet>
            <kp>1000000000000.0</kp>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kd>1</kd>
            <split_impulse>1</split_impulse>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
          </bullet>
        </contact>
        <friction>
          <torsional>
            <ode>
              <slip>0</slip>
            </ode>
            <surface_radius>0</surface_radius>
            <coefficient>1</coefficient>
            <use_patch_radius>1</use_patch_radius>
            <patch_radius>0</patch_radius>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <mu>1</mu>
            <slip1>0</slip1>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <rolling_friction>1</rolling_friction>
            <friction>1</friction>
            <friction2>1</friction2>
          </bullet>
        </friction>
      </surface>
      <contact>
        <ode>
          <kp>1000000000000.0</kp>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <min_depth>0</min_depth>
          <soft_cfm>0</soft_cfm>
          <max_vel>0.01</max_vel>
        </ode>
        <bullet>
          <kp>1000000000000.0</kp>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kd>1</kd>
          <split_impulse>1</split_impulse>
          <soft_cfm>0</soft_cfm>
          <soft_erp>0.2</soft_erp>
        </bullet>
      </contact>
      <geometry>
        <empty></empty>
      </geometry>
      <laser_retro>0</laser_retro>
    </collision>
    



```python
# Setting different geometries to the collision element
collision.geometry.box = create_sdf_element('box')
print(collision)
collision.geometry.sphere = create_sdf_element('sphere')
print(collision)
collision.geometry.cylinder = create_sdf_element('cylinder')
print(collision)
collision.geometry.plane = create_sdf_element('plane')
print(collision)
collision.geometry.mesh = create_sdf_element('mesh')
collision.geometry.mesh.reset(with_optional_elements=True)
print(collision)
collision.geometry.image = create_sdf_element('image')
print(collision)
collision.geometry.polyline = create_sdf_element('polyline')
print(collision)
```

    <collision name="collision">
      <max_contacts>30</max_contacts>
      <pose frame="base_link">0 0 1 0 0 0</pose>
      <surface>
        <bounce>
          <threshold>100000</threshold>
          <restitution_coefficient>0</restitution_coefficient>
        </bounce>
        <contact>
          <ode>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
          </ode>
          <bullet>
            <kp>1000000000000.0</kp>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kd>1</kd>
            <split_impulse>1</split_impulse>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
          </bullet>
        </contact>
        <friction>
          <torsional>
            <ode>
              <slip>0</slip>
            </ode>
            <surface_radius>0</surface_radius>
            <coefficient>1</coefficient>
            <use_patch_radius>1</use_patch_radius>
            <patch_radius>0</patch_radius>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <mu>1</mu>
            <slip1>0</slip1>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <rolling_friction>1</rolling_friction>
            <friction>1</friction>
            <friction2>1</friction2>
          </bullet>
        </friction>
      </surface>
      <contact>
        <ode>
          <kp>1000000000000.0</kp>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <min_depth>0</min_depth>
          <soft_cfm>0</soft_cfm>
          <max_vel>0.01</max_vel>
        </ode>
        <bullet>
          <kp>1000000000000.0</kp>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kd>1</kd>
          <split_impulse>1</split_impulse>
          <soft_cfm>0</soft_cfm>
          <soft_erp>0.2</soft_erp>
        </bullet>
      </contact>
      <geometry>
        <box>
          <size>0 0 0</size>
        </box>
      </geometry>
      <laser_retro>0</laser_retro>
    </collision>
    
    <collision name="collision">
      <max_contacts>30</max_contacts>
      <pose frame="base_link">0 0 1 0 0 0</pose>
      <surface>
        <bounce>
          <threshold>100000</threshold>
          <restitution_coefficient>0</restitution_coefficient>
        </bounce>
        <contact>
          <ode>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
          </ode>
          <bullet>
            <kp>1000000000000.0</kp>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kd>1</kd>
            <split_impulse>1</split_impulse>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
          </bullet>
        </contact>
        <friction>
          <torsional>
            <ode>
              <slip>0</slip>
            </ode>
            <surface_radius>0</surface_radius>
            <coefficient>1</coefficient>
            <use_patch_radius>1</use_patch_radius>
            <patch_radius>0</patch_radius>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <mu>1</mu>
            <slip1>0</slip1>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <rolling_friction>1</rolling_friction>
            <friction>1</friction>
            <friction2>1</friction2>
          </bullet>
        </friction>
      </surface>
      <contact>
        <ode>
          <kp>1000000000000.0</kp>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <min_depth>0</min_depth>
          <soft_cfm>0</soft_cfm>
          <max_vel>0.01</max_vel>
        </ode>
        <bullet>
          <kp>1000000000000.0</kp>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kd>1</kd>
          <split_impulse>1</split_impulse>
          <soft_cfm>0</soft_cfm>
          <soft_erp>0.2</soft_erp>
        </bullet>
      </contact>
      <geometry>
        <sphere>
          <radius>0</radius>
        </sphere>
      </geometry>
      <laser_retro>0</laser_retro>
    </collision>
    
    <collision name="collision">
      <max_contacts>30</max_contacts>
      <pose frame="base_link">0 0 1 0 0 0</pose>
      <surface>
        <bounce>
          <threshold>100000</threshold>
          <restitution_coefficient>0</restitution_coefficient>
        </bounce>
        <contact>
          <ode>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
          </ode>
          <bullet>
            <kp>1000000000000.0</kp>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kd>1</kd>
            <split_impulse>1</split_impulse>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
          </bullet>
        </contact>
        <friction>
          <torsional>
            <ode>
              <slip>0</slip>
            </ode>
            <surface_radius>0</surface_radius>
            <coefficient>1</coefficient>
            <use_patch_radius>1</use_patch_radius>
            <patch_radius>0</patch_radius>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <mu>1</mu>
            <slip1>0</slip1>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <rolling_friction>1</rolling_friction>
            <friction>1</friction>
            <friction2>1</friction2>
          </bullet>
        </friction>
      </surface>
      <contact>
        <ode>
          <kp>1000000000000.0</kp>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <min_depth>0</min_depth>
          <soft_cfm>0</soft_cfm>
          <max_vel>0.01</max_vel>
        </ode>
        <bullet>
          <kp>1000000000000.0</kp>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kd>1</kd>
          <split_impulse>1</split_impulse>
          <soft_cfm>0</soft_cfm>
          <soft_erp>0.2</soft_erp>
        </bullet>
      </contact>
      <geometry>
        <cylinder>
          <radius>0</radius>
          <length>0</length>
        </cylinder>
      </geometry>
      <laser_retro>0</laser_retro>
    </collision>
    
    <collision name="collision">
      <max_contacts>30</max_contacts>
      <pose frame="base_link">0 0 1 0 0 0</pose>
      <surface>
        <bounce>
          <threshold>100000</threshold>
          <restitution_coefficient>0</restitution_coefficient>
        </bounce>
        <contact>
          <ode>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
          </ode>
          <bullet>
            <kp>1000000000000.0</kp>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kd>1</kd>
            <split_impulse>1</split_impulse>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
          </bullet>
        </contact>
        <friction>
          <torsional>
            <ode>
              <slip>0</slip>
            </ode>
            <surface_radius>0</surface_radius>
            <coefficient>1</coefficient>
            <use_patch_radius>1</use_patch_radius>
            <patch_radius>0</patch_radius>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <mu>1</mu>
            <slip1>0</slip1>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <rolling_friction>1</rolling_friction>
            <friction>1</friction>
            <friction2>1</friction2>
          </bullet>
        </friction>
      </surface>
      <contact>
        <ode>
          <kp>1000000000000.0</kp>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <min_depth>0</min_depth>
          <soft_cfm>0</soft_cfm>
          <max_vel>0.01</max_vel>
        </ode>
        <bullet>
          <kp>1000000000000.0</kp>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kd>1</kd>
          <split_impulse>1</split_impulse>
          <soft_cfm>0</soft_cfm>
          <soft_erp>0.2</soft_erp>
        </bullet>
      </contact>
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>0 0</size>
        </plane>
      </geometry>
      <laser_retro>0</laser_retro>
    </collision>
    
    <collision name="collision">
      <max_contacts>30</max_contacts>
      <pose frame="base_link">0 0 1 0 0 0</pose>
      <surface>
        <bounce>
          <threshold>100000</threshold>
          <restitution_coefficient>0</restitution_coefficient>
        </bounce>
        <contact>
          <ode>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
          </ode>
          <bullet>
            <kp>1000000000000.0</kp>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kd>1</kd>
            <split_impulse>1</split_impulse>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
          </bullet>
        </contact>
        <friction>
          <torsional>
            <ode>
              <slip>0</slip>
            </ode>
            <surface_radius>0</surface_radius>
            <coefficient>1</coefficient>
            <use_patch_radius>1</use_patch_radius>
            <patch_radius>0</patch_radius>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <mu>1</mu>
            <slip1>0</slip1>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <rolling_friction>1</rolling_friction>
            <friction>1</friction>
            <friction2>1</friction2>
          </bullet>
        </friction>
      </surface>
      <contact>
        <ode>
          <kp>1000000000000.0</kp>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <min_depth>0</min_depth>
          <soft_cfm>0</soft_cfm>
          <max_vel>0.01</max_vel>
        </ode>
        <bullet>
          <kp>1000000000000.0</kp>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kd>1</kd>
          <split_impulse>1</split_impulse>
          <soft_cfm>0</soft_cfm>
          <soft_erp>0.2</soft_erp>
        </bullet>
      </contact>
      <geometry>
        <mesh>
          <uri></uri>
          <scale>1 1 1</scale>
          <submesh>
            <name>none</name>
            <center>0</center>
          </submesh>
        </mesh>
      </geometry>
      <laser_retro>0</laser_retro>
    </collision>
    
    <collision name="collision">
      <max_contacts>30</max_contacts>
      <pose frame="base_link">0 0 1 0 0 0</pose>
      <surface>
        <bounce>
          <threshold>100000</threshold>
          <restitution_coefficient>0</restitution_coefficient>
        </bounce>
        <contact>
          <ode>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
          </ode>
          <bullet>
            <kp>1000000000000.0</kp>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kd>1</kd>
            <split_impulse>1</split_impulse>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
          </bullet>
        </contact>
        <friction>
          <torsional>
            <ode>
              <slip>0</slip>
            </ode>
            <surface_radius>0</surface_radius>
            <coefficient>1</coefficient>
            <use_patch_radius>1</use_patch_radius>
            <patch_radius>0</patch_radius>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <mu>1</mu>
            <slip1>0</slip1>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <rolling_friction>1</rolling_friction>
            <friction>1</friction>
            <friction2>1</friction2>
          </bullet>
        </friction>
      </surface>
      <contact>
        <ode>
          <kp>1000000000000.0</kp>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <min_depth>0</min_depth>
          <soft_cfm>0</soft_cfm>
          <max_vel>0.01</max_vel>
        </ode>
        <bullet>
          <kp>1000000000000.0</kp>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kd>1</kd>
          <split_impulse>1</split_impulse>
          <soft_cfm>0</soft_cfm>
          <soft_erp>0.2</soft_erp>
        </bullet>
      </contact>
      <geometry>
        <image>
          <threshold>0</threshold>
          <uri></uri>
          <scale>1</scale>
          <granularity>1</granularity>
          <height>1</height>
        </image>
      </geometry>
      <laser_retro>0</laser_retro>
    </collision>
    
    <collision name="collision">
      <max_contacts>30</max_contacts>
      <pose frame="base_link">0 0 1 0 0 0</pose>
      <surface>
        <bounce>
          <threshold>100000</threshold>
          <restitution_coefficient>0</restitution_coefficient>
        </bounce>
        <contact>
          <ode>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
          </ode>
          <bullet>
            <kp>1000000000000.0</kp>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kd>1</kd>
            <split_impulse>1</split_impulse>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
          </bullet>
        </contact>
        <friction>
          <torsional>
            <ode>
              <slip>0</slip>
            </ode>
            <surface_radius>0</surface_radius>
            <coefficient>1</coefficient>
            <use_patch_radius>1</use_patch_radius>
            <patch_radius>0</patch_radius>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <mu>1</mu>
            <slip1>0</slip1>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <rolling_friction>1</rolling_friction>
            <friction>1</friction>
            <friction2>1</friction2>
          </bullet>
        </friction>
      </surface>
      <contact>
        <ode>
          <kp>1000000000000.0</kp>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <min_depth>0</min_depth>
          <soft_cfm>0</soft_cfm>
          <max_vel>0.01</max_vel>
        </ode>
        <bullet>
          <kp>1000000000000.0</kp>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kd>1</kd>
          <split_impulse>1</split_impulse>
          <soft_cfm>0</soft_cfm>
          <soft_erp>0.2</soft_erp>
        </bullet>
      </contact>
      <geometry>
        <polyline>
          <height>1</height>
        </polyline>
      </geometry>
      <laser_retro>0</laser_retro>
    </collision>
    



```python
# Optional elements can also be created dynamically
collision = create_sdf_element('collision')
print(collision)
```

    <collision name="collision">
      <geometry>
        <empty></empty>
      </geometry>
    </collision>
    



```python
collision.max_contacts = 40
print(collision)
```

    <collision name="collision">
      <max_contacts>40</max_contacts>
      <geometry>
        <empty></empty>
      </geometry>
    </collision>
    



```python
collision.pose = [0, 0.2, 0, 0, 0, 0]
collision.pose.frame = 'new_frame'
print(collision)
```

    <collision name="collision">
      <max_contacts>40</max_contacts>
      <pose frame="new_frame">0 0.2 0 0 0 0</pose>
      <geometry>
        <empty></empty>
      </geometry>
    </collision>
    



```python
# The geometry entity can be set with a dictionary with all the child parameters
collision.geometry.box = dict(size=[2, 3, 4])
print(collision)
```

    <collision name="collision">
      <max_contacts>40</max_contacts>
      <pose frame="new_frame">0 0.2 0 0 0 0</pose>
      <geometry>
        <box>
          <size>2 3 4</size>
        </box>
      </geometry>
    </collision>
    



```python
# The pose, as other variables, can be set using a dictionary
# For SDF elements with no child elements, only values, the dictionary must always have a key 'value'
#      d = {value=[0, 0, 0, 0, 0, 0]}
# If the element contains attributes, as the attribute 'frame' in the element 'pose', there should be a key
# 'attributes' with a dictionary containing all the attributes
#      d = {value=[0, 0, 0, 0, 0, 0], attributes=dict(frame='new_frame')}
collision.pose = {'value': [0, 0.2, 0, 0, 0, 0], 'attributes': {'frame': 'new_frame'}}
print(collision)
```

    <collision name="collision">
      <max_contacts>40</max_contacts>
      <pose frame="new_frame">0 0.2 0 0 0 0</pose>
      <geometry>
        <box>
          <size>2 3 4</size>
        </box>
      </geometry>
    </collision>
    



```python

```
