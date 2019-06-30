
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
      <laser_retro>0</laser_retro>
      <surface>
        <contact>
          <bullet>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
            <soft_cfm>0</soft_cfm>
            <split_impulse>1</split_impulse>
          </bullet>
          <ode>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
          </ode>
        </contact>
        <bounce>
          <restitution_coefficient>0</restitution_coefficient>
          <threshold>100000</threshold>
        </bounce>
        <friction>
          <torsional>
            <coefficient>1</coefficient>
            <patch_radius>0</patch_radius>
            <use_patch_radius>1</use_patch_radius>
            <surface_radius>0</surface_radius>
            <ode>
              <slip>0</slip>
            </ode>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <slip1>0</slip1>
            <mu>1</mu>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <friction2>1</friction2>
            <friction>1</friction>
            <rolling_friction>1</rolling_friction>
          </bullet>
        </friction>
      </surface>
      <max_contacts>10</max_contacts>
      <geometry>
        <empty></empty>
      </geometry>
      <contact>
        <bullet>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kp>1000000000000.0</kp>
          <soft_cfm>0</soft_cfm>
          <split_impulse>1</split_impulse>
        </bullet>
        <ode>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <soft_cfm>0</soft_cfm>
          <kp>1000000000000.0</kp>
          <min_depth>0</min_depth>
          <max_vel>0.01</max_vel>
        </ode>
      </contact>
      <pose frame="">0 0 0 0 0 0</pose>
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
      <laser_retro>0</laser_retro>
      <surface>
        <contact>
          <bullet>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
            <soft_cfm>0</soft_cfm>
            <split_impulse>1</split_impulse>
          </bullet>
          <ode>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
          </ode>
        </contact>
        <bounce>
          <restitution_coefficient>0</restitution_coefficient>
          <threshold>100000</threshold>
        </bounce>
        <friction>
          <torsional>
            <coefficient>1</coefficient>
            <patch_radius>0</patch_radius>
            <use_patch_radius>1</use_patch_radius>
            <surface_radius>0</surface_radius>
            <ode>
              <slip>0</slip>
            </ode>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <slip1>0</slip1>
            <mu>1</mu>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <friction2>1</friction2>
            <friction>1</friction>
            <rolling_friction>1</rolling_friction>
          </bullet>
        </friction>
      </surface>
      <max_contacts>30</max_contacts>
      <geometry>
        <empty></empty>
      </geometry>
      <contact>
        <bullet>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kp>1000000000000.0</kp>
          <soft_cfm>0</soft_cfm>
          <split_impulse>1</split_impulse>
        </bullet>
        <ode>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <soft_cfm>0</soft_cfm>
          <kp>1000000000000.0</kp>
          <min_depth>0</min_depth>
          <max_vel>0.01</max_vel>
        </ode>
      </contact>
      <pose frame="base_link">0 0 1 0 0 0</pose>
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
      <laser_retro>0</laser_retro>
      <surface>
        <contact>
          <bullet>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
            <soft_cfm>0</soft_cfm>
            <split_impulse>1</split_impulse>
          </bullet>
          <ode>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
          </ode>
        </contact>
        <bounce>
          <restitution_coefficient>0</restitution_coefficient>
          <threshold>100000</threshold>
        </bounce>
        <friction>
          <torsional>
            <coefficient>1</coefficient>
            <patch_radius>0</patch_radius>
            <use_patch_radius>1</use_patch_radius>
            <surface_radius>0</surface_radius>
            <ode>
              <slip>0</slip>
            </ode>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <slip1>0</slip1>
            <mu>1</mu>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <friction2>1</friction2>
            <friction>1</friction>
            <rolling_friction>1</rolling_friction>
          </bullet>
        </friction>
      </surface>
      <max_contacts>30</max_contacts>
      <geometry>
        <box>
          <size>0 0 0</size>
        </box>
      </geometry>
      <contact>
        <bullet>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kp>1000000000000.0</kp>
          <soft_cfm>0</soft_cfm>
          <split_impulse>1</split_impulse>
        </bullet>
        <ode>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <soft_cfm>0</soft_cfm>
          <kp>1000000000000.0</kp>
          <min_depth>0</min_depth>
          <max_vel>0.01</max_vel>
        </ode>
      </contact>
      <pose frame="base_link">0 0 1 0 0 0</pose>
    </collision>
    
    <collision name="collision">
      <laser_retro>0</laser_retro>
      <surface>
        <contact>
          <bullet>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
            <soft_cfm>0</soft_cfm>
            <split_impulse>1</split_impulse>
          </bullet>
          <ode>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
          </ode>
        </contact>
        <bounce>
          <restitution_coefficient>0</restitution_coefficient>
          <threshold>100000</threshold>
        </bounce>
        <friction>
          <torsional>
            <coefficient>1</coefficient>
            <patch_radius>0</patch_radius>
            <use_patch_radius>1</use_patch_radius>
            <surface_radius>0</surface_radius>
            <ode>
              <slip>0</slip>
            </ode>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <slip1>0</slip1>
            <mu>1</mu>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <friction2>1</friction2>
            <friction>1</friction>
            <rolling_friction>1</rolling_friction>
          </bullet>
        </friction>
      </surface>
      <max_contacts>30</max_contacts>
      <geometry>
        <sphere>
          <radius>0</radius>
        </sphere>
      </geometry>
      <contact>
        <bullet>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kp>1000000000000.0</kp>
          <soft_cfm>0</soft_cfm>
          <split_impulse>1</split_impulse>
        </bullet>
        <ode>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <soft_cfm>0</soft_cfm>
          <kp>1000000000000.0</kp>
          <min_depth>0</min_depth>
          <max_vel>0.01</max_vel>
        </ode>
      </contact>
      <pose frame="base_link">0 0 1 0 0 0</pose>
    </collision>
    
    <collision name="collision">
      <laser_retro>0</laser_retro>
      <surface>
        <contact>
          <bullet>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
            <soft_cfm>0</soft_cfm>
            <split_impulse>1</split_impulse>
          </bullet>
          <ode>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
          </ode>
        </contact>
        <bounce>
          <restitution_coefficient>0</restitution_coefficient>
          <threshold>100000</threshold>
        </bounce>
        <friction>
          <torsional>
            <coefficient>1</coefficient>
            <patch_radius>0</patch_radius>
            <use_patch_radius>1</use_patch_radius>
            <surface_radius>0</surface_radius>
            <ode>
              <slip>0</slip>
            </ode>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <slip1>0</slip1>
            <mu>1</mu>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <friction2>1</friction2>
            <friction>1</friction>
            <rolling_friction>1</rolling_friction>
          </bullet>
        </friction>
      </surface>
      <max_contacts>30</max_contacts>
      <geometry>
        <cylinder>
          <length>0</length>
          <radius>0</radius>
        </cylinder>
      </geometry>
      <contact>
        <bullet>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kp>1000000000000.0</kp>
          <soft_cfm>0</soft_cfm>
          <split_impulse>1</split_impulse>
        </bullet>
        <ode>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <soft_cfm>0</soft_cfm>
          <kp>1000000000000.0</kp>
          <min_depth>0</min_depth>
          <max_vel>0.01</max_vel>
        </ode>
      </contact>
      <pose frame="base_link">0 0 1 0 0 0</pose>
    </collision>
    
    <collision name="collision">
      <laser_retro>0</laser_retro>
      <surface>
        <contact>
          <bullet>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
            <soft_cfm>0</soft_cfm>
            <split_impulse>1</split_impulse>
          </bullet>
          <ode>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
          </ode>
        </contact>
        <bounce>
          <restitution_coefficient>0</restitution_coefficient>
          <threshold>100000</threshold>
        </bounce>
        <friction>
          <torsional>
            <coefficient>1</coefficient>
            <patch_radius>0</patch_radius>
            <use_patch_radius>1</use_patch_radius>
            <surface_radius>0</surface_radius>
            <ode>
              <slip>0</slip>
            </ode>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <slip1>0</slip1>
            <mu>1</mu>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <friction2>1</friction2>
            <friction>1</friction>
            <rolling_friction>1</rolling_friction>
          </bullet>
        </friction>
      </surface>
      <max_contacts>30</max_contacts>
      <geometry>
        <plane>
          <size>0 0</size>
          <normal>0 0 1</normal>
        </plane>
      </geometry>
      <contact>
        <bullet>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kp>1000000000000.0</kp>
          <soft_cfm>0</soft_cfm>
          <split_impulse>1</split_impulse>
        </bullet>
        <ode>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <soft_cfm>0</soft_cfm>
          <kp>1000000000000.0</kp>
          <min_depth>0</min_depth>
          <max_vel>0.01</max_vel>
        </ode>
      </contact>
      <pose frame="base_link">0 0 1 0 0 0</pose>
    </collision>
    
    <collision name="collision">
      <laser_retro>0</laser_retro>
      <surface>
        <contact>
          <bullet>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
            <soft_cfm>0</soft_cfm>
            <split_impulse>1</split_impulse>
          </bullet>
          <ode>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
          </ode>
        </contact>
        <bounce>
          <restitution_coefficient>0</restitution_coefficient>
          <threshold>100000</threshold>
        </bounce>
        <friction>
          <torsional>
            <coefficient>1</coefficient>
            <patch_radius>0</patch_radius>
            <use_patch_radius>1</use_patch_radius>
            <surface_radius>0</surface_radius>
            <ode>
              <slip>0</slip>
            </ode>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <slip1>0</slip1>
            <mu>1</mu>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <friction2>1</friction2>
            <friction>1</friction>
            <rolling_friction>1</rolling_friction>
          </bullet>
        </friction>
      </surface>
      <max_contacts>30</max_contacts>
      <geometry>
        <mesh>
          <scale>1 1 1</scale>
          <uri></uri>
          <submesh>
            <center>0</center>
            <name>none</name>
          </submesh>
        </mesh>
      </geometry>
      <contact>
        <bullet>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kp>1000000000000.0</kp>
          <soft_cfm>0</soft_cfm>
          <split_impulse>1</split_impulse>
        </bullet>
        <ode>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <soft_cfm>0</soft_cfm>
          <kp>1000000000000.0</kp>
          <min_depth>0</min_depth>
          <max_vel>0.01</max_vel>
        </ode>
      </contact>
      <pose frame="base_link">0 0 1 0 0 0</pose>
    </collision>
    
    <collision name="collision">
      <laser_retro>0</laser_retro>
      <surface>
        <contact>
          <bullet>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
            <soft_cfm>0</soft_cfm>
            <split_impulse>1</split_impulse>
          </bullet>
          <ode>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
          </ode>
        </contact>
        <bounce>
          <restitution_coefficient>0</restitution_coefficient>
          <threshold>100000</threshold>
        </bounce>
        <friction>
          <torsional>
            <coefficient>1</coefficient>
            <patch_radius>0</patch_radius>
            <use_patch_radius>1</use_patch_radius>
            <surface_radius>0</surface_radius>
            <ode>
              <slip>0</slip>
            </ode>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <slip1>0</slip1>
            <mu>1</mu>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <friction2>1</friction2>
            <friction>1</friction>
            <rolling_friction>1</rolling_friction>
          </bullet>
        </friction>
      </surface>
      <max_contacts>30</max_contacts>
      <geometry>
        <image>
          <granularity>1</granularity>
          <threshold>0</threshold>
          <height>1</height>
          <scale>1</scale>
          <uri></uri>
        </image>
      </geometry>
      <contact>
        <bullet>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kp>1000000000000.0</kp>
          <soft_cfm>0</soft_cfm>
          <split_impulse>1</split_impulse>
        </bullet>
        <ode>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <soft_cfm>0</soft_cfm>
          <kp>1000000000000.0</kp>
          <min_depth>0</min_depth>
          <max_vel>0.01</max_vel>
        </ode>
      </contact>
      <pose frame="base_link">0 0 1 0 0 0</pose>
    </collision>
    
    <collision name="collision">
      <laser_retro>0</laser_retro>
      <surface>
        <contact>
          <bullet>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
            <soft_cfm>0</soft_cfm>
            <split_impulse>1</split_impulse>
          </bullet>
          <ode>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
          </ode>
        </contact>
        <bounce>
          <restitution_coefficient>0</restitution_coefficient>
          <threshold>100000</threshold>
        </bounce>
        <friction>
          <torsional>
            <coefficient>1</coefficient>
            <patch_radius>0</patch_radius>
            <use_patch_radius>1</use_patch_radius>
            <surface_radius>0</surface_radius>
            <ode>
              <slip>0</slip>
            </ode>
          </torsional>
          <ode>
            <fdir1>0 0 0</fdir1>
            <slip1>0</slip1>
            <mu>1</mu>
            <slip2>0</slip2>
            <mu2>1</mu2>
          </ode>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <friction2>1</friction2>
            <friction>1</friction>
            <rolling_friction>1</rolling_friction>
          </bullet>
        </friction>
      </surface>
      <max_contacts>30</max_contacts>
      <geometry>
        <polyline>
          <height>1</height>
        </polyline>
      </geometry>
      <contact>
        <bullet>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kp>1000000000000.0</kp>
          <soft_cfm>0</soft_cfm>
          <split_impulse>1</split_impulse>
        </bullet>
        <ode>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <soft_cfm>0</soft_cfm>
          <kp>1000000000000.0</kp>
          <min_depth>0</min_depth>
          <max_vel>0.01</max_vel>
        </ode>
      </contact>
      <pose frame="base_link">0 0 1 0 0 0</pose>
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
