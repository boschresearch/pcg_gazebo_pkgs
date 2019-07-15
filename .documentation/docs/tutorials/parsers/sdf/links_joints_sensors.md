
# Links, joints and sensors

## Links

A physical link in the simulation contains inertia, collision and visual properties. A link must be a child of a model and a model can have multiple links.


```python
# Import the element creator
from pcg_gazebo.parsers.sdf import create_sdf_element
```


```python
# The link is empty by default
link = create_sdf_element('link')
print(link)
```

    <link name="link"/>
    



```python
# Let's create the elements dynamically at first
link = create_sdf_element('link')

# The link's name must be unique in a model
link.name = 'base_link'
print(link)
```

    <link name="base_link"/>
    



```python
# Mass of the link in kg
link.mass = 30
# The center of mass are the cartesian coordinates in link.inertial.pose
link.center_of_mass = [0, 10, 0]
# The moments of inertia describe the elements of the 3x3 rotational inertial matrix
link.inertia.ixx = 0.5
link.inertia.iyy = 0.5
link.inertia.izz = 0.5
print(link)
```

    <link name="base_link">
      <inertial>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <mass>30.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <iyy>0.5</iyy>
          <ixz>0</ixz>
          <iyz>0</iyz>
          <izz>0.5</izz>
          <ixy>0</ixy>
        </inertia>
      </inertial>
    </link>
    



```python
# If gravity is set as true, the link will be affected by gravity
link.gravity = True
print(link)
```

    <link name="base_link">
      <inertial>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <mass>30.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <iyy>0.5</iyy>
          <ixz>0</ixz>
          <iyz>0</iyz>
          <izz>0.5</izz>
          <ixy>0</ixy>
        </inertia>
      </inertial>
      <gravity>1</gravity>
    </link>
    



```python
# If kinematic is set to true, the link is kinematic only
link.kinematic = False
print(link)
```

    <link name="base_link">
      <inertial>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <mass>30.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <iyy>0.5</iyy>
          <ixz>0</ixz>
          <iyz>0</iyz>
          <izz>0.5</izz>
          <ixy>0</ixy>
        </inertia>
      </inertial>
      <kinematic>0</kinematic>
      <gravity>1</gravity>
    </link>
    



```python
# The pose of the link with respect to a frame
link.pose = [0, 0, 1, 0, 0, 0]
print(link)
```

    <link name="base_link">
      <inertial>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <mass>30.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <iyy>0.5</iyy>
          <ixz>0</ixz>
          <iyz>0</iyz>
          <izz>0.5</izz>
          <ixy>0</ixy>
        </inertia>
      </inertial>
      <kinematic>0</kinematic>
      <pose frame="">0 0 1 0 0 0</pose>
      <gravity>1</gravity>
    </link>
    



```python
# As mentioned in previous notebooks, a link can have multiple visual and collision elements
# To create an empty collision geometry, use the function add_collision as follows
link.add_collision(name='collision_1')
print(link.collisions[0])
```

    <collision name="collision_1">
      <geometry>
        <empty></empty>
      </geometry>
    </collision>
    



```python
# Set the geometry of the collision
link.collisions[0].box = create_sdf_element('box')
print(link)
```

    <link name="base_link">
      <inertial>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <mass>30.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <iyy>0.5</iyy>
          <ixz>0</ixz>
          <iyz>0</iyz>
          <izz>0.5</izz>
          <ixy>0</ixy>
        </inertia>
      </inertial>
      <kinematic>0</kinematic>
      <pose frame="">0 0 1 0 0 0</pose>
      <gravity>1</gravity>
      <collision name="collision_1">
        <geometry>
          <empty></empty>
        </geometry>
      </collision>
    </link>
    



```python
# You can also add a collision geometry by creating a collision entity and 
# adding it to the link as follows
collision = create_sdf_element('collision')
collision.reset(with_optional_elements=True)
collision.geometry.cylinder = create_sdf_element('cylinder')

link.add_collision('collision_2', collision)
print(link)
```

    <link name="base_link">
      <inertial>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <mass>30.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <iyy>0.5</iyy>
          <ixz>0</ixz>
          <iyz>0</iyz>
          <izz>0.5</izz>
          <ixy>0</ixy>
        </inertia>
      </inertial>
      <kinematic>0</kinematic>
      <pose frame="">0 0 1 0 0 0</pose>
      <gravity>1</gravity>
      <collision name="collision_1">
        <geometry>
          <empty></empty>
        </geometry>
      </collision>
      <collision name="collision_2">
        <geometry>
          <cylinder>
            <radius>0</radius>
            <length>0</length>
          </cylinder>
        </geometry>
        <max_contacts>10</max_contacts>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <surface>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <min_depth>0</min_depth>
              <max_vel>0.01</max_vel>
              <kd>1</kd>
              <soft_erp>0.2</soft_erp>
            </ode>
          </contact>
          <friction>
            <torsional>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <ode>
                <slip>0</slip>
              </ode>
              <coefficient>1</coefficient>
              <use_patch_radius>1</use_patch_radius>
            </torsional>
            <bullet>
              <fdir1>0 0 0</fdir1>
              <friction>1</friction>
              <rolling_friction>1</rolling_friction>
              <friction2>1</friction2>
            </bullet>
            <ode>
              <mu2>1</mu2>
              <mu>1</mu>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
    



```python
# You can't add collision or visual elements with duplicated names
# You can also add a collision geometry by creating a collision entity and 
# adding it to the link as follows
collision = create_sdf_element('collision')
collision.reset(with_optional_elements=True)
collision.geometry.box = create_sdf_element('box')

link.add_collision('collision_2', collision)
print(link)
```

    <link name="base_link">
      <inertial>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <mass>30.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <iyy>0.5</iyy>
          <ixz>0</ixz>
          <iyz>0</iyz>
          <izz>0.5</izz>
          <ixy>0</ixy>
        </inertia>
      </inertial>
      <kinematic>0</kinematic>
      <pose frame="">0 0 1 0 0 0</pose>
      <gravity>1</gravity>
      <collision name="collision_1">
        <geometry>
          <empty></empty>
        </geometry>
      </collision>
      <collision name="collision_2">
        <geometry>
          <cylinder>
            <radius>0</radius>
            <length>0</length>
          </cylinder>
        </geometry>
        <max_contacts>10</max_contacts>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <surface>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <min_depth>0</min_depth>
              <max_vel>0.01</max_vel>
              <kd>1</kd>
              <soft_erp>0.2</soft_erp>
            </ode>
          </contact>
          <friction>
            <torsional>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <ode>
                <slip>0</slip>
              </ode>
              <coefficient>1</coefficient>
              <use_patch_radius>1</use_patch_radius>
            </torsional>
            <bullet>
              <fdir1>0 0 0</fdir1>
              <friction>1</friction>
              <rolling_friction>1</rolling_friction>
              <friction2>1</friction2>
            </bullet>
            <ode>
              <mu2>1</mu2>
              <mu>1</mu>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>
      <collision name="collision_2_0">
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <max_contacts>10</max_contacts>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <surface>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <min_depth>0</min_depth>
              <max_vel>0.01</max_vel>
              <kd>1</kd>
              <soft_erp>0.2</soft_erp>
            </ode>
          </contact>
          <friction>
            <torsional>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <ode>
                <slip>0</slip>
              </ode>
              <coefficient>1</coefficient>
              <use_patch_radius>1</use_patch_radius>
            </torsional>
            <bullet>
              <fdir1>0 0 0</fdir1>
              <friction>1</friction>
              <rolling_friction>1</rolling_friction>
              <friction2>1</friction2>
            </bullet>
            <ode>
              <mu2>1</mu2>
              <mu>1</mu>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
    



```python
link.add_collision('collision_3', collision)
print(link)
```

    <link name="base_link">
      <inertial>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <mass>30.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <iyy>0.5</iyy>
          <ixz>0</ixz>
          <iyz>0</iyz>
          <izz>0.5</izz>
          <ixy>0</ixy>
        </inertia>
      </inertial>
      <kinematic>0</kinematic>
      <pose frame="">0 0 1 0 0 0</pose>
      <gravity>1</gravity>
      <collision name="collision_1">
        <geometry>
          <empty></empty>
        </geometry>
      </collision>
      <collision name="collision_2">
        <geometry>
          <cylinder>
            <radius>0</radius>
            <length>0</length>
          </cylinder>
        </geometry>
        <max_contacts>10</max_contacts>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <surface>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <min_depth>0</min_depth>
              <max_vel>0.01</max_vel>
              <kd>1</kd>
              <soft_erp>0.2</soft_erp>
            </ode>
          </contact>
          <friction>
            <torsional>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <ode>
                <slip>0</slip>
              </ode>
              <coefficient>1</coefficient>
              <use_patch_radius>1</use_patch_radius>
            </torsional>
            <bullet>
              <fdir1>0 0 0</fdir1>
              <friction>1</friction>
              <rolling_friction>1</rolling_friction>
              <friction2>1</friction2>
            </bullet>
            <ode>
              <mu2>1</mu2>
              <mu>1</mu>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>
      <collision name="collision_2_0">
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <max_contacts>10</max_contacts>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <surface>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <min_depth>0</min_depth>
              <max_vel>0.01</max_vel>
              <kd>1</kd>
              <soft_erp>0.2</soft_erp>
            </ode>
          </contact>
          <friction>
            <torsional>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <ode>
                <slip>0</slip>
              </ode>
              <coefficient>1</coefficient>
              <use_patch_radius>1</use_patch_radius>
            </torsional>
            <bullet>
              <fdir1>0 0 0</fdir1>
              <friction>1</friction>
              <rolling_friction>1</rolling_friction>
              <friction2>1</friction2>
            </bullet>
            <ode>
              <mu2>1</mu2>
              <mu>1</mu>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>
      <collision name="collision_3">
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <max_contacts>10</max_contacts>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <surface>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <min_depth>0</min_depth>
              <max_vel>0.01</max_vel>
              <kd>1</kd>
              <soft_erp>0.2</soft_erp>
            </ode>
          </contact>
          <friction>
            <torsional>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <ode>
                <slip>0</slip>
              </ode>
              <coefficient>1</coefficient>
              <use_patch_radius>1</use_patch_radius>
            </torsional>
            <bullet>
              <fdir1>0 0 0</fdir1>
              <friction>1</friction>
              <rolling_friction>1</rolling_friction>
              <friction2>1</friction2>
            </bullet>
            <ode>
              <mu2>1</mu2>
              <mu>1</mu>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
    



```python
# You can retrieve the collision geometry by its name
# If the name given is not found, the function will return None
print(link.get_collision_by_name('collision_1'))
print(link.get_collision_by_name('collision_10'))
# Or iterate in the collisions list
for elem in link.collisions:
    print(elem)
```

    <collision name="collision_1">
      <geometry>
        <empty></empty>
      </geometry>
    </collision>
    
    None
    <collision name="collision_1">
      <geometry>
        <empty></empty>
      </geometry>
    </collision>
    
    <collision name="collision_2">
      <geometry>
        <cylinder>
          <radius>0</radius>
          <length>0</length>
        </cylinder>
      </geometry>
      <max_contacts>10</max_contacts>
      <contact>
        <bullet>
          <soft_cfm>0</soft_cfm>
          <kp>1000000000000.0</kp>
          <soft_erp>0.2</soft_erp>
          <split_impulse>1</split_impulse>
          <kd>1</kd>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
        </bullet>
        <ode>
          <soft_cfm>0</soft_cfm>
          <kp>1000000000000.0</kp>
          <min_depth>0</min_depth>
          <max_vel>0.01</max_vel>
          <kd>1</kd>
          <soft_erp>0.2</soft_erp>
        </ode>
      </contact>
      <laser_retro>0</laser_retro>
      <pose frame="">0 0 0 0 0 0</pose>
      <surface>
        <bounce>
          <restitution_coefficient>0</restitution_coefficient>
          <threshold>100000</threshold>
        </bounce>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <friction>
          <torsional>
            <patch_radius>0</patch_radius>
            <surface_radius>0</surface_radius>
            <ode>
              <slip>0</slip>
            </ode>
            <coefficient>1</coefficient>
            <use_patch_radius>1</use_patch_radius>
          </torsional>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <friction>1</friction>
            <rolling_friction>1</rolling_friction>
            <friction2>1</friction2>
          </bullet>
          <ode>
            <mu2>1</mu2>
            <mu>1</mu>
            <fdir1>0 0 0</fdir1>
            <slip1>0</slip1>
            <slip2>0</slip2>
          </ode>
        </friction>
      </surface>
    </collision>
    
    <collision name="collision_2_0">
      <geometry>
        <box>
          <size>0 0 0</size>
        </box>
      </geometry>
      <max_contacts>10</max_contacts>
      <contact>
        <bullet>
          <soft_cfm>0</soft_cfm>
          <kp>1000000000000.0</kp>
          <soft_erp>0.2</soft_erp>
          <split_impulse>1</split_impulse>
          <kd>1</kd>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
        </bullet>
        <ode>
          <soft_cfm>0</soft_cfm>
          <kp>1000000000000.0</kp>
          <min_depth>0</min_depth>
          <max_vel>0.01</max_vel>
          <kd>1</kd>
          <soft_erp>0.2</soft_erp>
        </ode>
      </contact>
      <laser_retro>0</laser_retro>
      <pose frame="">0 0 0 0 0 0</pose>
      <surface>
        <bounce>
          <restitution_coefficient>0</restitution_coefficient>
          <threshold>100000</threshold>
        </bounce>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <friction>
          <torsional>
            <patch_radius>0</patch_radius>
            <surface_radius>0</surface_radius>
            <ode>
              <slip>0</slip>
            </ode>
            <coefficient>1</coefficient>
            <use_patch_radius>1</use_patch_radius>
          </torsional>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <friction>1</friction>
            <rolling_friction>1</rolling_friction>
            <friction2>1</friction2>
          </bullet>
          <ode>
            <mu2>1</mu2>
            <mu>1</mu>
            <fdir1>0 0 0</fdir1>
            <slip1>0</slip1>
            <slip2>0</slip2>
          </ode>
        </friction>
      </surface>
    </collision>
    
    <collision name="collision_3">
      <geometry>
        <box>
          <size>0 0 0</size>
        </box>
      </geometry>
      <max_contacts>10</max_contacts>
      <contact>
        <bullet>
          <soft_cfm>0</soft_cfm>
          <kp>1000000000000.0</kp>
          <soft_erp>0.2</soft_erp>
          <split_impulse>1</split_impulse>
          <kd>1</kd>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
        </bullet>
        <ode>
          <soft_cfm>0</soft_cfm>
          <kp>1000000000000.0</kp>
          <min_depth>0</min_depth>
          <max_vel>0.01</max_vel>
          <kd>1</kd>
          <soft_erp>0.2</soft_erp>
        </ode>
      </contact>
      <laser_retro>0</laser_retro>
      <pose frame="">0 0 0 0 0 0</pose>
      <surface>
        <bounce>
          <restitution_coefficient>0</restitution_coefficient>
          <threshold>100000</threshold>
        </bounce>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <friction>
          <torsional>
            <patch_radius>0</patch_radius>
            <surface_radius>0</surface_radius>
            <ode>
              <slip>0</slip>
            </ode>
            <coefficient>1</coefficient>
            <use_patch_radius>1</use_patch_radius>
          </torsional>
          <bullet>
            <fdir1>0 0 0</fdir1>
            <friction>1</friction>
            <rolling_friction>1</rolling_friction>
            <friction2>1</friction2>
          </bullet>
          <ode>
            <mu2>1</mu2>
            <mu>1</mu>
            <fdir1>0 0 0</fdir1>
            <slip1>0</slip1>
            <slip2>0</slip2>
          </ode>
        </friction>
      </surface>
    </collision>
    



```python
# The same is true for visual elements, create an empty visual element by using add_visual
link.add_visual('visual_1')
print(link)
```

    <link name="base_link">
      <inertial>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <mass>30.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <iyy>0.5</iyy>
          <ixz>0</ixz>
          <iyz>0</iyz>
          <izz>0.5</izz>
          <ixy>0</ixy>
        </inertia>
      </inertial>
      <visual name="visual_1">
        <geometry>
          <empty></empty>
        </geometry>
      </visual>
      <kinematic>0</kinematic>
      <pose frame="">0 0 1 0 0 0</pose>
      <gravity>1</gravity>
      <collision name="collision_1">
        <geometry>
          <empty></empty>
        </geometry>
      </collision>
      <collision name="collision_2">
        <geometry>
          <cylinder>
            <radius>0</radius>
            <length>0</length>
          </cylinder>
        </geometry>
        <max_contacts>10</max_contacts>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <surface>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <min_depth>0</min_depth>
              <max_vel>0.01</max_vel>
              <kd>1</kd>
              <soft_erp>0.2</soft_erp>
            </ode>
          </contact>
          <friction>
            <torsional>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <ode>
                <slip>0</slip>
              </ode>
              <coefficient>1</coefficient>
              <use_patch_radius>1</use_patch_radius>
            </torsional>
            <bullet>
              <fdir1>0 0 0</fdir1>
              <friction>1</friction>
              <rolling_friction>1</rolling_friction>
              <friction2>1</friction2>
            </bullet>
            <ode>
              <mu2>1</mu2>
              <mu>1</mu>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>
      <collision name="collision_2_0">
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <max_contacts>10</max_contacts>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <surface>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <min_depth>0</min_depth>
              <max_vel>0.01</max_vel>
              <kd>1</kd>
              <soft_erp>0.2</soft_erp>
            </ode>
          </contact>
          <friction>
            <torsional>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <ode>
                <slip>0</slip>
              </ode>
              <coefficient>1</coefficient>
              <use_patch_radius>1</use_patch_radius>
            </torsional>
            <bullet>
              <fdir1>0 0 0</fdir1>
              <friction>1</friction>
              <rolling_friction>1</rolling_friction>
              <friction2>1</friction2>
            </bullet>
            <ode>
              <mu2>1</mu2>
              <mu>1</mu>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>
      <collision name="collision_3">
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <max_contacts>10</max_contacts>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <surface>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <min_depth>0</min_depth>
              <max_vel>0.01</max_vel>
              <kd>1</kd>
              <soft_erp>0.2</soft_erp>
            </ode>
          </contact>
          <friction>
            <torsional>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <ode>
                <slip>0</slip>
              </ode>
              <coefficient>1</coefficient>
              <use_patch_radius>1</use_patch_radius>
            </torsional>
            <bullet>
              <fdir1>0 0 0</fdir1>
              <friction>1</friction>
              <rolling_friction>1</rolling_friction>
              <friction2>1</friction2>
            </bullet>
            <ode>
              <mu2>1</mu2>
              <mu>1</mu>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
    



```python
# Set the geometry of the visual element
link.visuals[0].geometry.plane = create_sdf_element('plane')
print(link)
```

    <link name="base_link">
      <inertial>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <mass>30.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <iyy>0.5</iyy>
          <ixz>0</ixz>
          <iyz>0</iyz>
          <izz>0.5</izz>
          <ixy>0</ixy>
        </inertia>
      </inertial>
      <visual name="visual_1">
        <geometry>
          <plane>
            <size>0 0</size>
            <normal>0 0 1</normal>
          </plane>
        </geometry>
      </visual>
      <kinematic>0</kinematic>
      <pose frame="">0 0 1 0 0 0</pose>
      <gravity>1</gravity>
      <collision name="collision_1">
        <geometry>
          <empty></empty>
        </geometry>
      </collision>
      <collision name="collision_2">
        <geometry>
          <cylinder>
            <radius>0</radius>
            <length>0</length>
          </cylinder>
        </geometry>
        <max_contacts>10</max_contacts>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <surface>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <min_depth>0</min_depth>
              <max_vel>0.01</max_vel>
              <kd>1</kd>
              <soft_erp>0.2</soft_erp>
            </ode>
          </contact>
          <friction>
            <torsional>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <ode>
                <slip>0</slip>
              </ode>
              <coefficient>1</coefficient>
              <use_patch_radius>1</use_patch_radius>
            </torsional>
            <bullet>
              <fdir1>0 0 0</fdir1>
              <friction>1</friction>
              <rolling_friction>1</rolling_friction>
              <friction2>1</friction2>
            </bullet>
            <ode>
              <mu2>1</mu2>
              <mu>1</mu>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>
      <collision name="collision_2_0">
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <max_contacts>10</max_contacts>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <surface>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <min_depth>0</min_depth>
              <max_vel>0.01</max_vel>
              <kd>1</kd>
              <soft_erp>0.2</soft_erp>
            </ode>
          </contact>
          <friction>
            <torsional>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <ode>
                <slip>0</slip>
              </ode>
              <coefficient>1</coefficient>
              <use_patch_radius>1</use_patch_radius>
            </torsional>
            <bullet>
              <fdir1>0 0 0</fdir1>
              <friction>1</friction>
              <rolling_friction>1</rolling_friction>
              <friction2>1</friction2>
            </bullet>
            <ode>
              <mu2>1</mu2>
              <mu>1</mu>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>
      <collision name="collision_3">
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <max_contacts>10</max_contacts>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <surface>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <min_depth>0</min_depth>
              <max_vel>0.01</max_vel>
              <kd>1</kd>
              <soft_erp>0.2</soft_erp>
            </ode>
          </contact>
          <friction>
            <torsional>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <ode>
                <slip>0</slip>
              </ode>
              <coefficient>1</coefficient>
              <use_patch_radius>1</use_patch_radius>
            </torsional>
            <bullet>
              <fdir1>0 0 0</fdir1>
              <friction>1</friction>
              <rolling_friction>1</rolling_friction>
              <friction2>1</friction2>
            </bullet>
            <ode>
              <mu2>1</mu2>
              <mu>1</mu>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
    



```python
# You can also add a collision geometry by creating a collision entity and 
# adding it to the link as follows
visual = create_sdf_element('visual')
visual.reset(with_optional_elements=True)
visual.geometry.cylinder = create_sdf_element('cylinder')

link.add_visual('visual_2', visual)
print(link)
```

    <link name="base_link">
      <inertial>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <mass>30.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <iyy>0.5</iyy>
          <ixz>0</ixz>
          <iyz>0</iyz>
          <izz>0.5</izz>
          <ixy>0</ixy>
        </inertia>
      </inertial>
      <visual name="visual_1">
        <geometry>
          <plane>
            <size>0 0</size>
            <normal>0 0 1</normal>
          </plane>
        </geometry>
      </visual>
      <visual name="visual_2">
        <pose frame="">0 0 0 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0</radius>
            <length>0</length>
          </cylinder>
        </geometry>
        <cast_shadows>1</cast_shadows>
        <transparency>0</transparency>
        <material>
          <shader type="pixel">
            <normal_map>default</normal_map>
          </shader>
          <specular>0.1 0.1 0.1 1</specular>
          <ambient>0 0 0 1</ambient>
          <emissive>0 0 0 1</emissive>
          <lighting>0</lighting>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>default</name>
          </script>
          <diffuse>0 0 0 1</diffuse>
        </material>
      </visual>
      <kinematic>0</kinematic>
      <pose frame="">0 0 1 0 0 0</pose>
      <gravity>1</gravity>
      <collision name="collision_1">
        <geometry>
          <empty></empty>
        </geometry>
      </collision>
      <collision name="collision_2">
        <geometry>
          <cylinder>
            <radius>0</radius>
            <length>0</length>
          </cylinder>
        </geometry>
        <max_contacts>10</max_contacts>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <surface>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <min_depth>0</min_depth>
              <max_vel>0.01</max_vel>
              <kd>1</kd>
              <soft_erp>0.2</soft_erp>
            </ode>
          </contact>
          <friction>
            <torsional>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <ode>
                <slip>0</slip>
              </ode>
              <coefficient>1</coefficient>
              <use_patch_radius>1</use_patch_radius>
            </torsional>
            <bullet>
              <fdir1>0 0 0</fdir1>
              <friction>1</friction>
              <rolling_friction>1</rolling_friction>
              <friction2>1</friction2>
            </bullet>
            <ode>
              <mu2>1</mu2>
              <mu>1</mu>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>
      <collision name="collision_2_0">
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <max_contacts>10</max_contacts>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <surface>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <min_depth>0</min_depth>
              <max_vel>0.01</max_vel>
              <kd>1</kd>
              <soft_erp>0.2</soft_erp>
            </ode>
          </contact>
          <friction>
            <torsional>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <ode>
                <slip>0</slip>
              </ode>
              <coefficient>1</coefficient>
              <use_patch_radius>1</use_patch_radius>
            </torsional>
            <bullet>
              <fdir1>0 0 0</fdir1>
              <friction>1</friction>
              <rolling_friction>1</rolling_friction>
              <friction2>1</friction2>
            </bullet>
            <ode>
              <mu2>1</mu2>
              <mu>1</mu>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>
      <collision name="collision_3">
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <max_contacts>10</max_contacts>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <kp>1000000000000.0</kp>
            <min_depth>0</min_depth>
            <max_vel>0.01</max_vel>
            <kd>1</kd>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <surface>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <kp>1000000000000.0</kp>
              <min_depth>0</min_depth>
              <max_vel>0.01</max_vel>
              <kd>1</kd>
              <soft_erp>0.2</soft_erp>
            </ode>
          </contact>
          <friction>
            <torsional>
              <patch_radius>0</patch_radius>
              <surface_radius>0</surface_radius>
              <ode>
                <slip>0</slip>
              </ode>
              <coefficient>1</coefficient>
              <use_patch_radius>1</use_patch_radius>
            </torsional>
            <bullet>
              <fdir1>0 0 0</fdir1>
              <friction>1</friction>
              <rolling_friction>1</rolling_friction>
              <friction2>1</friction2>
            </bullet>
            <ode>
              <mu2>1</mu2>
              <mu>1</mu>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <slip2>0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
    



```python
# You can retrieve the visual geometry by its name
# If the name given is not found, the function will return None
print(link.get_visual_by_name('visual_1'))
print(link.get_visual_by_name('visual_10'))
# Or iterate in the visuals list
for elem in link.visuals:
    print(elem)
```

    <visual name="visual_1">
      <geometry>
        <plane>
          <size>0 0</size>
          <normal>0 0 1</normal>
        </plane>
      </geometry>
    </visual>
    
    None
    <visual name="visual_1">
      <geometry>
        <plane>
          <size>0 0</size>
          <normal>0 0 1</normal>
        </plane>
      </geometry>
    </visual>
    
    <visual name="visual_2">
      <pose frame="">0 0 0 0 0 0</pose>
      <geometry>
        <cylinder>
          <radius>0</radius>
          <length>0</length>
        </cylinder>
      </geometry>
      <cast_shadows>1</cast_shadows>
      <transparency>0</transparency>
      <material>
        <shader type="pixel">
          <normal_map>default</normal_map>
        </shader>
        <specular>0.1 0.1 0.1 1</specular>
        <ambient>0 0 0 1</ambient>
        <emissive>0 0 0 1</emissive>
        <lighting>0</lighting>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>default</name>
        </script>
        <diffuse>0 0 0 1</diffuse>
      </material>
    </visual>
    


## Joints



```python
# The joint is empty by default
joint = create_sdf_element('joint')
print(joint)
```

    <joint name="joint" type="revolute">
      <parent>parent</parent>
      <child>none</child>
    </joint>
    


## Sensors



```python
sensor = create_sdf_element('sensor')
print(sensor)
```

    <sensor name="default" type="altimeter"/>
    



```python
print(sensor.get_modes())
```

    ['none', 'altimeter', 'camera', 'contact', 'depth', 'gps', 'gpu_ray', 'imu', 'logical_camera', 'magnetometer', 'multicamera', 'ray', 'rfid', 'rfidtag', 'sonar', 'wireless_receiver', 'wireless_transmitter', 'force_torque']



```python
sensor.reset(mode='altimeter', with_optional_elements=True)
print(sensor)
```

    <sensor name="default" type="default">
      <topic>none</topic>
      <update_rate>0</update_rate>
      <always_on>0</always_on>
      <altimeter>
        <vertical_position>
          <noise type="none">
            <precision>0</precision>
            <stddev>0</stddev>
            <bias_stddev>0</bias_stddev>
            <bias_mean>0</bias_mean>
            <mean>0</mean>
          </noise>
        </vertical_position>
        <vertical_velocity>
          <noise type="none">
            <precision>0</precision>
            <stddev>0</stddev>
            <bias_stddev>0</bias_stddev>
            <bias_mean>0</bias_mean>
            <mean>0</mean>
          </noise>
        </vertical_velocity>
      </altimeter>
      <pose frame="">0 0 0 0 0 0</pose>
      <visualize>0</visualize>
      <plugin filename="" name=""/>
    </sensor>
    



```python
sensor.reset(mode='camera', with_optional_elements=True)
print(sensor)
```

    <sensor name="default" type="default">
      <topic>none</topic>
      <update_rate>0</update_rate>
      <always_on>0</always_on>
      <pose frame="">0 0 0 0 0 0</pose>
      <visualize>0</visualize>
      <plugin filename="" name=""/>
      <camera name="default">
        <clip>
          <near>100</near>
          <far>0.1</far>
        </clip>
        <distortion>
          <k3>0</k3>
          <k2>0</k2>
          <k1>0</k1>
          <center>0.5 0.5</center>
          <p2>0</p2>
          <p1>0</p1>
        </distortion>
        <noise type="none">
          <precision>0</precision>
          <stddev>0</stddev>
          <bias_stddev>0</bias_stddev>
          <bias_mean>0</bias_mean>
          <mean>0</mean>
        </noise>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>320</width>
          <format>R8G8B8</format>
          <height>1</height>
        </image>
        <depth_camera>
          <output>depths</output>
        </depth_camera>
        <save enabled="False">
          <path>__default__</path>
        </save>
      </camera>
    </sensor>
    



```python
sensor.reset(mode='force_torque', with_optional_elements=True)
print(sensor)
```

    <sensor name="default" type="default">
      <topic>none</topic>
      <update_rate>0</update_rate>
      <always_on>0</always_on>
      <pose frame="">0 0 0 0 0 0</pose>
      <visualize>0</visualize>
      <plugin filename="" name=""/>
      <force_torque>
        <frame>child</frame>
        <measure_direction>child_to_parent</measure_direction>
      </force_torque>
    </sensor>
    



```python

```
