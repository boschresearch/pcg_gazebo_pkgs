
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
        <mass>30.0</mass>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <inertia>
          <ixy>0</ixy>
          <iyz>0</iyz>
          <iyy>0.5</iyy>
          <ixx>0.5</ixx>
          <ixz>0</ixz>
          <izz>0.5</izz>
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
        <mass>30.0</mass>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <inertia>
          <ixy>0</ixy>
          <iyz>0</iyz>
          <iyy>0.5</iyy>
          <ixx>0.5</ixx>
          <ixz>0</ixz>
          <izz>0.5</izz>
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
        <mass>30.0</mass>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <inertia>
          <ixy>0</ixy>
          <iyz>0</iyz>
          <iyy>0.5</iyy>
          <ixx>0.5</ixx>
          <ixz>0</ixz>
          <izz>0.5</izz>
        </inertia>
      </inertial>
      <gravity>1</gravity>
      <kinematic>0</kinematic>
    </link>
    



```python
# The pose of the link with respect to a frame
link.pose = [0, 0, 1, 0, 0, 0]
print(link)
```

    <link name="base_link">
      <inertial>
        <mass>30.0</mass>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <inertia>
          <ixy>0</ixy>
          <iyz>0</iyz>
          <iyy>0.5</iyy>
          <ixx>0.5</ixx>
          <ixz>0</ixz>
          <izz>0.5</izz>
        </inertia>
      </inertial>
      <pose frame="">0 0 1 0 0 0</pose>
      <gravity>1</gravity>
      <kinematic>0</kinematic>
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
        <mass>30.0</mass>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <inertia>
          <ixy>0</ixy>
          <iyz>0</iyz>
          <iyy>0.5</iyy>
          <ixx>0.5</ixx>
          <ixz>0</ixz>
          <izz>0.5</izz>
        </inertia>
      </inertial>
      <pose frame="">0 0 1 0 0 0</pose>
      <collision name="collision_1">
        <geometry>
          <empty></empty>
        </geometry>
      </collision>
      <gravity>1</gravity>
      <kinematic>0</kinematic>
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
        <mass>30.0</mass>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <inertia>
          <ixy>0</ixy>
          <iyz>0</iyz>
          <iyy>0.5</iyy>
          <ixx>0.5</ixx>
          <ixz>0</ixz>
          <izz>0.5</izz>
        </inertia>
      </inertial>
      <pose frame="">0 0 1 0 0 0</pose>
      <collision name="collision_1">
        <geometry>
          <empty></empty>
        </geometry>
      </collision>
      <collision name="collision_2">
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <max_contacts>10</max_contacts>
        <geometry>
          <cylinder>
            <length>0</length>
            <radius>0</radius>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <bullet>
              <friction2>1</friction2>
              <fdir1>0 0 0</fdir1>
              <rolling_friction>1</rolling_friction>
              <friction>1</friction>
            </bullet>
            <torsional>
              <patch_radius>0</patch_radius>
              <coefficient>1</coefficient>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
            <ode>
              <mu2>1</mu2>
              <slip2>0</slip2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <mu>1</mu>
            </ode>
          </friction>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <kp>1000000000000.0</kp>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <max_vel>0.01</max_vel>
              <soft_erp>0.2</soft_erp>
              <kd>1</kd>
              <min_depth>0</min_depth>
              <kp>1000000000000.0</kp>
            </ode>
          </contact>
          <bounce>
            <threshold>100000</threshold>
            <restitution_coefficient>0</restitution_coefficient>
          </bounce>
        </surface>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
      </collision>
      <gravity>1</gravity>
      <kinematic>0</kinematic>
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
        <mass>30.0</mass>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <inertia>
          <ixy>0</ixy>
          <iyz>0</iyz>
          <iyy>0.5</iyy>
          <ixx>0.5</ixx>
          <ixz>0</ixz>
          <izz>0.5</izz>
        </inertia>
      </inertial>
      <pose frame="">0 0 1 0 0 0</pose>
      <collision name="collision_1">
        <geometry>
          <empty></empty>
        </geometry>
      </collision>
      <collision name="collision_2">
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <max_contacts>10</max_contacts>
        <geometry>
          <cylinder>
            <length>0</length>
            <radius>0</radius>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <bullet>
              <friction2>1</friction2>
              <fdir1>0 0 0</fdir1>
              <rolling_friction>1</rolling_friction>
              <friction>1</friction>
            </bullet>
            <torsional>
              <patch_radius>0</patch_radius>
              <coefficient>1</coefficient>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
            <ode>
              <mu2>1</mu2>
              <slip2>0</slip2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <mu>1</mu>
            </ode>
          </friction>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <kp>1000000000000.0</kp>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <max_vel>0.01</max_vel>
              <soft_erp>0.2</soft_erp>
              <kd>1</kd>
              <min_depth>0</min_depth>
              <kp>1000000000000.0</kp>
            </ode>
          </contact>
          <bounce>
            <threshold>100000</threshold>
            <restitution_coefficient>0</restitution_coefficient>
          </bounce>
        </surface>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
      </collision>
      <collision name="collision_2_0">
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <max_contacts>10</max_contacts>
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <bullet>
              <friction2>1</friction2>
              <fdir1>0 0 0</fdir1>
              <rolling_friction>1</rolling_friction>
              <friction>1</friction>
            </bullet>
            <torsional>
              <patch_radius>0</patch_radius>
              <coefficient>1</coefficient>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
            <ode>
              <mu2>1</mu2>
              <slip2>0</slip2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <mu>1</mu>
            </ode>
          </friction>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <kp>1000000000000.0</kp>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <max_vel>0.01</max_vel>
              <soft_erp>0.2</soft_erp>
              <kd>1</kd>
              <min_depth>0</min_depth>
              <kp>1000000000000.0</kp>
            </ode>
          </contact>
          <bounce>
            <threshold>100000</threshold>
            <restitution_coefficient>0</restitution_coefficient>
          </bounce>
        </surface>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
      </collision>
      <gravity>1</gravity>
      <kinematic>0</kinematic>
    </link>
    



```python
link.add_collision('collision_3', collision)
print(link)
```

    <link name="base_link">
      <inertial>
        <mass>30.0</mass>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <inertia>
          <ixy>0</ixy>
          <iyz>0</iyz>
          <iyy>0.5</iyy>
          <ixx>0.5</ixx>
          <ixz>0</ixz>
          <izz>0.5</izz>
        </inertia>
      </inertial>
      <pose frame="">0 0 1 0 0 0</pose>
      <collision name="collision_1">
        <geometry>
          <empty></empty>
        </geometry>
      </collision>
      <collision name="collision_2">
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <max_contacts>10</max_contacts>
        <geometry>
          <cylinder>
            <length>0</length>
            <radius>0</radius>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <bullet>
              <friction2>1</friction2>
              <fdir1>0 0 0</fdir1>
              <rolling_friction>1</rolling_friction>
              <friction>1</friction>
            </bullet>
            <torsional>
              <patch_radius>0</patch_radius>
              <coefficient>1</coefficient>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
            <ode>
              <mu2>1</mu2>
              <slip2>0</slip2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <mu>1</mu>
            </ode>
          </friction>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <kp>1000000000000.0</kp>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <max_vel>0.01</max_vel>
              <soft_erp>0.2</soft_erp>
              <kd>1</kd>
              <min_depth>0</min_depth>
              <kp>1000000000000.0</kp>
            </ode>
          </contact>
          <bounce>
            <threshold>100000</threshold>
            <restitution_coefficient>0</restitution_coefficient>
          </bounce>
        </surface>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
      </collision>
      <collision name="collision_2_0">
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <max_contacts>10</max_contacts>
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <bullet>
              <friction2>1</friction2>
              <fdir1>0 0 0</fdir1>
              <rolling_friction>1</rolling_friction>
              <friction>1</friction>
            </bullet>
            <torsional>
              <patch_radius>0</patch_radius>
              <coefficient>1</coefficient>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
            <ode>
              <mu2>1</mu2>
              <slip2>0</slip2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <mu>1</mu>
            </ode>
          </friction>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <kp>1000000000000.0</kp>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <max_vel>0.01</max_vel>
              <soft_erp>0.2</soft_erp>
              <kd>1</kd>
              <min_depth>0</min_depth>
              <kp>1000000000000.0</kp>
            </ode>
          </contact>
          <bounce>
            <threshold>100000</threshold>
            <restitution_coefficient>0</restitution_coefficient>
          </bounce>
        </surface>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
      </collision>
      <collision name="collision_3">
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <max_contacts>10</max_contacts>
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <bullet>
              <friction2>1</friction2>
              <fdir1>0 0 0</fdir1>
              <rolling_friction>1</rolling_friction>
              <friction>1</friction>
            </bullet>
            <torsional>
              <patch_radius>0</patch_radius>
              <coefficient>1</coefficient>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
            <ode>
              <mu2>1</mu2>
              <slip2>0</slip2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <mu>1</mu>
            </ode>
          </friction>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <kp>1000000000000.0</kp>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <max_vel>0.01</max_vel>
              <soft_erp>0.2</soft_erp>
              <kd>1</kd>
              <min_depth>0</min_depth>
              <kp>1000000000000.0</kp>
            </ode>
          </contact>
          <bounce>
            <threshold>100000</threshold>
            <restitution_coefficient>0</restitution_coefficient>
          </bounce>
        </surface>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
      </collision>
      <gravity>1</gravity>
      <kinematic>0</kinematic>
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
      <laser_retro>0</laser_retro>
      <pose frame="">0 0 0 0 0 0</pose>
      <max_contacts>10</max_contacts>
      <geometry>
        <cylinder>
          <length>0</length>
          <radius>0</radius>
        </cylinder>
      </geometry>
      <surface>
        <friction>
          <bullet>
            <friction2>1</friction2>
            <fdir1>0 0 0</fdir1>
            <rolling_friction>1</rolling_friction>
            <friction>1</friction>
          </bullet>
          <torsional>
            <patch_radius>0</patch_radius>
            <coefficient>1</coefficient>
            <surface_radius>0</surface_radius>
            <use_patch_radius>1</use_patch_radius>
            <ode>
              <slip>0</slip>
            </ode>
          </torsional>
          <ode>
            <mu2>1</mu2>
            <slip2>0</slip2>
            <fdir1>0 0 0</fdir1>
            <slip1>0</slip1>
            <mu>1</mu>
          </ode>
        </friction>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
        <bounce>
          <threshold>100000</threshold>
          <restitution_coefficient>0</restitution_coefficient>
        </bounce>
      </surface>
      <contact>
        <bullet>
          <soft_cfm>0</soft_cfm>
          <soft_erp>0.2</soft_erp>
          <split_impulse>1</split_impulse>
          <kd>1</kd>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kp>1000000000000.0</kp>
        </bullet>
        <ode>
          <soft_cfm>0</soft_cfm>
          <max_vel>0.01</max_vel>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <min_depth>0</min_depth>
          <kp>1000000000000.0</kp>
        </ode>
      </contact>
    </collision>
    
    <collision name="collision_2_0">
      <laser_retro>0</laser_retro>
      <pose frame="">0 0 0 0 0 0</pose>
      <max_contacts>10</max_contacts>
      <geometry>
        <box>
          <size>0 0 0</size>
        </box>
      </geometry>
      <surface>
        <friction>
          <bullet>
            <friction2>1</friction2>
            <fdir1>0 0 0</fdir1>
            <rolling_friction>1</rolling_friction>
            <friction>1</friction>
          </bullet>
          <torsional>
            <patch_radius>0</patch_radius>
            <coefficient>1</coefficient>
            <surface_radius>0</surface_radius>
            <use_patch_radius>1</use_patch_radius>
            <ode>
              <slip>0</slip>
            </ode>
          </torsional>
          <ode>
            <mu2>1</mu2>
            <slip2>0</slip2>
            <fdir1>0 0 0</fdir1>
            <slip1>0</slip1>
            <mu>1</mu>
          </ode>
        </friction>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
        <bounce>
          <threshold>100000</threshold>
          <restitution_coefficient>0</restitution_coefficient>
        </bounce>
      </surface>
      <contact>
        <bullet>
          <soft_cfm>0</soft_cfm>
          <soft_erp>0.2</soft_erp>
          <split_impulse>1</split_impulse>
          <kd>1</kd>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kp>1000000000000.0</kp>
        </bullet>
        <ode>
          <soft_cfm>0</soft_cfm>
          <max_vel>0.01</max_vel>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <min_depth>0</min_depth>
          <kp>1000000000000.0</kp>
        </ode>
      </contact>
    </collision>
    
    <collision name="collision_3">
      <laser_retro>0</laser_retro>
      <pose frame="">0 0 0 0 0 0</pose>
      <max_contacts>10</max_contacts>
      <geometry>
        <box>
          <size>0 0 0</size>
        </box>
      </geometry>
      <surface>
        <friction>
          <bullet>
            <friction2>1</friction2>
            <fdir1>0 0 0</fdir1>
            <rolling_friction>1</rolling_friction>
            <friction>1</friction>
          </bullet>
          <torsional>
            <patch_radius>0</patch_radius>
            <coefficient>1</coefficient>
            <surface_radius>0</surface_radius>
            <use_patch_radius>1</use_patch_radius>
            <ode>
              <slip>0</slip>
            </ode>
          </torsional>
          <ode>
            <mu2>1</mu2>
            <slip2>0</slip2>
            <fdir1>0 0 0</fdir1>
            <slip1>0</slip1>
            <mu>1</mu>
          </ode>
        </friction>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
        <bounce>
          <threshold>100000</threshold>
          <restitution_coefficient>0</restitution_coefficient>
        </bounce>
      </surface>
      <contact>
        <bullet>
          <soft_cfm>0</soft_cfm>
          <soft_erp>0.2</soft_erp>
          <split_impulse>1</split_impulse>
          <kd>1</kd>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <kp>1000000000000.0</kp>
        </bullet>
        <ode>
          <soft_cfm>0</soft_cfm>
          <max_vel>0.01</max_vel>
          <soft_erp>0.2</soft_erp>
          <kd>1</kd>
          <min_depth>0</min_depth>
          <kp>1000000000000.0</kp>
        </ode>
      </contact>
    </collision>
    



```python
# The same is true for visual elements, create an empty visual element by using add_visual
link.add_visual('visual_1')
print(link)
```

    <link name="base_link">
      <inertial>
        <mass>30.0</mass>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <inertia>
          <ixy>0</ixy>
          <iyz>0</iyz>
          <iyy>0.5</iyy>
          <ixx>0.5</ixx>
          <ixz>0</ixz>
          <izz>0.5</izz>
        </inertia>
      </inertial>
      <gravity>1</gravity>
      <pose frame="">0 0 1 0 0 0</pose>
      <collision name="collision_1">
        <geometry>
          <empty></empty>
        </geometry>
      </collision>
      <collision name="collision_2">
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <max_contacts>10</max_contacts>
        <geometry>
          <cylinder>
            <length>0</length>
            <radius>0</radius>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <bullet>
              <friction2>1</friction2>
              <fdir1>0 0 0</fdir1>
              <rolling_friction>1</rolling_friction>
              <friction>1</friction>
            </bullet>
            <torsional>
              <patch_radius>0</patch_radius>
              <coefficient>1</coefficient>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
            <ode>
              <mu2>1</mu2>
              <slip2>0</slip2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <mu>1</mu>
            </ode>
          </friction>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <kp>1000000000000.0</kp>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <max_vel>0.01</max_vel>
              <soft_erp>0.2</soft_erp>
              <kd>1</kd>
              <min_depth>0</min_depth>
              <kp>1000000000000.0</kp>
            </ode>
          </contact>
          <bounce>
            <threshold>100000</threshold>
            <restitution_coefficient>0</restitution_coefficient>
          </bounce>
        </surface>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
      </collision>
      <collision name="collision_2_0">
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <max_contacts>10</max_contacts>
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <bullet>
              <friction2>1</friction2>
              <fdir1>0 0 0</fdir1>
              <rolling_friction>1</rolling_friction>
              <friction>1</friction>
            </bullet>
            <torsional>
              <patch_radius>0</patch_radius>
              <coefficient>1</coefficient>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
            <ode>
              <mu2>1</mu2>
              <slip2>0</slip2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <mu>1</mu>
            </ode>
          </friction>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <kp>1000000000000.0</kp>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <max_vel>0.01</max_vel>
              <soft_erp>0.2</soft_erp>
              <kd>1</kd>
              <min_depth>0</min_depth>
              <kp>1000000000000.0</kp>
            </ode>
          </contact>
          <bounce>
            <threshold>100000</threshold>
            <restitution_coefficient>0</restitution_coefficient>
          </bounce>
        </surface>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
      </collision>
      <collision name="collision_3">
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <max_contacts>10</max_contacts>
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <bullet>
              <friction2>1</friction2>
              <fdir1>0 0 0</fdir1>
              <rolling_friction>1</rolling_friction>
              <friction>1</friction>
            </bullet>
            <torsional>
              <patch_radius>0</patch_radius>
              <coefficient>1</coefficient>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
            <ode>
              <mu2>1</mu2>
              <slip2>0</slip2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <mu>1</mu>
            </ode>
          </friction>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <kp>1000000000000.0</kp>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <max_vel>0.01</max_vel>
              <soft_erp>0.2</soft_erp>
              <kd>1</kd>
              <min_depth>0</min_depth>
              <kp>1000000000000.0</kp>
            </ode>
          </contact>
          <bounce>
            <threshold>100000</threshold>
            <restitution_coefficient>0</restitution_coefficient>
          </bounce>
        </surface>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
      </collision>
      <kinematic>0</kinematic>
      <visual name="visual_1">
        <geometry>
          <empty></empty>
        </geometry>
      </visual>
    </link>
    



```python
# Set the geometry of the visual element
link.visuals[0].geometry.plane = create_sdf_element('plane')
print(link)
```

    <link name="base_link">
      <inertial>
        <mass>30.0</mass>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <inertia>
          <ixy>0</ixy>
          <iyz>0</iyz>
          <iyy>0.5</iyy>
          <ixx>0.5</ixx>
          <ixz>0</ixz>
          <izz>0.5</izz>
        </inertia>
      </inertial>
      <gravity>1</gravity>
      <pose frame="">0 0 1 0 0 0</pose>
      <collision name="collision_1">
        <geometry>
          <empty></empty>
        </geometry>
      </collision>
      <collision name="collision_2">
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <max_contacts>10</max_contacts>
        <geometry>
          <cylinder>
            <length>0</length>
            <radius>0</radius>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <bullet>
              <friction2>1</friction2>
              <fdir1>0 0 0</fdir1>
              <rolling_friction>1</rolling_friction>
              <friction>1</friction>
            </bullet>
            <torsional>
              <patch_radius>0</patch_radius>
              <coefficient>1</coefficient>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
            <ode>
              <mu2>1</mu2>
              <slip2>0</slip2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <mu>1</mu>
            </ode>
          </friction>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <kp>1000000000000.0</kp>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <max_vel>0.01</max_vel>
              <soft_erp>0.2</soft_erp>
              <kd>1</kd>
              <min_depth>0</min_depth>
              <kp>1000000000000.0</kp>
            </ode>
          </contact>
          <bounce>
            <threshold>100000</threshold>
            <restitution_coefficient>0</restitution_coefficient>
          </bounce>
        </surface>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
      </collision>
      <collision name="collision_2_0">
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <max_contacts>10</max_contacts>
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <bullet>
              <friction2>1</friction2>
              <fdir1>0 0 0</fdir1>
              <rolling_friction>1</rolling_friction>
              <friction>1</friction>
            </bullet>
            <torsional>
              <patch_radius>0</patch_radius>
              <coefficient>1</coefficient>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
            <ode>
              <mu2>1</mu2>
              <slip2>0</slip2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <mu>1</mu>
            </ode>
          </friction>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <kp>1000000000000.0</kp>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <max_vel>0.01</max_vel>
              <soft_erp>0.2</soft_erp>
              <kd>1</kd>
              <min_depth>0</min_depth>
              <kp>1000000000000.0</kp>
            </ode>
          </contact>
          <bounce>
            <threshold>100000</threshold>
            <restitution_coefficient>0</restitution_coefficient>
          </bounce>
        </surface>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
      </collision>
      <collision name="collision_3">
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <max_contacts>10</max_contacts>
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <bullet>
              <friction2>1</friction2>
              <fdir1>0 0 0</fdir1>
              <rolling_friction>1</rolling_friction>
              <friction>1</friction>
            </bullet>
            <torsional>
              <patch_radius>0</patch_radius>
              <coefficient>1</coefficient>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
            <ode>
              <mu2>1</mu2>
              <slip2>0</slip2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <mu>1</mu>
            </ode>
          </friction>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <kp>1000000000000.0</kp>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <max_vel>0.01</max_vel>
              <soft_erp>0.2</soft_erp>
              <kd>1</kd>
              <min_depth>0</min_depth>
              <kp>1000000000000.0</kp>
            </ode>
          </contact>
          <bounce>
            <threshold>100000</threshold>
            <restitution_coefficient>0</restitution_coefficient>
          </bounce>
        </surface>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
      </collision>
      <kinematic>0</kinematic>
      <visual name="visual_1">
        <geometry>
          <plane>
            <size>0 0</size>
            <normal>0 0 1</normal>
          </plane>
        </geometry>
      </visual>
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
        <mass>30.0</mass>
        <pose frame="">0.0 10.0 0.0 0 0 0</pose>
        <inertia>
          <ixy>0</ixy>
          <iyz>0</iyz>
          <iyy>0.5</iyy>
          <ixx>0.5</ixx>
          <ixz>0</ixz>
          <izz>0.5</izz>
        </inertia>
      </inertial>
      <gravity>1</gravity>
      <pose frame="">0 0 1 0 0 0</pose>
      <collision name="collision_1">
        <geometry>
          <empty></empty>
        </geometry>
      </collision>
      <collision name="collision_2">
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <max_contacts>10</max_contacts>
        <geometry>
          <cylinder>
            <length>0</length>
            <radius>0</radius>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <bullet>
              <friction2>1</friction2>
              <fdir1>0 0 0</fdir1>
              <rolling_friction>1</rolling_friction>
              <friction>1</friction>
            </bullet>
            <torsional>
              <patch_radius>0</patch_radius>
              <coefficient>1</coefficient>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
            <ode>
              <mu2>1</mu2>
              <slip2>0</slip2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <mu>1</mu>
            </ode>
          </friction>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <kp>1000000000000.0</kp>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <max_vel>0.01</max_vel>
              <soft_erp>0.2</soft_erp>
              <kd>1</kd>
              <min_depth>0</min_depth>
              <kp>1000000000000.0</kp>
            </ode>
          </contact>
          <bounce>
            <threshold>100000</threshold>
            <restitution_coefficient>0</restitution_coefficient>
          </bounce>
        </surface>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
      </collision>
      <collision name="collision_2_0">
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <max_contacts>10</max_contacts>
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <bullet>
              <friction2>1</friction2>
              <fdir1>0 0 0</fdir1>
              <rolling_friction>1</rolling_friction>
              <friction>1</friction>
            </bullet>
            <torsional>
              <patch_radius>0</patch_radius>
              <coefficient>1</coefficient>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
            <ode>
              <mu2>1</mu2>
              <slip2>0</slip2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <mu>1</mu>
            </ode>
          </friction>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <kp>1000000000000.0</kp>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <max_vel>0.01</max_vel>
              <soft_erp>0.2</soft_erp>
              <kd>1</kd>
              <min_depth>0</min_depth>
              <kp>1000000000000.0</kp>
            </ode>
          </contact>
          <bounce>
            <threshold>100000</threshold>
            <restitution_coefficient>0</restitution_coefficient>
          </bounce>
        </surface>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
      </collision>
      <collision name="collision_3">
        <laser_retro>0</laser_retro>
        <pose frame="">0 0 0 0 0 0</pose>
        <max_contacts>10</max_contacts>
        <geometry>
          <box>
            <size>0 0 0</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <bullet>
              <friction2>1</friction2>
              <fdir1>0 0 0</fdir1>
              <rolling_friction>1</rolling_friction>
              <friction>1</friction>
            </bullet>
            <torsional>
              <patch_radius>0</patch_radius>
              <coefficient>1</coefficient>
              <surface_radius>0</surface_radius>
              <use_patch_radius>1</use_patch_radius>
              <ode>
                <slip>0</slip>
              </ode>
            </torsional>
            <ode>
              <mu2>1</mu2>
              <slip2>0</slip2>
              <fdir1>0 0 0</fdir1>
              <slip1>0</slip1>
              <mu>1</mu>
            </ode>
          </friction>
          <contact>
            <bullet>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <split_impulse>1</split_impulse>
              <kd>1</kd>
              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
              <kp>1000000000000.0</kp>
            </bullet>
            <ode>
              <soft_cfm>0</soft_cfm>
              <max_vel>0.01</max_vel>
              <soft_erp>0.2</soft_erp>
              <kd>1</kd>
              <min_depth>0</min_depth>
              <kp>1000000000000.0</kp>
            </ode>
          </contact>
          <bounce>
            <threshold>100000</threshold>
            <restitution_coefficient>0</restitution_coefficient>
          </bounce>
        </surface>
        <contact>
          <bullet>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <split_impulse>1</split_impulse>
            <kd>1</kd>
            <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
            <kp>1000000000000.0</kp>
          </bullet>
          <ode>
            <soft_cfm>0</soft_cfm>
            <max_vel>0.01</max_vel>
            <soft_erp>0.2</soft_erp>
            <kd>1</kd>
            <min_depth>0</min_depth>
            <kp>1000000000000.0</kp>
          </ode>
        </contact>
      </collision>
      <kinematic>0</kinematic>
      <visual name="visual_1">
        <geometry>
          <plane>
            <size>0 0</size>
            <normal>0 0 1</normal>
          </plane>
        </geometry>
      </visual>
      <visual name="visual_2">
        <transparency>0</transparency>
        <geometry>
          <cylinder>
            <length>0</length>
            <radius>0</radius>
          </cylinder>
        </geometry>
        <material>
          <emissive>0 0 0 1</emissive>
          <lighting>0</lighting>
          <shader type="pixel">
            <normal_map>default</normal_map>
          </shader>
          <diffuse>0 0 0 1</diffuse>
          <ambient>0 0 0 1</ambient>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>default</name>
          </script>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
        <cast_shadows>1</cast_shadows>
        <pose frame="">0 0 0 0 0 0</pose>
      </visual>
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
      <transparency>0</transparency>
      <geometry>
        <cylinder>
          <length>0</length>
          <radius>0</radius>
        </cylinder>
      </geometry>
      <material>
        <emissive>0 0 0 1</emissive>
        <lighting>0</lighting>
        <shader type="pixel">
          <normal_map>default</normal_map>
        </shader>
        <diffuse>0 0 0 1</diffuse>
        <ambient>0 0 0 1</ambient>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>default</name>
        </script>
        <specular>0.1 0.1 0.1 1</specular>
      </material>
      <cast_shadows>1</cast_shadows>
      <pose frame="">0 0 0 0 0 0</pose>
    </visual>
    


## Joints



```python
# The joint is empty by default
joint = create_sdf_element('joint')
print(joint)
```

    <joint name="joint" type="revolute">
      <child>none</child>
      <parent>parent</parent>
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

    type child element not available for version 1.6
    accel child element not available for version 1.6
    rate child element not available for version 1.6
    type child element not available for version 1.6
    accel child element not available for version 1.6
    rate child element not available for version 1.6
    <sensor name="default" type="default">
      <pose frame="">0 0 0 0 0 0</pose>
      <always_on>0</always_on>
      <plugin filename="" name=""/>
      <topic>none</topic>
      <altimeter>
        <vertical_velocity>
          <noise type="none">
            <bias_stddev>0</bias_stddev>
            <bias_mean>0</bias_mean>
            <mean>0</mean>
            <stddev>0</stddev>
            <precision>0</precision>
          </noise>
        </vertical_velocity>
        <vertical_position>
          <noise type="none">
            <bias_stddev>0</bias_stddev>
            <bias_mean>0</bias_mean>
            <mean>0</mean>
            <stddev>0</stddev>
            <precision>0</precision>
          </noise>
        </vertical_position>
      </altimeter>
      <visualize>0</visualize>
      <update_rate>0</update_rate>
    </sensor>
    



```python
sensor.reset(mode='camera', with_optional_elements=True)
print(sensor)
```

    type child element not available for version 1.6
    accel child element not available for version 1.6
    rate child element not available for version 1.6
    <sensor name="default" type="default">
      <pose frame="">0 0 0 0 0 0</pose>
      <always_on>0</always_on>
      <plugin filename="" name=""/>
      <topic>none</topic>
      <camera name="default">
        <distortion>
          <center>0.5 0.5</center>
          <k2>0</k2>
          <p2>0</p2>
          <p1>0</p1>
          <k3>0</k3>
          <k1>0</k1>
        </distortion>
        <horizontal_fov>1.047</horizontal_fov>
        <depth_camera>
          <output>depths</output>
        </depth_camera>
        <save enabled="False">
          <path>__default__</path>
        </save>
        <noise type="none">
          <bias_stddev>0</bias_stddev>
          <bias_mean>0</bias_mean>
          <mean>0</mean>
          <stddev>0</stddev>
          <precision>0</precision>
        </noise>
        <image>
          <format>R8G8B8</format>
          <width>320</width>
          <height>1</height>
        </image>
        <clip>
          <near>100</near>
          <far>0.1</far>
        </clip>
      </camera>
      <visualize>0</visualize>
      <update_rate>0</update_rate>
    </sensor>
    



```python
sensor.reset(mode='force_torque', with_optional_elements=True)
print(sensor)
```

    <sensor name="default" type="default">
      <pose frame="">0 0 0 0 0 0</pose>
      <always_on>0</always_on>
      <plugin filename="" name=""/>
      <topic>none</topic>
      <visualize>0</visualize>
      <force_torque>
        <measure_direction>child_to_parent</measure_direction>
        <frame>child</frame>
      </force_torque>
      <update_rate>0</update_rate>
    </sensor>
    



```python

```
