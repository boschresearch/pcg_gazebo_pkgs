
# Converting from SDF file


```python
import os
from pcg_gazebo.parsers import parse_xml
```


```python
def download_sdf(url, model_name):
    output_sdf = '/tmp/{}.sdf'.format(model_name)
    os.system('wget {} -O {}'.format(url, output_sdf))
    
    with open(output_sdf, 'r') as f:
        for line in f:
            print(line.replace('\n', ''))
        
    return output_sdf    
```

# Bookshelf


```python
url = 'https://bitbucket.org/osrf/gazebo_models/raw/9533d55593096e7ebdfb539e99d2bf9cb1bff347/bookshelf/model.sdf'
obj = parse_xml(download_sdf(url, 'bookshelf'), type='sdf')


```

    <?xml version="1.0" ?>
    <sdf version="1.5">
      <model name="bookshelf">
        <static>true</static>
        <link name="link">
          <inertial>
            <mass>1.0</mass>
          </inertial>
          <collision name="back">
            <pose>0 0.005 0.6 0 0 0</pose>
            <geometry>
              <box>
                <size>0.9 0.01 1.2</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual1">
            <pose>0 0.005 0.6 0 0 0</pose>
            <geometry>
              <box>
                <size>0.9 0.01 1.2</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Wood</name>
              </script>
            </material>
          </visual>
          <collision name="left_side">
            <pose>0.45 -0.195 0.6 0 0 0</pose>
            <geometry>
              <box>
                <size>0.02 0.4 1.2</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual2">
            <pose>0.45 -0.195 0.6 0 0 0</pose>
            <geometry>
              <box>
                <size>0.02 0.4 1.2</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Wood</name>
              </script>
            </material>
          </visual>
          <collision name="right_side">
            <pose>-0.45 -0.195 0.6 0 0 0</pose>
            <geometry>
              <box>
                <size>0.02 0.4 1.2</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual3">
            <pose>-0.45 -0.195 0.6 0 0 0</pose>
            <geometry>
              <box>
                <size>0.02 0.4 1.2</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Wood</name>
              </script>
            </material>
          </visual>
          <collision name="bottom">
            <pose>0 -0.195 0.03 0 0 0</pose>
            <geometry>
              <box>
                <size>0.88 0.4 0.06</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual4">
            <pose>0 -0.195 0.03 0 0 0</pose>
            <geometry>
              <box>
                <size>0.88 0.4 0.06</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Wood</name>
              </script>
            </material>
          </visual>
          <collision name="top">
            <pose>0 -0.195 1.19 0 0 0</pose>
            <geometry>
              <box>
                <size>0.88 0.4 0.02</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual5">
            <pose>0 -0.195 1.19 0 0 0</pose>
            <geometry>
              <box>
                <size>0.88 0.4 0.02</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Wood</name>
              </script>
            </material>
          </visual>
          <collision name="low_shelf">
            <pose>0 -0.195 0.43 0 0 0</pose>
            <geometry>
              <box>
                <size>0.88 0.4 0.02</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual6">
            <pose>0 -0.195 0.43 0 0 0</pose>
            <geometry>
              <box>
                <size>0.88 0.4 0.02</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Wood</name>
              </script>
            </material>
          </visual>
          <collision name="high_shelf">
            <pose>0 -0.195 0.8 0 0 0</pose>
            <geometry>
              <box>
                <size>0.88 0.4 0.02</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual7">
            <pose>0 -0.195 0.8 0 0 0</pose>
            <geometry>
              <box>
                <size>0.88 0.4 0.02</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Wood</name>
              </script>
            </material>
          </visual>
        </link>
      </model>
    </sdf>



```python
print(obj)
```

    <sdf version="1.5">
      <model name="bookshelf">
        <link name="link">
          <inertial>
            <mass>1.0</mass>
            <inertia>
              <ixz>0</ixz>
              <iyz>0</iyz>
              <iyy>0</iyy>
              <izz>0</izz>
              <ixx>0</ixx>
              <ixy>0</ixy>
            </inertia>
            <pose frame="">0 0 0 0 0 0</pose>
          </inertial>
          <visual name="visual1">
            <pose frame="">0.0 0.005 0.6 0.0 0.0 0.0</pose>
            <geometry>
              <box>
                <size>0.9 0.01 1.2</size>
              </box>
            </geometry>
            <material>
              <script>
                <name>Gazebo/Wood</name>
                <uri>file://media/materials/scripts/gazebo.material</uri>
              </script>
            </material>
          </visual>
          <visual name="visual2">
            <pose frame="">0.45 -0.195 0.6 0.0 0.0 0.0</pose>
            <geometry>
              <box>
                <size>0.02 0.4 1.2</size>
              </box>
            </geometry>
            <material>
              <script>
                <name>Gazebo/Wood</name>
                <uri>file://media/materials/scripts/gazebo.material</uri>
              </script>
            </material>
          </visual>
          <visual name="visual3">
            <pose frame="">-0.45 -0.195 0.6 0.0 0.0 0.0</pose>
            <geometry>
              <box>
                <size>0.02 0.4 1.2</size>
              </box>
            </geometry>
            <material>
              <script>
                <name>Gazebo/Wood</name>
                <uri>file://media/materials/scripts/gazebo.material</uri>
              </script>
            </material>
          </visual>
          <visual name="visual4">
            <pose frame="">0.0 -0.195 0.03 0.0 0.0 0.0</pose>
            <geometry>
              <box>
                <size>0.88 0.4 0.06</size>
              </box>
            </geometry>
            <material>
              <script>
                <name>Gazebo/Wood</name>
                <uri>file://media/materials/scripts/gazebo.material</uri>
              </script>
            </material>
          </visual>
          <visual name="visual5">
            <pose frame="">0.0 -0.195 1.19 0.0 0.0 0.0</pose>
            <geometry>
              <box>
                <size>0.88 0.4 0.02</size>
              </box>
            </geometry>
            <material>
              <script>
                <name>Gazebo/Wood</name>
                <uri>file://media/materials/scripts/gazebo.material</uri>
              </script>
            </material>
          </visual>
          <visual name="visual6">
            <pose frame="">0.0 -0.195 0.43 0.0 0.0 0.0</pose>
            <geometry>
              <box>
                <size>0.88 0.4 0.02</size>
              </box>
            </geometry>
            <material>
              <script>
                <name>Gazebo/Wood</name>
                <uri>file://media/materials/scripts/gazebo.material</uri>
              </script>
            </material>
          </visual>
          <visual name="visual7">
            <pose frame="">0.0 -0.195 0.8 0.0 0.0 0.0</pose>
            <geometry>
              <box>
                <size>0.88 0.4 0.02</size>
              </box>
            </geometry>
            <material>
              <script>
                <name>Gazebo/Wood</name>
                <uri>file://media/materials/scripts/gazebo.material</uri>
              </script>
            </material>
          </visual>
          <collision name="back">
            <pose frame="">0.0 0.005 0.6 0.0 0.0 0.0</pose>
            <geometry>
              <box>
                <size>0.9 0.01 1.2</size>
              </box>
            </geometry>
          </collision>
          <collision name="left_side">
            <pose frame="">0.45 -0.195 0.6 0.0 0.0 0.0</pose>
            <geometry>
              <box>
                <size>0.02 0.4 1.2</size>
              </box>
            </geometry>
          </collision>
          <collision name="right_side">
            <pose frame="">-0.45 -0.195 0.6 0.0 0.0 0.0</pose>
            <geometry>
              <box>
                <size>0.02 0.4 1.2</size>
              </box>
            </geometry>
          </collision>
          <collision name="bottom">
            <pose frame="">0.0 -0.195 0.03 0.0 0.0 0.0</pose>
            <geometry>
              <box>
                <size>0.88 0.4 0.06</size>
              </box>
            </geometry>
          </collision>
          <collision name="top">
            <pose frame="">0.0 -0.195 1.19 0.0 0.0 0.0</pose>
            <geometry>
              <box>
                <size>0.88 0.4 0.02</size>
              </box>
            </geometry>
          </collision>
          <collision name="low_shelf">
            <pose frame="">0.0 -0.195 0.43 0.0 0.0 0.0</pose>
            <geometry>
              <box>
                <size>0.88 0.4 0.02</size>
              </box>
            </geometry>
          </collision>
          <collision name="high_shelf">
            <pose frame="">0.0 -0.195 0.8 0.0 0.0 0.0</pose>
            <geometry>
              <box>
                <size>0.88 0.4 0.02</size>
              </box>
            </geometry>
          </collision>
        </link>
        <static>1</static>
      </model>
    </sdf>
    


# Beer can


```python
url = 'https://bitbucket.org/osrf/gazebo_models/raw/9533d55593096e7ebdfb539e99d2bf9cb1bff347/beer/model.sdf'
obj = parse_xml(download_sdf(url, 'beer'), type='sdf')
```

    <?xml version="1.0" ?>
    <sdf version="1.5">
      <model name="beer">
        <link name="link">
          <pose>0 0 0.115 0 0 0</pose>
          <inertial>
            <mass>0.390</mass>
            <inertia>
              <ixx>0.00058</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>0.00058</iyy>
              <iyz>0</iyz>
              <izz>0.00019</izz>
            </inertia>
          </inertial>
          <collision name="collision">
            <geometry>
              <cylinder>
                <radius>0.055000</radius>
                <length>0.230000</length>
              </cylinder>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <cylinder>
                <radius>0.055000</radius>
                <length>0.230000</length>
              </cylinder>
            </geometry>
            <material>
              <script>
                <uri>model://beer/materials/scripts</uri>
                <uri>model://beer/materials/textures</uri>
                <name>Beer/Diffuse</name>
              </script>
            </material>
          </visual>
        </link>
      </model>
    </sdf>



```python
print(obj)
```

    <sdf version="1.5">
      <model name="beer">
        <link name="link">
          <pose frame="">0.0 0.0 0.115 0.0 0.0 0.0</pose>
          <inertial>
            <mass>0.39</mass>
            <inertia>
              <ixz>0.0</ixz>
              <iyz>0.0</iyz>
              <iyy>0.00058</iyy>
              <izz>0.00019</izz>
              <ixx>0.00058</ixx>
              <ixy>0.0</ixy>
            </inertia>
            <pose frame="">0 0 0 0 0 0</pose>
          </inertial>
          <visual name="visual">
            <geometry>
              <cylinder>
                <length>0.23</length>
                <radius>0.055</radius>
              </cylinder>
            </geometry>
            <material>
              <script>
                <name>Beer/Diffuse</name>
                <uri>model://beer/materials/scripts</uri>
                <uri>model://beer/materials/textures</uri>
              </script>
            </material>
          </visual>
          <collision name="collision">
            <geometry>
              <cylinder>
                <length>0.23</length>
                <radius>0.055</radius>
              </cylinder>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>
    

