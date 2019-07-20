
# Simulation stage conditions


```python
# If there is a Gazebo instance running, you can spawn the box into the simulation
from pcg_gazebo.task_manager import Server
# First create a simulation server
server = Server()
# Create a simulation manager named default
server.create_simulation('default')
simulation = server.get_simulation('default')
# Run an instance of the empty.world scenario
# This is equivalent to run
#      roslaunch gazebo_ros empty_world.launch
# with all default parameters
simulation.create_gazebo_empty_world_task(paused=True, simulation_timeout=10)
```


```python
from pcg_gazebo.simulation import create_object
from pcg_gazebo.generators import WorldGenerator

def model_exists(name):
    print('Testing if model {} exists'.format(name))
    gazebo_proxy = simulation.get_gazebo_proxy()    
    success = name in gazebo_proxy.get_model_names()
    print('Model {} exists? {}'.format(name, success))
    return success

def spawn_model():
    print('Spawning box into Gazebo')
    obj = create_object('box')
    obj.size = [0.8, 0.7, 0.9]
    obj.add_inertial(30)
    
    gazebo_proxy = simulation.get_gazebo_proxy()    
    generator = WorldGenerator(gazebo_proxy=gazebo_proxy)
    for x in [-5, 0, 5]:
        for y in [-5, 0, 5]:
            generator.spawn_model(
                    model=obj, 
                    robot_namespace='box_{}_{}'.format(x, y),
                    pos=[x, y, 10])
    print('Spawning box finished')
    return True

def unpause():
    print('Unpause simulation')
    gazebo_proxy = simulation.get_gazebo_proxy()    
    gazebo_proxy.unpause()
    return True
    
```


```python
# Adding a stage starting condition to the gazebo stage to check 
# if roscore is running
simulation.add_stage_start_condition('gazebo', simulation.is_roscore_running)
# Add stage end condition to be sure Gazebo is running
simulation.add_stage_end_condition('gazebo', simulation.is_gazebo_running)
# Add pre-stage function to spawn the models
simulation.add_post_stage_fcn('gazebo', spawn_model)

# Adding final empty stage to check if model was created
simulation.add_stage('post-init')
# Adding stage end condition where the model must exist in 
# Gazebo
simulation.add_stage_start_condition('post-init', lambda: model_exists('box_0_0'))
simulation.add_post_stage_fcn('post-init', unpause)
```


```python
for tag in simulation.stages:
    print('Stage: {}'.format(tag))
    for task in simulation.get_tasks_from_stage(tag):
        print('  - {}'.format(task))
```


```python
simulation.run_all_tasks()
```


```python
simulation.wait()
```
