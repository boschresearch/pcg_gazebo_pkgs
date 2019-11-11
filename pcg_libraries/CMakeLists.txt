cmake_minimum_required(VERSION 2.8.3)
project(pcg_libraries)

find_package(catkin REQUIRED)
    
set(SETUP_PYTHON_REQUIREMENTS ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/_create_env.py)

# Install missing Python dependencies
execute_process(
    COMMAND ${SETUP_PYTHON_REQUIREMENTS}
)

catkin_python_setup()

catkin_package()

install(
    FILES requirements.txt
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/get_list_of_joints                               
                               scripts/set_random_joint_positions
                               scripts/simulation_timer
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(rosunit REQUIRED)

    foreach(T
        test/test_assets_manager.py        
        # test/test_collision_checker.py
        test/test_engine_and_constraint_managers.py
        test/test_inertia.py                    
        test/test_jinja_sdf_file_generation.py  
        test/test_load_yaml.py
        # test/test_mesh_property.py  
        test/test_model_factory.py  
        test/test_model_group_generator.py
        test/test_model_group.py
        test/test_parse_sdf.py
        test/test_parser_urdf.py
        test/test_pose.py           
        test/test_resolve_paths.py
        test/test_sdf.py                    
        test/test_simulation_objects.py
        test/test_world_generator.py
        )
        catkin_add_nosetests(${T})
    endforeach()
endif()