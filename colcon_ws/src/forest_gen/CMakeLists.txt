cmake_minimum_required(VERSION 2.8.3)
project(forest_gen)

# find_package(catkin_simple REQUIRED)
# catkin_simple(ALL_DEPS_REQUIRED)

##########
# EXPORT #
##########
# cs_install()
# cs_export()


find_package(ament_cmake REQUIRED)

# ament_package()

install(DIRECTORY 
	models_low_res
	models_high_res
          DESTINATION share/${PROJECT_NAME})
