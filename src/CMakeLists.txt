# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS ${app_sources}
                    REQUIRES esp_video esp_cam_sensor esp_cam_sensor_imx esp_driver_jpeg esp_http_server)
