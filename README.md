# README

This is an experimental image to read and use the light sensor that are used in the ETHZ Autolab.

To launch calibration procedure, use : 

`dts duckiebot demo --duckiebot_name WATCHTOWER_NAME --demo_name light_sensor_calibration --package_name light_sensor_calibration --image duckietown/dt-light-sensor:update_DTROS-arm32v7 --debug` 

Then to launch the light sensor node, use :

`dts duckiebot demo --duckiebot_name WATCHTOWER_NAME --demo_name light_sensor --package_name light_sensor --image duckietown/dt-light-sensor:update_DTROS-arm32v7` 

To plug in the RGB sensor have a look at: 
