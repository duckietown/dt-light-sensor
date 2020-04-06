#!/usr/bin/env python
import rospy
import time
import RPi.GPIO as GPIO
import Adafruit_TCS34725
import yaml
import os.path
from duckietown_msgs.msg import LightSensor
from duckietown import DTROS


class LightSensorNode(DTROS):
    """Node handling the measurment of light.
        measures the light in the room and publish it.

        Publishers:
           ~sensor_data (:obj:`LightSensor`): Publishes measured intesitiy of red, green and blue, the calculated lux and the real_lux value that takes 
           callibration into account. It publishes also the temperature of the light. 
    """
    
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LightSensorNode, self).__init__(node_name=node_name)

        self.veh_name = rospy.get_namespace().strip("/")
        
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(18, GPIO.OUT)
        GPIO.output(18, GPIO.LOW)

        # Add the node parameters to the parameters dictionary and load their default values
        self.parameters['~default_mult'] = None
        self.parameters['~default_offset']=None
        self.parameters['~integration_time']=None
        self.parameters['~integration_gain']=None

        self.updateParameters()

        #define parameter
        self.mult = 0
        self.offset = 0
        self.default_mult=self.parameters['~default_mult']
        self.default_offset= self.parameters['~default_offset']
        self.integration_time = self.parameters['~integration_time']
        self.integration_gain = self.parameters['~integration_gain']
        
        #convert integration time and gain if form that it has to be
        self.convert_integration()
        
        #set integration time and value
        self.tcs = Adafruit_TCS34725.TCS34725(
            integration_time=self.integration_time,gain=self.integration_gain)

        #define file path
        self.filepath = '/data/config/calibrations/light-sensor/' + self.veh_name + ".yaml"

        #set parametervalue from callibration
        self.readParamFromFile()

        # ROS-Publications
        self.msg_light_sensor = LightSensor()
        self.sensor_pub = self.publisher('~sensor_data', LightSensor, queue_size=1)
        while not rospy.is_shutdown():
            self.get_lux()
            
        
    def get_lux(self):
        # Read R, G, B, C color data from the sensor.
        r, g, b, c = self.tcs.get_raw_data()
        # Calulate color temp
        #temp = Adafruit_TCS34725.calculate_color_temperature(r, g, b)
        # Calculate lux and multiply it with gain
        lux = Adafruit_TCS34725.calculate_lux(r, g, b)
        real_lux = self.mult * lux + self.offset

        # Publish to topic
        #header
        self.msg_light_sensor.header.stamp = rospy.Time.now()
        self.msg_light_sensor.header.frame_id = rospy.get_namespace()[1:-1] 

        self.msg_light_sensor.r = r
        self.msg_light_sensor.g = g
        self.msg_light_sensor.b = b
        self.msg_light_sensor.c = c
        self.msg_light_sensor.real_lux = real_lux
        self.msg_light_sensor.lux = lux
        #self.msg_light_sensor.temp = temp
        self.sensor_pub.publish(self.msg_light_sensor)
        
        #print message
        rospy.loginfo(self.msg_light_sensor)

    

    def readParamFromFile(self):
        # Use default.yaml if file doesn't exsit
        if not os.path.isfile(self.filepath):
            rospy.logwarn("[%s] %s does not exist. Using default values" % (self.node_name, self.filepath))
            self.mult = self.default_mult
            self.offset = self.default_offset
            
        else:
            with open(self.filepath, 'r') as in_file:
                yaml_dict = yaml.load(in_file)
                self.mult = yaml_dict["mult"]
                self.offset = yaml_dict["offset"]
                
        rospy.loginfo("[%s] from callibration %s = %s " % (self.node_name,"mult", self.mult))
        rospy.loginfo("[%s] from callibration %s = %s " % (self.node_name,"offset", self.offset))
    
    def convert_integration(self):
        #convert integration time in the form that it has to be
        if self.integration_time==700:
            self.integration_time = Adafruit_TCS34725.TCS34725_INTEGRATIONTIME_700MS
        if self.integration_time==154:
            self.integration_time = Adafruit_TCS34725.TCS34725_INTEGRATIONTIME_154MS
        if self.integration_time==101:
            self.integration_time = Adafruit_TCS34725.TCS34725_INTEGRATIONTIME_101MS
        if self.integration_time==50:
            self.integration_time = Adafruit_TCS34725.TCS34725_INTEGRATIONTIME_50MS
        if self.integration_time==24:
            self.integration_time = Adafruit_TCS34725.TCS34725_INTEGRATIONTIME_24MS
            
        #convert integration gain in the form that it has to be
        if self.integration_gain==1:
            self.integration_gain = Adafruit_TCS34725.TCS34725_GAIN_1X
        if self.integration_gain==4:
            self.integration_gain = Adafruit_TCS34725.TCS34725_GAIN_4X
        if self.integration_gain==16:
            self.integration_gain = Adafruit_TCS34725.TCS34725_GAIN_16X
        if self.integration_gain==60:
            self.integration_gain = Adafruit_TCS34725.TCS34725_GAIN_60X



if __name__ == '__main__':
    node = LightSensorNode(node_name='light_sensor_node')
    rospy.spin()
