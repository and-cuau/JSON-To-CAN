from perception_comm.msg import CAN
from std_msgs.msg import Header, Int16MultiArray, Float32MultiArray, Int64MultiArray
import json
import numpy as np
import os
import can
import cantools
from rclpy.node import Node
import rclpy
import time
from .message import Message
# from .package import globaldata
# from .package import globaldata
from multiprocessing import Process, Value, Array
from .can_send_main import SendCANMessage as sCan
import multiprocessing
from multiprocessing import Manager
from .scoring import AVstate
#from gps_msgs.msg  import GPSFix
from threading import Thread, Lock
import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber
import copy
from rclpy.executors import MultiThreadedExecutor
# from novatel_gps_msgs.msg import NovatelHeading2


# manager=Manager()
# can_msg_DICT=manager.dict()
#+++++
# json_dict=json.load(open("/home/tsz/Documents/tszlearning/package_ws/src/can_sending_pkg/can_sending_pkg/CAN_format_default.json"))

#+++++
# global_auto_stat_val = multiprocessing.Value('i', 1)
# steer_ctrl_val = multiprocessing.Value('i', 0)
# fric_brake_val = multiprocessing.Value('i', 2)
# prop_ctrl_val = multiprocessing.Value('i', 0)
# avState = AVstate(can_msg_DICT)

# p5 = multiprocessing.Process(target=avState.test_send)
# p5.start()
# p5.join()





# print(data['Roadstate'])




# data['new_key'] = 'new_value'

# Write back to the file (optional)
# with open('CAN_format_default.json', 'w') as file:
#    json.dump(data, file)







class json2can(Node):
    index = 0
    def __init__(self):
        super().__init__('sub_can_processing')
        self.can_msg_DICT={}

        # self.subscription = self.create_subscription(
        #     CAN,
        #     'perception_msg',
        #     self.listener_callback,
        #     10)

     
        # #++++++ Set up for listening two topic +++++++++++++++++++++++
        # self.perce_msg=message_filters.Subscriber(self, CAN, "/perception_msg") # sub to perception topic
        # self.AVState_msg=message_filters.Subscriber(self, Int64MultiArray, "/scoring_can") #The Subscriber for your code.
        # # self.perce_msg=message_filters.Subscriber(self, NovatelHeading2, "/heading2") #The first topic I want to subscribe
        # # self.AVState_msg=message_filters.Subscriber(self, GPSFix, "/gps") #The Second topic I want to subscribe

        

        # queue_size = 100
        # ts_tolerance = 1

        # ats = ApproximateTimeSynchronizer([self.AVState_msg, self.perce_msg], queue_size, ts_tolerance)
        # ats.registerCallback(self.SC_callback) # triggers self.SC_callback when messages succesfully synced.
        # #+++++++++++++++++++++++++++
        

# Path to the JSON file that stores CAN format data
path = '/home/andre_cuau/ros2_ws/src/sub_can_processing_single/sub_can_processing_single/CAN_format_default.json'

# Open the JSON file in read mode
with open(path, 'r') as file:
    # Read the contents of the file as a string
    json_data = file.read()

# Load the JSON data as a Python dictionary
my_dict = json.loads(json_data)

# Store the loaded dictionary in the object dict variable
self.dict = my_dict

# Initialize an empty list to store the keys from the dictionary
keys = []
key = ""

# Loop through all the keys in the dictionary and append them to 'keys' list
for k in self.dict.keys():
    keys.append(k)

print("KEY AND DICT:")

# Set the 'key' variable to the current key based on 'json2can.index'
# 'json2can.index' is an index that keeps track of the current key being processed
key = keys[json2can.index]
# Increment the index for the next key
json2can.index += 1

# If index greater than number of keys, last key has been reached
if json2can.index + 1 > len(keys):
    print(f"{key} is the last key")

# Print the current key being processed
print(key)

# Create a new dictionary with the current key and its associated value from the original dictionary
key_dict = {key: self.dict[key]}

# Print the newly created dictionary for the current key
print(key_dict)
print()

# Store the key-specific dictionary in the 'can_msg_DICT' attribute
self.can_msg_DICT = key_dict

self.flag = False


self.lock = Lock()

        #Init Can Parameters
        #The path of the ADC_SC.dbc
        self.db = cantools.database.load_file(os.path.join(os.getcwd(), 'src', 'sub_can_processing', 'sub_can_processing' , 'ADC_SC.dbc'))
        #self.db = cantools.database.load_file('PUT CURRENT WS LOCATION/src/sub_can_processing/ADC_SC.dbc')
        self.interval=0.1

        #Depends on how you init the CAN BUS on Intel computer. Ask Javed.
        #self.can_bus = can.interface.Bus('can0', bustype='socketcan')
        
         


    def test_send(self): 
        while True:
            with self.lock:
                scm=sCan()#Init CAN
                # print(self.can_msg_DICT)
                # print(self.can_msg_DICT['VehicleLocation']['VehicleLatitude'])
                start = time.time()
            
                #Encode and send the data into SC
                scm.encode_send(self.db, self.can_bus,self.can_msg_DICT) 
               
                # print(can_msg_dict['V2XIntersection_MAP'])
                # print(self.can_msg_DICT['VehicleLocation']['VehicleLatitude'])
                end = time.time()
                time.sleep(abs(self.interval-(end-start)))
        
    def start(self):
        # self.p5=multiprocessing.Process(target=self.test_send, args=())
        # self.p5.start()
        # self.p5.join()
        self.thread=Thread(target=self.test_send, args=())
        self.thread.start()
        # self.test_send()


    def SC_callback(self,AVState_msg, perce_msg):
        
        #AVState_msg: the AVState data message subcribe from '/scoring_can'



        #Init the SendCan cladd
        # scm=sCan()
        # self.test_send()
        # print(self.can_msg_DICT)

        # Post-Processing data
        can_msg = Message.from_ros(perce_msg)

        can_dict=can_msg.to_can()

        # Set AVState fo++++++++
        
        can_dict['AVState']['GlobalAutonomyStatus']=AVState_msg[0]
        can_dict['AVState']['SteeringCtrlActive']=AVState_msg[1]
        can_dict['AVState']['FrictionBrakeCtrlActive']=AVState_msg[2]
        can_dict['AVState']['PropulsionCtrlActive']=AVState_msg[3]
        V2XIntersection_SPAT={'IntersectionID':0,'SignalGroup':0,'SPAT_Timestamp':0,'SecondsUntilChangeMinimum':0,'SecondsUntilChangeMaximum':0,'MovementEventState':0,'Rolling_Count':0,'SPAT_Validity':0}
        V2XIntersection_MAP={'IntersectionID':0,'IngressLaneID':0,'PlannedEgressLaneID':0,'PlannedManeuver':0,'SignalGroup':0,'Rolling_Count':0,'MAP_Validity':0}
        can_dict['V2XIntersection_SPAT']=V2XIntersection_SPAT
        can_dict['V2XIntersection_MAP']=V2XIntersection_MAP

        #This will modify the global data and update the data in thread.
        self.can_msg_DICT=can_dict

        #+++++++++++++++++++++++

        # self.can_msg_DICT.pop('AVState')
        # print(self.can_msg_DICT['VehicleLocation']['VehicleLatitude'])
        # self.can_msg_DICT=can_msg.to_can()

        # If the thread does not start, start it, otherwise pass.
        if self.flag!=True:
            self.flag=True
            self.start()

        # else:
        #     self.p5.close()
        #     self.start()
        # print(can_msg.)

        #globaldata.global_Message=copy.deepcopy(can_msg)
        #globaldata.global_scoring_dict=copy.deepcopy(globaldata.global_Message.to_can())
        #print(ppp.global_var.value)

        # can_msg_DICT=copy.deepcopy(manager.dict(can_msg.to_can()))

        # print(can_msg_DICT['Object_TrackA'])
        # can_msg_dict['RoadState']['DistToLLnEdge']=0
        # can_msg_dict['AVState']['GlobalAutonomyStatus']=2

        # print(can_msg_dict['Object_TrackA'])
        # can_msg_dict['Object_TrackB']['Object_Relative_Orientation']=0
        # can_msg_dict['AVState']['GlobalAutonomyStatus']=ppp.global_auto_stat_val.value #Copy AVState values into dict
        # can_msg_dict['AVState']['SteeringCtrlActive']=copy.deepcopy(ppp.steer_ctrl_val.value) #Copy AVState values into dict
        # can_msg_dict['AVState']['FrictionBrakeCtrlActive']=copy.deepcopy(ppp.fric_brake_val.value) #Copy AVState values into dict
        # can_msg_dict['AVState']['PropulsionCtrlActive']=copy.deepcopy(ppp.prop_ctrl_val.value) #Copy AVState values into dict
        
        # print(can_msg_dict('AVState'))#Copy AVState values into dict
        
        # Data sending
        # can_msg_avstate={}

        # can_msg_avstate['VehicleLocation']=can_msg_dict['VehicleLocation']
        # can_msg_avstate['VehicleLocation']['VehicleLatitude']=0.11
        # can_msg_avstate['VehicleLocation']['VehicleLongitude']=0.11

        # print(can_msg_avstate)
        # start = time.time()
        # scm.encode_send(self.db, self.can_bus,can_msg_dict)
        # # print(can_msg_dict['V2XIntersection_MAP'])
        # end = time.time()
        # time.sleep(abs(self.interval-(end-start)))




        """
        To access value in can_msg, using can_msg.CAN_KEY or can_msg.CAN_KEY.CAN_SUBKEY

        For instance:
            
            [option 1]
            can_msg.RoadState

            This will return a Message Class object of RoadState

            [option 2]
            can_msg.RoadState.Active_Lane_Number

            This will return the value of Active_Lane_Number

        
        """
        # print(can_msg.RoadState)

        



def main(args=None):
    rclpy.init(args=args)
    # globaldata.init()

 
    
    node = json2can()
    num = 0

    
    #Extract values of "VehicleLocation" and "Objects"
    # vehicle_location_values = list(my_dict["VehicleLocation"].values())
    # objects_values = list(my_dict["Objects"].values())


   
    # road_state_dict = {key: my_dict["RoadState"][key] for key in my_dict["RoadState"]}
    
    for i in range(16):
        node.__init__()
    
    #    node.test_send()
    #rclpy.spin(node)
    node.destroy_node()
    

    # executor = MultiThreadedExecutor()
    # executor.add_node(node) # managing this node 
    # executor.spin()
    # rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
