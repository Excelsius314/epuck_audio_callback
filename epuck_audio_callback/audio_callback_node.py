import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Float32
from std_msgs.msg import Bool

from epuck_driver_interfaces.msg import RecordActivation
from epuck_driver_interfaces.msg import RecordResult

import numpy as np

class AudioCallBack(Node):

    def __init__(self):
        super().__init__('epuck_audio_callback')
        
        self.robot_ids = ["epuck"] #  , "rob_1", "rob_2"]
        self.num_microphones = 4
        self.microphone_buffers = {id:{k:[] for k in range(self.num_microphones)} for id in self.robot_ids}
        self.microphone_data_subscribers = [[self.create_subscription(Float32, 
                                                                      "{}/microphone_{}".format(id, k), 
                                                                      self.create_microphone_data_callback(id, k), 5) for k in range(self.num_microphones)] for id in self.robot_ids]

        self.record_activation_subscriber = self.create_subscription(RecordActivation, "audio_collection_activation", self.activation_callback, 1)
        self.record_finisher_publisher = self.create_publisher(RecordResult, "record_result", 1)
        self.record = False

        self.timer = None

        self.average_over_robot = True
        
    def activation_callback(self, msg):

        print("Activating recording of data")

        self.clear_buffers()
        self.average_over_robot = msg.average_over_robot
        self.record = True
        self.timer = self.create_timer(msg.recording_time, self.record_done_callback)
    
    def create_microphone_data_callback(self, robot_id, mic_id):
         
        def callback(msg):
            if self.record:
                self.microphone_buffers[robot_id][mic_id].append(msg.data)
        
        return callback
    
    def clear_buffers(self):
         
        for id in self.robot_ids:
            for k in range(self.num_microphones):
                self.microphone_buffers[id][k].clear()
    
    def get_formated_data_from_buffer(self):

        robot_audio_data = []
        min_len = 10*4

        for i, id in enumerate(self.robot_ids):

            robot_audio_data.append([])
            for k in range(self.num_microphones):
                
                mic_data = self.microphone_buffers[id][k]
                if len(mic_data) == 0:
                    continue

                robot_audio_data[i].append(mic_data)
                min_len = min(min_len, len(mic_data))
        
        robot_audio_data = np.array(robot_audio_data[:][:][ :min_len])
        print("Recorded Data:")
        print(robot_audio_data)

        avg_axis = (1, 2) if self.average_over_robot else 2
        averaged_result = np.average(robot_audio_data, axis=avg_axis)

        print("Averaged Result {}; average over robot: {}".format(averaged_result, self.average_over_robot))
        return averaged_result

    def record_done_callback(self):

        print("Recording done")
        self.record = False
        self.timer.cancel()
        self.timer.destroy()
        result_msg = RecordResult()
        result_msg.averaged_audio_levels = list(self.get_formated_data_from_buffer().flatten())
        self.record_finisher_publisher.publish(result_msg)


def main():

    rclpy.init()
    aud_node = AudioCallBack()
    rclpy.spin(aud_node)

    aud_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
