import rclpy
import re
import json
from rclpy.node import Node
import socket
import threading
from std_msgs.msg import String
from mv2_com_interfaces.srv._move_pose import MovePose
from geometry_msgs.msg import Pose,Point,Quaternion



# TODO RETURN INVALID COMMAND TO CLIENT
# TODO SET PORT BY LAUNCH FILE
# HOST = '127.0.0.1'
# HOST = '192.168.1.141'
HOST=socket.gethostbyname(socket.gethostname())
PORT = 65432
ADDR = (HOST, PORT)
HEADER = 64
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"



class MoveCommandClient(Node):

    def __init__(self):
        super().__init__('move_commands_client')
        self.cli = self.create_client(MovePose, 'plan_move_robot')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.get_logger().info("STARTING MOVE COMMANDS CLIENT ON %s" % HOST)
        self.req = MovePose.Request()



        # create server
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(ADDR)
        self.server.listen()
        # run server in loop
        while True:
            conn, addr = self.server.accept()
            thread = threading.Thread(target=self.handle_client, args=(conn, addr))
            thread.start()
            # self.get_logger().info('[ACTIVE CONNECTIONS] "%s"' % str(threading.activeCount() - 1))
        


    def shutdown_callback(self):
        self.get_logger().info('Shutting down...')
        print("closing server")
        self.server.close()


        # Add your shutdown logic here
    def handle_client(self, conn, addr):
        # self.get_logger().info('[NEW CONNECTION] "%s" connected.' % str(addr))
        connected = True
        while connected:
            msg_length = conn.recv(HEADER).decode(FORMAT) #first we receive the header file to tell us how long the message is
            if msg_length:
                msg_length = int(msg_length)
                msg = conn.recv(msg_length).decode(FORMAT)  #then we use the received message length to parse te actual message
                if msg == DISCONNECT_MESSAGE:
                    print(f" Successfully Disconnected from [{addr}]")

                    connected = False

                else:
                    # self.get_logger().info('[MSG RECEIVED] "%s" ' % str(msg))
                    print(f"[{addr}] {msg}")
                    # self.parse_messag`e_and_publish(msg)
                    self.parse_message_and_execute(msg)
                    conn.send("Msg received".encode(FORMAT))
        conn.close()
        self.get_logger().info('[DISCONNECTED]')

    def parse_message_and_execute(self,message):
        parsed_message = json.loads(message)
        for singlemsg in parsed_message["commands"]:
            print("received:")
            print(singlemsg)
            print(singlemsg["command"])

            if (singlemsg["commandtype"]=="moveit_move"):

                parsedcmd = self.parsemove(singlemsg["command"])
                self.get_logger().info('[SENDING MOVEIT MOVE CMD] "%s" ' % self.format_2d_array(parsedcmd))
                # self.cli.call(self.req)
                pose = Pose()
                position_values=parsedcmd[0]
                orientation_values=parsedcmd[1]
                position = Point()
                position.x, position.y, position.z = position_values
                orientation = Quaternion()
                orientation.x, orientation.y, orientation.z, orientation.w = orientation_values
                pose.position = position
                pose.orientation = orientation
                self.req.pose=pose
                self.future= self.cli.call_async(self.req)
                rclpy.spin_until_future_complete(self,self.future)
                self.get_logger().info('[RECEIVED RESPONSE ]  %r' % self.future.result().success)





            # self.publisher_.publish(msg_publish)
    def parsemove(self, input):
        print(input)

        # Use regular expression to extract the contents within parentheses
        matches = re.findall(r'\((.*?)\)', input)
        print(matches)

        # Split each match based on commas and convert them to floating-point numbers
        array_2d = [[float(num) for num in match.split(',')] for match in matches]        
        print(array_2d)

        return array_2d
        
    def format_2d_array(self,array_2d):
        formatted_string = "["
        for sublist in array_2d:
            formatted_string += "["
            for item in sublist:
                formatted_string += "%.2f, " % item  # %.2f formats floating-point numbers to two decimal places
            formatted_string = formatted_string[:-4]  # Remove the trailing comma and space
            formatted_string += "], "
        formatted_string = formatted_string[:-4]  # Remove the trailing comma and space
        formatted_string += "]"
        return formatted_string
    
    def parse_message_and_publish(self,message):
        parsed_message = json.loads(message)
        for singlemsg in parsed_message["commands"]:


            msg_publish=String()
            dumpedmsg = json.dumps(singlemsg)

            msg_publish.data=dumpedmsg
            self.get_logger().info('[SENDING COMMAND] "%s" ' % str(dumpedmsg))
            self.publisher_.publish(msg_publish)





def main(args=None):
    rclpy.init(args=args)
    
    minimal_publisher = MoveCommandClient()
    def shutdown_callback():
        print("Shutting down...")
        # Call node function directly
        minimal_publisher.shutdown_callback()

    rclpy.on_shutdown(shutdown_callback)
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
    rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)


    


if __name__ == '__main__':
    main()
