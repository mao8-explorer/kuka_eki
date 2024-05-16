from EkiConfig.ekiclients import EkiMotionClient
from EkiConfig.krl_struct import Axis,Pos
import signal
import sys
import math


def sigint_handler(sig, frame):
    global eki_motion_client
    print("SIGINT received, closing client...")
    eki_motion_client.stop()
    eki_motion_client.close()
    eki_motion_client = None  # Release reference to allow garbage collection
    sys.exit(0)

def sigterm_handler(sig, frame):
    global eki_motion_client
    print("SIGTERM received, closing client...")
    eki_motion_client.stop()
    eki_motion_client.close()
    eki_motion_client = None  # Release reference to allow garbage collection
    sys.exit(0)

# 注册SIGINT和SIGTERM信号处理函数
signal.signal(signal.SIGINT, sigint_handler)
signal.signal(signal.SIGTERM, sigterm_handler)


if __name__ == "__main__":

    eki_motion_client = EkiMotionClient("172.31.1.147", 54605)
    eki_motion_client.connect()
    
    
    max_buffersize_limit = 10 # by default, limit command buffer to 5 (size of advance run in KRL)
    target_axis = Axis(a1=-90, a2=-90.0, a3=90, a4=0.0, a5=0.0, a6=0.0, e1 = 20)
    target_pos = Pos(10.0, 1765.0, 1910.0, -40.0, 90.0, -93.0, e1 = 20)

    eki_motion_client.ptp(target_pos, 50)

    # Define circle parameters
    circle_radius = 500  # radius of the circle
    circle_center = (0.0, 1765.0)  # center coordinates of the circle
    num_points = 4  # number of points to divide the circle
    angle_increment = 2 * math.pi / num_points  # angle increment for each point

    # Initialize angle and initial e1_value
    current_angle = 0
    e1_value = 10  # initial value for e1 axis

    # e1 parameters
    e1_step = 10.0
    e1_direction = 1.0
    e1_lower_limit = 20.0
    e1_upper_limit = 50.0

    try:
        while eki_motion_client._is_running:
            cur_buffersize = eki_motion_client.ReadBufferSize()
            if cur_buffersize < max_buffersize_limit: 
                print("current buffer size: ", cur_buffersize)
                # Calculate target position based on current angle
                x = circle_center[0] + circle_radius * math.cos(current_angle)
                y = circle_center[1] + circle_radius * math.sin(current_angle)

                # Create Pos object with constant z-value, calculated x, y values, and e1 axis value
                target_pos = Pos(x,y, 1910.0, -40.0, 90.0, -93.0, e1=e1_value)
                # Perform point-to-point motion to target position
                eki_motion_client.ptp(target_pos, 50)

                # Increment angle for next iteration
                current_angle += angle_increment
                if current_angle >= 2 * math.pi:
                    current_angle -= 2 * math.pi

                if(e1_value > e1_upper_limit):
                    e1_direction = -1
                if(e1_value < e1_lower_limit):
                    e1_direction = 1
    
                e1_value +=  e1_step * e1_direction

    except Exception as e:
        print(f"An error occurred: {e}")
        eki_motion_client.close()

    finally:
        # Close the motion client
        eki_motion_client.close()

