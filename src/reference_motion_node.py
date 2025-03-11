import os
import json
import rospy
from std_msgs.msg import Float64MultiArray

def load_motion_files(dir: str) -> dict:
    motion_files = {}
    for f in os.listdir(dir):
        with open(os.path.join(dir, f), "r") as file:
            raw_data = json.load(file)
            motion_files[f] = raw_data[0]

    # sort alphabetically by key
    motion_files = dict(sorted(motion_files.items()))

    return motion_files

def publish_motion(pub: rospy.Publisher, motion: list):
    cur_item = 0
    rate = rospy.Rate(50)
    print("[INFO] Publishing motion...")
    while cur_item < len(motion):
        if rospy.is_shutdown():
            break
        msg = Float64MultiArray(data=motion[cur_item])
        pub.publish(msg)
        cur_item += 1
        rate.sleep()
    msg = Float64MultiArray(data=[0.0]*16)
    pub.publish(msg)
    print("[INFO] Finished publishing!")
    

def main():
    # Initialize the node
    rospy.init_node("reference_motion_node", anonymous=True)
    # Create a publisher
    pub = rospy.Publisher("reference_motion", Float64MultiArray, queue_size=1)

    cur_dir = os.path.dirname(os.path.realpath(__file__))
    motion_dir = os.path.join(cur_dir, "../motion_data")

    motion_files = load_motion_files(motion_dir)

    while not rospy.is_shutdown():
        print("\nSelect an option:")
        print("    - list:              List all available motion files")
        print("    - pub MOTION_FILE:   Publish the selected motion file")
        print("    - exit:              Exit node")
        usr_input = input("> ")
        if usr_input == "list":
            for f in motion_files.keys():
                l = len(motion_files[f])
                print("{} - [{}]".format(f, l))
        elif usr_input.startswith("pub"):
            usr_motion = usr_input.split(" ")[1]
            if usr_motion not in motion_files.keys():
                print("[WARN] Invalid motion file: {}".format(usr_motion))
                continue
            print("[INFO] Trying to publish motion {}...".format(usr_motion))
            publish_motion(pub, motion_files[usr_motion])
        elif usr_input == "exit":
            break
        else:
            print("[WARN] Invalid options: {}".format(usr_input))
            
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass