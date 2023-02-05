import rospy
import csv
from std_msgs.msg import String
import os
import logging
from enum import Enum
import tf
from std_msgs.msg import Bool
import shutil

# Define the states of postit detection
# 1 = first run through and search
# 2 = go to postits

def callback_found_postit(found: bool) -> None:
    # Wait for the found topic to publish something
    # and check if coord is the same like senden (Ungefähr) 
    # mark in the csv as found as x in front of line 
    try:
        (trans,rot) = tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.loginfo('Failed %s', e)
        return

    curr_pos = [round(trans[0],2), round(trans[1],2), round(trans[2],2)] # last one is height
    rospy.loginfo(f'Logging current pos{curr_pos}')
    master_logic(curr_pos)


def master_logic(curr_pos: list) -> None:
    # Initialize the node
    rospy.init_node('coord_master_logic', anonymous=True)
    
    # Define the publisher
    pub = rospy.Publisher('master_logic_topic', String, queue_size=10)
    
    # Define the rate at which to publish the messages
    rate = rospy.Rate(10) # 10 Hz

    # Open the CSV file
    path = os.path.expanduser('~/Documents/position_data.csv')
    
    with open(path, 'r+') as csvfile, open('temp.csv', 'w') as temp:
        # Create a CSV reader
        reader = csv.reader(csvfile)
    
    
        writer = csv.writer(temp)
        
        #data = list(reader)
        first = True
        # Loop through each line in the CSV file
        
        for idx, line in enumerate(reader):
            if line[0] == '"':
                line[0] = ''
                line[-1] = ''
            # Convert the line to a string
            line_str = ','.join(line)
            l = ','.join(map(str, line))
            
            
            if (first):
                first = False
                print(f'read line: {line_str}')
                if line_str[0] == 'x':
                    rospy.loginfo(f'Tag nr {line_str[0]} already found - skipping')
                    writer.writerow([l])
                    first = True
                    continue
                
                curr_line = [float(x) for x in list(line_str.split(',')[1:])]
                rospy.loginfo(f'comparing positions inc_currentline:{curr_line}| currentpos{curr_pos}')
                if (compare_arrays(curr_line, curr_pos)):
                    rospy.loginfo(f'comparison true -> publishing postit {line_str[0]}')
                    rospy.loginfo(f'index:{idx}')
                    
                    rospy.loginfo(f'{l}')
                    rospy.loginfo(first)

                    writer.writerow([('x,' + l)])
                    pub.publish(line_str[0])
                else:
                    rospy.loginfo(f'{l}')

                    writer.writerow([l])
                    
                    # alter csv line
                    #writer.writerow('x' + line_str)
                    
                    # Publish the string as a message to the topic
                    
            else:
                rospy.loginfo('write rest')
                writer.writerow([l])
                
            # Replace the original file with the temporary file
        rospy.loginfo("SWAP")
        shutil.move('temp.csv', path)
        # only send one next postit
        return
                

            
def compare_arrays(arr1, arr2):
    if len(arr1) != len(arr2):
        return False
    for i in range(len(arr1)):
        if abs(arr1[i] - arr2[i]) > 5:
            return False
    return True

def callback_search_postit(found: bool) -> None:
    
    # get coordinates of rosbulla
    try:
        (trans,rot) = tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.loginfo('Failed %s', e)
        return

    curr_pos = [round(trans[0],2), round(trans[1],2), round(trans[2],2)] # last one is height

    
    pos_saved = False
    
    path = os.path.expanduser('~/Documents/position_data.csv')
    with open(path, 'r+') as csv_file:
        reader = csv.reader(csv_file)
        
        for row in reader:
            rospy.loginfo(row)
            
            line_str = ','.join(row)
            curr_row = [float(x) for x in list(line_str.split(',')[1:])]
            # check if already in csv
            if compare_arrays(curr_row, curr_pos):
                rospy.loginfo(f"Comparing csv_row:{curr_row} with bulla_pos {curr_pos}")
                pos_saved = True
        
        # check if in range

    
    
    # save to csv
    
        if not pos_saved:
            writer = csv.writer(csv_file)
            #line_str = ','.join(curr_pos)
            rospy.loginfo(curr_pos)
            writer.writerow(curr_pos)

# Call the node's main function if the node is run as the main script
if __name__ == '__main__':
    try:
        rospy.init_node("coord_master_logic", anonymous=True)
        # Preqreuirement the pixyfinder needs to be started first
        
        
        # WTF is this shit and why is it working
        global tf_listener
        tf_listener = tf.TransformListener()
        
        mode = rospy.get_param("~mode")
        rospy.loginfo(f"Rosbulla running on Mode {mode}")
        if mode == 1:
            rospy.Subscriber("/found", Bool, callback_search_postit)
        else:
            rospy.Subscriber("/found", Bool, callback_found_postit)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
