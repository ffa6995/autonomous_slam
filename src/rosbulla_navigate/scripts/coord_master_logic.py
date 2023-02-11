from typing import Union
import rospy
import csv
import time
from std_msgs.msg import String
import os
import logging
from enum import Enum
import tf
from std_msgs.msg import Bool
from std_msgs.msg import Int32
import shutil

# #  Define the states of postit detection
# 1 = first run through and search
# 2 = go to postits


class MasterLogico():
    def __init__(self) -> None:
        # Define the publisher
        self.pub = rospy.Publisher('/postit_num', Int32, queue_size=1)
        self.searchnext_pub = rospy.Publisher('/searchnext', Bool, queue_size=1)
        
        self.postit_count: int = 0
        self.current_published_postit: int = -1

    
    @staticmethod
    def get_next_postit_number(reader) -> Union[int, list]:
        try:
            lines = []
            line = next(reader)
            
            while (line is not None):
            
                
                line = ','.join(line)
                if line[0] == '"':
                    line[0] = ''
                    line[-1] = ''
                if line[0] != 'x':
                    lines.append([line])
                    return int(line[0]), lines
                else:
                    lines.append([line])
                    #return None, line
                line = next(reader)
            return None, lines
            
        except StopIteration:
            rospy.loginfo('Exception: StopIterationq')
            return (None, lines)


    def callback_found_postit(self, found: bool) -> None:
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
        self.master_logic(curr_pos)


    def master_logic(self, curr_pos: list) -> None:
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
                # if line has quotes remove them
                if line[0] == '"':
                    line[0] = ''
                    line[-1] = ''

                # Convert the line to a string
                line_str = ','.join(line)
                l = ','.join(map(str, line))
                

                if (first):
                    
                    print(f'read line: {line_str}')
                    if line_str[0] == 'x':
                        rospy.loginfo(f'Tag nr {line_str[2]} already found - skipping')
                        writer.writerow([l])
                        first = True
                        continue
                    
                    # added for compare external - might remove this
                    #elif line_str[0] == 'x' and self.current_published_postit == line_str[1]:
                    #    writer.writerow([l])
                    #    first = True
                    
                    curr_line = [float(x) for x in list(line_str.split(',')[1:])]
                    rospy.loginfo(f'comparing positions inc_currentline:{curr_line}| currentpos{curr_pos}')
                    if (self.compare_arrays(curr_line, curr_pos)):
                        rospy.loginfo(f'Rosbulla confirmed near postit -> publishing next postit')

                        # mark found postit
                        first = False
                        writer.writerow([('x,' + l)])

                        # Read and publish next postit
                        
                        next_postit, next_line = self.get_next_postit_number(reader)

                        if next_line is not None:
                            #writer.writerow([next_line])
                            writer.writerows(next_line)
                        if next_postit is None :
                            rospy.loginfo('no next postit')
                            continue
                        rospy.loginfo(f'publishing postit: {next_postit}')
                        self.current_published_postit = int(next_postit)
                        self.pub.publish(int(next_postit))
                    else:
                        rospy.loginfo(f'{l}')

                        writer.writerow([l])
                        
                        # alter csv line
                        #writer.writerow('x' + line_str)
                        
                        # Publish the string as a message to the topic
                        
                else:
                    writer.writerow([l])
                    
                # Replace the original file with the temporary file
            shutil.move('temp.csv', path)
            # only send one next postit
            return
                    



    @staticmethod                
    # This function compares two arrays and returns true if they are the same
    # First we check to see if the arrays are the same length
    def compare_arrays(arr1, arr2):
        if len(arr1) != len(arr2):
            return False

        # If they are the same length, check each element to see if the difference is less than 0.25m
        for i in range(len(arr1)):
            if abs(arr1[i] - arr2[i]) > 0.25:
                return False
        return True

    def callback_search_postit(self, found: bool) -> None:
        """Search all postits in first walkthrough"""
        
        # get coordinates of rosbulla
        try:
            (trans,rot) = tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo('Failed %s', e)
            return

        curr_pos = [round(trans[0],2), round(trans[1],2), round(trans[2],2)] # last one is height

        rospy.loginfo(f'current position {curr_pos}')
        pos_saved = False
        
        path = os.path.expanduser('~/Documents/position_data.csv')
        with open(path, 'r+') as csv_file:
            reader = csv.reader(csv_file)
            
            for row in reader:
                rospy.loginfo(row)
                
                line_str = ','.join(row)
                curr_row = [float(x) for x in list(line_str.split(',')[1:])]
                # check if already in csv

                #rospy.loginfo(f"Comparing csv_row:{curr_row} with bulla_pos {curr_pos}")
                if self.compare_arrays(curr_row, curr_pos):
                    rospy.loginfo("Found a postit in range of rosbulla - wont save position!")                    
                    pos_saved = True
            
            # check if in range

        # save to csv
        
            if not pos_saved:
                writer = csv.writer(csv_file)
                #line_str = ','.join(curr_pos)
                numerated_pos = curr_pos
                self.postit_count += 1
                # Publish to drive witer
                self.searchnext_pub.publish(True)
                numerated_pos.insert(0, str(self.postit_count))

                writer.writerow(numerated_pos)
                

    def publish_first_postit_number(self, pub) -> None:
        """Publish the first postit number"""
        path = os.path.expanduser('~/Documents/position_data.csv')
        with open(path, 'r') as csv_file:
            reader = csv.reader(csv_file)
            for row in reader:
                rospy.loginfo(row)
                line_str = ','.join(row)
                rospy.loginfo(f'Publishing postit: {line_str[0]}')
                if line_str[0] != "x":
                    self.current_published_postit = int(line_str[0])
                    pub.publish(int(line_str[0]))
                    break

    def mark_postit_number(self, number: int) -> None:
        """Mark a given the postit number into the csv file"""
        path = os.path.expanduser('~/Documents/position_data.csv')
        
        with open(path, 'r') as csv_file:
            reader = csv.reader(csv_file)
            
            rows = []

            for i,row in enumerate(reader):
                if row[0] == '"':                     
                    row[0] = ''                     
                    row[-1] = ''
                row = ','.join(row)
                row = row.split(',')
                rows.append(row)
                if row[0] != "x" and int(row[0]) == number:
                    rospy.loginfo(f'marking postit number {number} as found')
                    row.insert(0, 'x')

        with open(path, 'w') as csv_file:
            writer = csv.writer(csv_file)
            rospy.loginfo(rows)
            writer.writerows(rows)
 
        
    def callback_extern_found_postit(self, number) -> None:
        number = number.data
        """Callback for external found postit"""
        rospy.loginfo(f"Received external found postit {number}")
        self.mark_postit_number(number)

        if self.current_published_postit == number:
            rospy.loginfo(f"Rosbulla was on the way to a found postit {number} - publishing next postit")
            self.callback_found_postit(self.pub)



# Call the node's main function if the node is run as the main script
if __name__ == '__main__':
    try:
        rospy.init_node("coord_master_logic", anonymous=True)
        # Preqreuirement the pixyfinder needs to be started first
        
        global tf_listener
        tf_listener = tf.TransformListener()

        master = MasterLogico()
        
        
        mode = rospy.get_param("~mode")
        rospy.loginfo(f"Rosbulla running on Mode {mode}")
        if mode == 1:
            time.sleep(1)                
            master.searchnext_pub.publish(True)
            rospy.Subscriber("/found", Bool, master.callback_search_postit)
        else:
            master.publish_first_postit_number(master.pub)
            rospy.Subscriber("/found", Bool, master.callback_found_postit)
            rospy.Subscriber("/found_postit", Int32, master.callback_extern_found_postit)
            
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
