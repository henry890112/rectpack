#!/usr/bin python3

from rectpack import newPacker
import random
from time import sleep
import rospy
from rectpack.msg import AfterPackPose
from rectpack.msg import BeforePackPose
'''
can pub the obb properties to the topic /obb_data
and use the pack algorithm to pack the obb
the result will pub to the topic /pack_algorithm to execute in the real robot

'''
'''
float64[3] obb_xyz (before_pack_pose)
(rwidth, rheight, rid)

float64[6] after_pack_pose
# (b, x, y, w, h, rid)
# b - Bin index
# x - Rectangle bottom-left corner x coordinate
# y - Rectangle bottom-left corner y coordinate
# w - Rectangle width
# h - Rectangle height
# rid - User asigned rectangle id or None
'''

class PackNode():
    def __init__(self):
        self.N_BINS = 1
        self.BIN_WIDTH = 30
        self.BIN_HEIGHT = 30

        class Enum(tuple): 
            __getattr__ = tuple.index
            
        PackingMode = Enum(["Online", "Offline"])
        PackingBin = Enum(["BNF", "BFF", "BBF", "Global"])

        self.packer0 = newPacker(
            mode=PackingMode.Online,
            bin_algo=PackingBin.BBF,
            rotation=False,
        )

        self.packer1 = newPacker(
            mode=PackingMode.Online,
            bin_algo=PackingBin.BBF,    
            rotation=False,
        )

        self.packer2 = newPacker(
            mode=PackingMode.Online,
            bin_algo=PackingBin.BBF,
            rotation=False,
        )
        '''
        要先分好rectangles類別  0: turp, 1: orange, 2: cucumber
        '''
        for _ in range(self.N_BINS):
            self.packer0.add_bin(width=self.BIN_WIDTH, height=self.BIN_HEIGHT)
            self.packer1.add_bin(width=self.BIN_WIDTH, height=self.BIN_HEIGHT)
            self.packer2.add_bin(width=self.BIN_WIDTH, height=self.BIN_HEIGHT)

        self.pub_bin = rospy.Publisher('/pack_algorithm', AfterPackPose, queue_size=100)
        # pub the obb data on the topic to execute the pack algorithm
        self.sub = rospy.Subscriber('/obb_data', BeforePackPose, self.pack_callback)

    def check_bin_full(self, rectangle, rid):
        if rectangle is False:
            print("Could not pack in bin {} anymore!!!!!!!".format(rid))
        return rectangle, rid

    def pack_callback(self, data):
        rwidth, rheight, rid = data.obb_xyz
        # change the float to the int type
        # rwidth = int(rwidth)
        # rheight = int(rheight)
        # rid = int(rid)
        bin_result = AfterPackPose()
        if rid ==0:
            rectangle0 = self.packer0.add_rect(rwidth, rheight, rid=0)
            all_rects_0 = self.packer0.rect_list()
            flag, rid = self.check_bin_full(rectangle0, rid)
            print("bin 0 length：", len(self.packer0.rect_list()))
            result = all_rects_0[-1]
            
        elif rid ==1:
            rectangle1 = self.packer1.add_rect(rwidth, rheight, rid=1)
            all_rects_1 = self.packer1.rect_list()
            flag, rid = self.check_bin_full(rectangle1, rid)
            print("bin 1 length：", len(self.packer1.rect_list()))
            result = all_rects_1[-1]
            

        elif rid ==2:
            rectangle2 = self.packer2.add_rect(rwidth, rheight, rid=2)
            all_rects_2 = self.packer2.rect_list()
            flag, rid = self.check_bin_full(rectangle2, rid)
            print("bin 2 length：", len(self.packer2.rect_list()))
            result = all_rects_2[-1]
        # change the result to the tuple to the list type
        result = list(result)
        bin_result.after_pack_pose = result
        print(bin_result)
        self.pub_bin.publish(bin_result)




if __name__ == '__main__':
    rospy.init_node('pack_algorithm', anonymous=True, disable_signals=True)
    rospy.loginfo("Init pack node")
    pack_node = PackNode()
    rate = rospy.Rate(10)
    
    try:
        while True:
            rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo('Shutting down...')
