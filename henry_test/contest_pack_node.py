#!/usr/bin/env python

from rectpack import newPacker
import random
from time import sleep
import rospy
# from pack_ros.msg import AfterPackPose
# from pack_ros.msg import BeforePackPose
'''
flaoat64 rid
float64[3] before_pack_pose
(rwidth, rheight)

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

        # self.pub_bin = rospy.Publisher('pack_algorithm', AfterPackPose, queue_size=10)
        # self.sub_obb = rospy.Subscriber('pack_algorithm', BeforePackPose, self.obb_callback)

    def check_bin_full(self, rectangle, rid):
        if rectangle is False:
            print("Could not pack in bin {} anymore!!!!!!!".format(rid))
        return rectangle

    def obb_callback(self, msg):
        self.rwidth = msg.rwidth
        self.rheight = msg.rheight
        self.rid = msg.rid

    def pack_callback(self, rwidth, rheight, rid):
        # bin_result = AfterPackPose()
        if rid ==0:
            rectangle0 = self.packer0.add_rect(rwidth, rheight, rid=0)
            all_rects_0 = self.packer0.rect_list()
            flag0 = self.check_bin_full(rectangle0, rid)
            # 一次傳入一個rectangle
            # bin_result.after_pack_pose = all_rects_0[:-1]
            print(all_rects_0[:-1])
            # plot(all_rects_0)

        elif rid ==1:
            rectangle1 = self.packer1.add_rect(rwidth, rheight, rid=1)
            all_rects_1 = self.packer1.rect_list()
            flag1 = self.check_bin_full(rectangle1, rid)
            # bin_result.after_pack_pose = all_rects_1[:-1]
            print(all_rects_1[:-1])

            # plot(all_rects_1)

        elif rid ==2:
            rectangle2 = self.packer2.add_rect(rwidth, rheight, rid=2)
            print("bin 2 length：", len(self.packer2.rect_list()))
            all_rects_2 = self.packer2.rect_list()
            # for_dynamic_plot_rectangle().plot(all_rects_2)
            flag2 = self.check_bin_full(rectangle2, rid)
            # bin_result.after_pack_pose = all_rects_2[:-1]
            print(all_rects_2[:-1])
        
        # self.pub_bin.publish(bin_result)


if __name__ == '__main__':
    rospy.init_node('pack_algorithm', anonymous=True, disable_signals=True)
    rospy.loginfo("Init pack node")
    pack_node = PackNode()
    # pack_result = AfterPackPose()
    rate = rospy.Rate(10)
    try:
        while True:
            
            # rwidth = pack_result.before_pack_pose[0]
            # rheight = pack_result.before_pack_pose[1]
            # rid = pack_result.rid
            # create a subscriber to get the data from the topic
            
            rwidth = random.randint(5, 10)
            rheight = random.randint(5, 10)
            rid = random.randint(0, 2)

            pack_node.pack_callback(rwidth, rheight, rid)
            rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo('Shutting down...')
