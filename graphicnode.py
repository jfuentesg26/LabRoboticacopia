#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from math import pi, atan2
import cv2
from nav_msgs.msg import OccupancyGrid
import random
from scipy import spatial
import math
import time
from geometry_msgs.msg import PoseArray, Pose


class GraphicNode( object ):

    def __init__( self ):
        rospy.init_node( 'GraphicNode' )
        # Para mapa 
        self.mapimg = np.array( [] )
        self.map_resolution = 0
        self.load_map = False
        rospy.Subscriber('/map', OccupancyGrid, self.set_map)
        rospy.Subscriber('/part', PoseArray, self.draw_part)
        self.rate_obj = rospy.Rate( 10 )

    def set_map(self, occupancy_grid ):
        self.width = occupancy_grid.info.width
        self.height = occupancy_grid.info.height
        self.map_resolution = occupancy_grid.info.resolution
        print( 'map resolution: (%d,%d)' % (self.height, self.width) )
        self.mapimg = 100 - np.array( occupancy_grid.data ).reshape( (self.height, self.width) )
        self.mapimg = ( self.mapimg * (255/100.0) ).astype( np.uint8 )
        self.mapimg = cv2.cvtColor( self.mapimg, cv2.COLOR_GRAY2RGB )
        self.mapimg = cv2.resize( self.mapimg, (self.width, self.height), interpolation = cv2.INTER_NEAREST )
        self.load_map = True
        

    def draw_part(self, part):
        if len( self.mapimg ) == 0:
            return None
        while not self.load_map:
            rospy.sleep(0.2)
        mapimg_laser = self.mapimg.copy()
        for p in part.poses:
            zkx_pix = int( p.position.x / self.map_resolution )
            zky_pix = - int( (p.position.y - self.mapimg.shape[0]*self.map_resolution) / self.map_resolution )
            radio = 2
            if 0 < zky_pix-radio and zky_pix-radio < mapimg_laser.shape[0] and \
                0 < zky_pix+radio and zky_pix+radio < mapimg_laser.shape[0] and \
                0 < zkx_pix-radio and zkx_pix-radio < mapimg_laser.shape[1] and \
                0 < zkx_pix+radio and zkx_pix+radio < mapimg_laser.shape[1]:
                angulo = p.orientation.z*180/math.pi
                if angulo < 0:
                    angulo = 360 + angulo
                    rojo = int(255 - (angulo*0.7))
                    verde = int(angulo*0.7)
                    color = (0, verde, rojo)
                    mapimg_laser = cv2.rectangle(mapimg_laser,(zkx_pix, zky_pix),(zkx_pix + radio ,zky_pix+ radio),color,-1)
                else:
                    rojo = int(255 - (angulo*0.7))
                    verde = int(angulo*0.7)
                    center = (zkx_pix, zky_pix)
                    color = (0, verde, rojo)
                    mapimg_laser = cv2.circle(mapimg_laser, center, radio, color, -1)
        mapimg_laser = cv2.resize( mapimg_laser, (self.width*2, self.height*2), interpolation = cv2.INTER_NEAREST )
        cv2.imshow( '2D Map', mapimg_laser )
        cv2.waitKey( 3000 )

        

if __name__ == '__main__':
    GraphicNode()
    rospy.spin()
