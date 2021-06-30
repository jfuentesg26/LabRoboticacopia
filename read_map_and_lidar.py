#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

import numpy as np
import cv2

mapimg = np.array( [] )
map_resolution = 0

def set_map( occupancy_grid ):
  global mapimg, converter, map_resolution
  width = occupancy_grid.info.width
  height = occupancy_grid.info.height
  map_resolution = occupancy_grid.info.resolution
  print( 'map resolution: (%d,%d)' % (height, width) )
  mapimg = 100 - np.array( occupancy_grid.data ).reshape( (height, width) )
  mapimg = ( mapimg * (255/100.0) ).astype( np.uint8 )
  mapimg = cv2.cvtColor( mapimg, cv2.COLOR_GRAY2RGB )
  mapimg = cv2.resize( mapimg, (width, height), interpolation = cv2.INTER_NEAREST )

def laser_scan_hd( scan ):
  global mapimg, map_resolution
  if len( mapimg ) == 0:
    return None
  robot_pose = [1.0, 1.0, 0.0] # [m]
  robot_pose_pix_x = int( robot_pose[0] / map_resolution )
  robot_pose_pix_y = - int( (robot_pose[1]-mapimg.shape[0]*map_resolution) / map_resolution )
  robot_pose_pix = [robot_pose_pix_x, robot_pose_pix_y, robot_pose[2]]
  mapimg_laser = mapimg.copy()
  angles = np.linspace( scan.angle_min, scan.angle_max, int( (scan.angle_max-scan.angle_min)/scan.angle_increment ) )
  for angle, zk in zip( angles, scan.ranges ):
    if zk < scan.range_max:
      zkx = zk * np.cos( angle + robot_pose[2] ) + robot_pose[0]
      zky = zk * np.sin( angle + robot_pose[2] ) + robot_pose[1]
      zkx_pix = int( zkx / map_resolution )
      zky_pix = - int( (zky-mapimg.shape[0]*map_resolution) / map_resolution )
      radio = 1
      if 0 < zky_pix-radio and zky_pix-radio < mapimg_laser.shape[0] and \
         0 < zky_pix+radio and zky_pix+radio < mapimg_laser.shape[0] and \
         0 < zkx_pix-radio and zkx_pix-radio < mapimg_laser.shape[1] and \
         0 < zkx_pix+radio and zkx_pix+radio < mapimg_laser.shape[1]:
        mapimg_laser[zky_pix-radio:zky_pix+radio+1, zkx_pix-radio:zkx_pix+radio+1, 0] = 0
        mapimg_laser[zky_pix-radio:zky_pix+radio+1, zkx_pix-radio:zkx_pix+radio+1, 1] = 0
        mapimg_laser[zky_pix-radio:zky_pix+radio+1, zkx_pix-radio:zkx_pix+radio+1, 2] = 255
  robot_radio = int( (0.355/2.0) / map_resolution )
  cv2.circle( mapimg_laser, tuple( robot_pose_pix[:2] ), robot_radio, (0, 0, 255), -1 )
  cv2.imshow( '2D Map', mapimg_laser )
  cv2.waitKey( 1 )

if __name__ == '__main__':
  rospy.sleep(15)
  rospy.init_node( 'show_lidar' )
  rospy.Subscriber( 'map', OccupancyGrid, set_map )
  rospy.Subscriber( '/scan', LaserScan, laser_scan_hd, queue_size = 1 )
  rospy.spin()


