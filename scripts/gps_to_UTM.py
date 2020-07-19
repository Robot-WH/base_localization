#!/usr/bin/env python  
 
import rospy
from gps_pkg.msg import sensorgps
from gps_pkg.msg import sensorgps_utm
import WGS84

lon=0        
lat=0          
roadtype=0      
lanetype=0       
heading=0      
pitch=0        
roll=0        
velocity=0     
status=0         
satenum=0   
gpstime=0       
isvalid=0      
timestamp=0    

send_flag=False

def gps_callback(data):
    global lon
    global lat
    global roadtype
    global lanetype
    global heading
    global pitch
    global roll
    global velocity
    global status
    global satenum
    global gpstime
    global isvalid
    global timestamp
    global send_flag
    if send_flag is False:
        lon=data.lon
        lat=data.lat
        print(lon,lat)
        roadtype=data.roadtype
        lanetype=data.lanetype
        heading=data.heading
        pitch=data.pitch
        roll=data.roll
        velocity=data.velocity
        status=data.status
        satenum=data.satenum
        gpstime=data.gpstime
        isvalid=data.isvalid
        timestamp=data.timestamp
        send_flag=True

gps_UTM=sensorgps_utm()

if __name__=='__main__':
    rospy.init_node('GPS_conversion', anonymous=True)
    rospy.Subscriber('data_gps', sensorgps, gps_callback)
    gps_utm_pub=rospy.Publisher('gps_UTM', sensorgps_utm, queue_size=5)
    while not rospy.is_shutdown():
        if send_flag:
            gps_UTM.x,gps_UTM.y=WGS84.CoordConverter.LatLonToUTMXY(lat,lon)    
            print(gps_UTM.x,gps_UTM.y)         
            gps_UTM.roadtype=roadtype
            gps_UTM.lanetype=lanetype
            gps_UTM.heading=heading
            gps_UTM.pitch=pitch
            gps_UTM.roll=roll
            gps_UTM.velocity=velocity
            gps_UTM.status=status
            gps_UTM.satenum=satenum
            gps_UTM.gpstime=gpstime
            gps_UTM.isvalid=isvalid
            gps_UTM.timestamp=timestamp
            gps_utm_pub.publish(gps_UTM)
            send_flag=False