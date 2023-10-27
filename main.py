#####################################
#   Main Code For Mission Driving   #
#   2023-08-09                      #
#   Author : NKH                    #
#####################################

from scripts.core import *       
from module import *

parallel  =  PARALLEL()


if __name__ == "__main__":
     
    Wa                 =   WayMaker()
    data_IO            =   DataCommunication("morai")
    cmd_gear           =   4
    cmd_velocity       =   7
    cmd_steer          =   0
    cmd_brake          =   0
    lat,lon,x,y,yaw    =   data_IO.get_att_data(Wa.init_lat,Wa.init_lon)
    lidar_data         =   []
    target_ind ,_      =   Wa.Trajectory.search_target_index(x, y)

    
    while  isEnd() == False:
        
        lat, lon, x, y, yaw  = data_IO.get_att_data(Wa.init_lat,Wa.init_lon)
        lidar_data           = data_IO.get_lidar_data()
        
        target_ind , _  =  Wa.Trajectory.search_target_index(x, y)
        
        cmd_steer  , _  =  impact_Angle_steer_control(x, y, yaw, Wa.cyaw, Wa.Trajectory, target_ind)
         
        parallel.parallel_parking(lat, lon, yaw, lidar_data, [cmd_gear, cmd_velocity, cmd_steer, cmd_brake])               # parallel parking algorithm       

        data_IO.command_to_vehicle(parallel.gearcmd, parallel.steercmd, parallel.velcmd, parallel.brakecmd)
        
        # plot
        # plt.plot(wp_x,wp_y,'k*')
        tx = Wa.cx[target_ind]
        ty = Wa.cy[target_ind]

        
        
    
    
    
    
    
