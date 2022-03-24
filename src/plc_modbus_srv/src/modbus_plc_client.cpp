   #include "ros/ros.h"
   #include "plc_modbus_srv/modbus_plc.h"
   #include <cstdlib>
   
   int main(int argc, char **argv)
   {
     ros::init(argc, argv, "modbus_plc_client");
     if (argc != 2)
     {
       ROS_INFO("usage: type the integer to send to modbus");
       return 1;
     }
   
     ros::NodeHandle n;
     ros::ServiceClient client = n.serviceClient<plc_modbus_srv::modbus_plc>("modbus_plc");
     plc_modbus_srv::modbus_plc srv;
     srv.request.request= atoll(argv[1]);
     if (client.call(srv))
     {
       ROS_INFO("Response: %ld", (long int)srv.response.register_response);
     }
     else
     {
       ROS_ERROR("Failed to call service modbus_plc");
       return 1;
     }
   
     return 0;
   }
