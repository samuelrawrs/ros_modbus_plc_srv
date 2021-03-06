#include "ros/ros.h"
#include "plc_modbus_srv/modbus_plc.h"
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <modbus/modbus.h>

class plc_modbus_manager {
    public:
      plc_modbus_manager();
      std_msgs::UInt16MultiArray regs_val;
      std_msgs::ByteMultiArray coils_val;
      modbus_t *plc;
      void write_to_reg(const int value);


    private:
      ros::NodeHandle node;
      
      std::vector<int> regs_addr;
      std::vector<int> coils_addr;



      std::string ip_address;
      int port;
      int spin_rate;

      void regs_callBack(const std_msgs::UInt16MultiArray::ConstPtr &regs_data);
      void coils_callBack(const std_msgs::ByteMultiArray::ConstPtr &coils_data);
  };
  
plc_modbus_manager::plc_modbus_manager() {

    node.param<std::string>("modbus_plc_server/ip", ip_address, "192.168.0.100");
    node.param("modbus_plc_server/port", port, 502);
    node.param("modbus_plc_server/spin_rate",spin_rate,30);

    if (!node.getParam("modbus_plc_server/regs_addr", regs_addr)) {
        ROS_WARN("No reg addrs given!");
    }
    if (!node.getParam("modbus_plc_server/coils_addr", coils_addr)) {
        ROS_WARN("No coil addrs given!");
    }

    ROS_INFO("Connecting to modbus device on %s/%d", ip_address.c_str(), port);
    plc = modbus_new_tcp(ip_address.c_str(), port);
    if (plc == NULL) {
        ROS_FATAL("Unable to allocate libmodbus context\n");
        return;
    }
    if (modbus_connect(plc) == -1) {
        ROS_FATAL("Failed to connect to modbus device!!!");
        ROS_FATAL("%s", modbus_strerror(errno));
        modbus_free(plc);
        return;
    } else {
        ROS_INFO("Connection to modbus device established");
    }

    // ros::Rate loop_rate(spin_rate);

    // while (ros::ok()) {
    //     regs_val.data.clear();
    //     coils_val.data.clear();

    //     for (int i = 0; i < regs_addr.size(); i++) {
    //         uint16_t temp[1] = {0};
    //         if (modbus_read_registers(plc, regs_addr.at(i), 1, temp) == -1) {
    //             ROS_ERROR("Unable to read reg addr:%d", regs_addr.at(i));
    //             ROS_ERROR("%s", modbus_strerror(errno));
    //         } else {
    //             regs_val.data.push_back(temp[0]);
    //         }
    //     }

    //     for (int i = 0; i < coils_addr.size(); i++) {
    //         uint8_t temp[1] = {0};
    //         if (modbus_read_bits(plc, coils_addr.at(i), 1, temp) == -1) {
    //             ROS_ERROR("Unable to read coil addr:%d", coils_addr.at(i));
    //             ROS_ERROR("%s", modbus_strerror(errno));
    //         } else {
    //             coils_val.data.push_back(temp[0]);
    //         }
    //     }

    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    // modbus_close(plc);
    // modbus_free(plc);
    return;
}

void plc_modbus_manager::regs_callBack(const std_msgs::UInt16MultiArray::ConstPtr &regs_data) {
    if (regs_data->data.size() != regs_addr.size()) {
        ROS_ERROR("%d registers to write but only %d given!", regs_addr.size(), regs_data->data.size());
        return;
    }
    for (int i = 0; i < regs_data->data.size(); i++) {
        ROS_DEBUG("regs_out[%d]:%u", i, regs_data->data.at(i));
        uint16_t temp[1] = {regs_data->data.at(i)};
        if (modbus_write_registers(plc, regs_addr.at(i), 1, temp) == -1) {
            ROS_ERROR("Modbus reg write failed at addr:%d with value:%u", regs_addr.at(i), regs_data->data.at(i));
            ROS_ERROR("%s", modbus_strerror(errno));
        } else {
            ROS_INFO("Modbus register write at addr:%d with value:%u", regs_addr.at(i), regs_data->data.at(i));
        }
    
}
}

void plc_modbus_manager::coils_callBack(const std_msgs::ByteMultiArray::ConstPtr &coils_data) {
    if (coils_data->data.size() != coils_addr.size()) {
        ROS_ERROR("%d coils to write but %d given!", coils_addr.size(), coils_data->data.size());
        return;
    }
    for (int i = 0; i < coils_data->data.size(); i++) {
        ROS_DEBUG("regs_out[%d]:%u", i, coils_data->data.at(i));
        uint8_t temp[1] = {coils_data->data.at(i)};
        if (modbus_write_bits(plc, coils_addr.at(i), 1, temp) == -1) {
            ROS_ERROR("Modbus coil write failed at addr:%d with value:%u", coils_addr.at(i), coils_data->data.at(i));
            ROS_ERROR("%s", modbus_strerror(errno));
        } else {
            ROS_INFO("Modbus coil write at addr:%d with value:%u", coils_addr.at(i), coils_data->data.at(i));
        }
    }
}

void plc_modbus_manager::write_to_reg(const int value) {
        ROS_INFO("hello this is the value, %d", value);
        if (modbus_write_register(plc, regs_addr.at(0), value) == -1) {
            ROS_ERROR("Modbus reg write failed at addr:%d with value:%u", regs_addr.at(0), value);
            ROS_ERROR("%s", modbus_strerror(errno));
        } else {
            ROS_INFO("Modbus register write at addr:%d with value:%u", regs_addr.at(0), value);
        }
    
}

bool servicename(plc_modbus_srv::modbus_plc::Request  &req,
          plc_modbus_srv::modbus_plc::Response &res)
   {
     ROS_INFO("request: %d", (int)req.request);
     plc_modbus_manager mm;
     mm.write_to_reg(int(req.request));
     ROS_INFO("sending back response: coil: [%ld], register: [%ld]", (long int)res.coil_response,(long int)res.register_response);
     return true;
   }
   
   int main(int argc, char **argv)
   {
    ros::init(argc, argv, "modbus_plc_server");
    ros::NodeHandle node;
    ros::ServiceServer service = node.advertiseService("modbus_plc", servicename);
    ROS_INFO("Ready to send command to modbus");
    ros::spin();
   
     return 0;
   }
