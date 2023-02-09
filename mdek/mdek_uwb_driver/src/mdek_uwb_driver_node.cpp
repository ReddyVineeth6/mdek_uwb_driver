//
// Created by pschroepfer on 14/10/2021.
//
#include <iostream>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <sys/signal.h>
#include <sstream>
#include <ros/ros.h>
#include <mdek_uwb_driver/Uwb_Measurement.h>
#include <mdek_uwb_driver/Uwb.h>
extern "C"{
#include "dwm_api.h"
}


class UwbDriver{
    ros::Publisher uwb_pub;
    std::string uwb_topic;
    std::string device_name;

public:
    UwbDriver(ros::NodeHandle &nh){
        uint16_t ur=1, urs=1;
        dwm_cfg_tag_t tag_config;
        dwm_anchor_list_t anchorList = {};
        dwm_uwb_scan_result_t result = {};
        dwm_loc_data_t data = {};

        nh.param<std::string>("uwb_pub_topic",uwb_topic,"uwb_distances");
        nh.param<std::string>("device_name",device_name,"/dev/ttyACM0");
        uwb_pub = nh.advertise<mdek_uwb_driver::Uwb>(uwb_topic,1);

        //connect to device
        dwm_init((char*)device_name.c_str());

        tag_config.low_power_en = false;
        tag_config.common.ble_en = true;
        tag_config.stnry_en = false; //if turned on this means the device will go in low power mode when it is stationary
        tag_config.loc_engine_en = false; //this will automatically calculate location based on position of anchors
        tag_config.meas_mode = DWM_MEAS_MODE_TWR;
        tag_config.common.led_en = false;
        tag_config.common.uwb_mode = DWM_UWB_MODE_ACTIVE;
        tag_config.common.fw_update_en = true;
        tag_config.common.enc_en = false;

        //configure the device
        dwm_upd_rate_set(ur,urs);
        dwm_upd_rate_get(&ur,&urs);
        dwm_cfg_tag_set(&tag_config);
        sleep(1);

    }
    ~UwbDriver(){
        dwm_deinit();
    }

    void Publish_Anchor_Measurements(){
        mdek_uwb_driver::Uwb distances;
        distances.header.stamp = ros::Time::now();
        dwm_loc_data_t data = {};
        dwm_loc_get(&data);
        for (int i = 0; i < data.anchors.dist.cnt; i++){
            std::stringstream id;
            mdek_uwb_driver::Uwb_Measurement m;
            id << "0x" << std::setfill('0') << std::setw(4) << std::hex <<  data.anchors.dist.addr[i];
            m.id = id.str();
            m.distance = data.anchors.dist.dist[i]/1000.0;
            m.qualty_factor = data.anchors.dist.qf[i];
            distances.ranges.push_back(m);
        }

        uwb_pub.publish(distances);
	distances.ranges.clear();

    }
};




int main (int argc, char **argv){

    ros::init(argc,argv,"mdek_uwb_driver_node");
    ros::NodeHandle nh("~");
    UwbDriver uwbDriver(nh);

    ros::Rate loop_rate(10);
    while (ros::ok()){
        uwbDriver.Publish_Anchor_Measurements();
        ros::spinOnce();
        loop_rate.sleep();
    }




}
