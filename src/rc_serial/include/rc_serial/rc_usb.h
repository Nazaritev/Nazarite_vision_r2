#ifndef CPP_MYUSB_H
#define CPP_MYUSB_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include<thread>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <regex>
#include <string>
#include <utility>   
#include <libudev.h>

using namespace std;        // IO 服务，用于处理异步操作
using namespace boost::asio;
using namespace boost::placeholders;

class DOGUART{
public:
    /*
        搜索串口设备
    */
    // 修改为支持多个连接
    std::map<std::string, std::string> port_map;
    std::map<std::string, boost::asio::serial_port*> physical_to_sp;  // 多个连接
    std::vector<serial_port*> sp_ptrs;         // 多个连接
    std::vector<std::string> connected_ports;  // 已连接的端口
    std::pair<std::vector<std::string>, std::vector<std::string>> scanSerialDevices();
    void scan_usb();
    void receive_usb();
    bool send_usb(const std::string& physical_id,const float* data, size_t data_size); 
    bool send_usb_string(const std::string& physical_id,const std::string& data);
    std::string get_port_info(const std::string& device_name);
    // 访问器方法，获取数组元素
    // 若索引无效，返回 -1 表示错误
    float getArrayElement(int index) const {
        if (index >= 0 && index < result.size()) {
            return result[index];
        }
        // 索引无效，返回 -1 作为错误标识
        return -1; 
    }

    uint8_t get_rxData(int index) const {
        return rxData[index]; 
    }

    // 导入外部数据的函数
    void importData(const float inputData[8]) {
        for (int i = 0; i < 8; ++i) {
            txData[i] = inputData[i];
        }
    }

    void async_send_usb(const float* data, size_t size) {
        if(sp_ptr && sp_ptr->is_open()) {
            boost::asio::async_write(*sp_ptr, 
                boost::asio::buffer(data, size*sizeof(float)),
                [this](const boost::system::error_code& ec, size_t) {
                    if(ec) handle_error(ec);
                });
        }
    }
    
    // 添加发送状态检查
    bool check_write_ready() const {
        return sp_ptr && sp_ptr->is_open();
    }
    
    // 错误处理函数
    void handle_error(const boost::system::error_code& ec) {
        // 错误处理逻辑...
    }

    bool serial_connected = false;

private:
    std::vector<std::string> ACM_devices;
    std::vector<std::string> USB_devices;
    io_service iosev;
    // 创建serial_port对象指针
    serial_port* sp_ptr = nullptr; 
    boost::system::error_code err;  
    //接受数据的长度
    size_t bytes_read;
    // 串口固定消息队列
    uint8_t msg_queue[18];
    // 串口固定接受缓冲
    uint8_t rxData[108];
    // 串口固定发送缓冲
    float txData[3];
    // 中间数据
    std::vector<float> result;
    // 相关处理参数
    int len_ACM;
    int len_USB;
};

#endif 