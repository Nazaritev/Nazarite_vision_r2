#include "rc_serial/rc_usb.h"

/*
    先检索出所有的设备
*/
std::pair<std::vector<std::string>, std::vector<std::string>> DOGUART::scanSerialDevices() {
    std::vector<std::string> ACMdeviceList;
    std::vector<std::string> USBdeviceList;
    std::regex acm("^ttyACM\\d+$");  // 匹配以 "ttyACM" 开头并跟着数字的设备文件名
    std::regex usb("^ttyUSB\\d+$");  // 匹配以 "ttyUSB" 开头并跟着数字的设备文件名

    namespace bfs = boost::filesystem;
    for (bfs::directory_iterator it("/dev"); it != bfs::directory_iterator(); ++it) {
        std::string filename = it->path().filename().string(); 
        // 使用正则表达式检查设备名是否符合 "ttyACM" 格式
        if (std::regex_match(filename, acm)) {
            ACMdeviceList.push_back(it->path().string());
        }
        // 使用正则表达式检查设备名是否符合 "ttyUSB" 格式
        if (std::regex_match(filename, usb)) {
            USBdeviceList.push_back(it->path().string());
        }
    }
    return std::make_pair(ACMdeviceList, USBdeviceList);
}

void DOGUART::scan_usb()
{
    while (!serial_connected)  // 或者检查是否连接了至少一个设备
    {
        try
        {
            auto device_pair = scanSerialDevices();
            ACM_devices = device_pair.first;
            USB_devices = device_pair.second;
            
            if (ACM_devices.empty() && USB_devices.empty())
            {
                std::cout << "未找到串口设备，正在重新扫描..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            } 
            len_ACM = ACM_devices.size();
            len_USB = USB_devices.size();
            std::cout << "找到" << len_ACM << "个ttyACM相关串口设备" << std::endl;
            std::cout << "找到" << len_USB << "个ttyUSB相关串口设备" << std::endl;
            int success_count = 0;
            // 尝试连接所有ACM设备
            for (int i = 0; i < len_ACM; i++)
            {   
                std::cout << "尝试连接ACM设备: " << ACM_devices[i] << std::endl;
                try
                {
                    serial_port* sp = new serial_port(iosev, ACM_devices[i]);
                    sp->set_option(serial_port::baud_rate(115200));
                    sp->set_option(serial_port::flow_control(serial_port::flow_control::none));
                    sp->set_option(serial_port::parity(serial_port::parity::none));
                    sp->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
                    sp->set_option(serial_port::character_size(8));
                    if (sp->is_open()) {
                        // sp_ptrs.push_back(sp);
                        // connected_ports.push_back(ACM_devices[i]);
                        std::string physical_id = DOGUART::get_port_info(ACM_devices[i]);
                        if (!physical_id.empty()) {
                            physical_to_sp[physical_id] = sp;
                            sp_ptrs.push_back(sp);
                            connected_ports.push_back(ACM_devices[i]);
                            std::cout << "成功连接到ACM设备: " << ACM_devices[i] << " 物理ID: " << physical_id << std::endl;
                        }
                        success_count++;
                        // std::cout << "成功连接到ACM设备: " << ACM_devices[i] << std::endl;
                    } else {
                        delete sp;
                        std::cout << "无法打开ACM设备: " << ACM_devices[i] << std::endl;
                    }
                }
                catch (const boost::system::system_error& e)
                {
                    std::cout << "连接ACM设备失败 " << ACM_devices[i] << ": " << e.what() << std::endl;
                }
            }
            // 尝试连接所有USB设备
            for (int i = 0; i < len_USB; i++)
            {   
                std::cout << "尝试连接USB设备: " << USB_devices[i] << std::endl;
                try
                {
                    serial_port* sp = new serial_port(iosev, USB_devices[i]);
                    sp->set_option(serial_port::baud_rate(115200));
                    sp->set_option(serial_port::flow_control(serial_port::flow_control::none));
                    sp->set_option(serial_port::parity(serial_port::parity::none));
                    sp->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
                    sp->set_option(serial_port::character_size(8));
                    if (sp->is_open()) {
                        // sp_ptrs.push_back(sp);
                        // connected_ports.push_back(USB_devices[i]);
                        std::string physical_id = DOGUART::get_port_info(USB_devices[i]);
                        if (!physical_id.empty()) {
                            // port_map[physical_id] = USB_devices[i];
                            // std::cout << "成功连接到USB设备: " << USB_devices[i] << " 物理ID: " << physical_id << std::endl;
                            physical_to_sp[physical_id] = sp;
                            sp_ptrs.push_back(sp);
                            connected_ports.push_back(USB_devices[i]);
                            std::cout << "成功连接到USB设备: " << USB_devices[i] << " 物理ID: " << physical_id << std::endl;
                        }
                        success_count++;
                        // std::cout << "成功连接到USB设备: " << USB_devices[i] << std::endl;
                    } else {
                        delete sp;
                        std::cout << "无法打开USB设备: " << USB_devices[i] << std::endl;
                    }
                }
                catch (const boost::system::system_error& e)
                {
                    std::cout << "连接USB设备失败 " << USB_devices[i] << ": " << e.what() << std::endl;
                }
            }
            // 检查是否连接成功
            if (success_count > 0) {
                serial_connected = true;
            } else {
                std::cout << "所有设备连接尝试失败,1秒后重新扫描..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        } 
        catch (const boost::system::system_error& e) 
        {
            std::cout << "扫描过程中发生错误: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

std::string DOGUART::get_port_info(const std::string& device_path)
{   
    std::string device_name = device_path;
    if (device_path.find("/dev/") == 0) {
        device_name = device_path.substr(5); // 移除 "/dev/"
    }
    struct udev *udev = udev_new();
    if (!udev) {
        std::cerr << "无法初始化udev" << std::endl;
        return "";
    }
    struct udev_device *dev = udev_device_new_from_subsystem_sysname(udev, "tty", device_name.c_str());
    std::string id_path_res = "";
    if (dev){
        const char* id_path = udev_device_get_property_value(dev, "ID_PATH");
        const char* syspath = udev_device_get_syspath(dev);
        // std::cout << "设备[" << device_name << "] 信息" << std::endl;
        if (id_path){
            // std::cout << "ID_PATH: " << id_path << std::endl;
            id_path_res = std::string(id_path);
        }
        else{
            std::cout << "未找到ID_PATH属性" << std::endl;
        }
        // std::cout << "SYSPATH: " << syspath << std::endl;
        udev_device_unref(dev);
        return id_path_res;
    }else{
        std::cout << "未找到设备[" << device_name << "]" << std::endl;
        return "";
    }
}

/*
    发送数据(数组类型)
*/
bool DOGUART::send_usb(const std::string& physical_id,const float* data, size_t data_size) {
    int index = 1;  // 用于跟踪 msg_temp 中的位置
    // 将输入的 float 数据依次复制到 msg_queue 中
    for (size_t i = 0; i < data_size; ++i) {
        memcpy(&msg_queue[index], &data[i], sizeof(float));
        index += sizeof(float);
    }
    // 串口写数据
    msg_queue[0] = 0xAA;
    msg_queue[17] = 0xBB;
    boost::system::error_code err;
    try {
        auto it = physical_to_sp.find(physical_id);
        if (it != physical_to_sp.end()) {
            boost::asio::serial_port* sp = it->second;
            sp->write_some(boost::asio::buffer(msg_queue, sizeof(msg_queue)), err);
            if (err)
            {
                if (err == boost::asio::error::eof) {
                std::cout << "串口连接已断开，尝试重新扫描串口设备..." << std::endl;
                // 关闭当前串口
                if (sp_ptr!= nullptr) {
                    sp_ptr->close();
                    delete sp_ptr;
                    sp_ptr = nullptr;
                }
                serial_connected = false;
                // 进入重新扫描串口设备的循环
                scan_usb();
                } else {
                std::cout << "串口连接已断开，尝试重新扫描串口设备..." << std::endl;
                // 关闭当前串口
                if (sp_ptr!= nullptr) {
                    sp_ptr->close();
                    delete sp_ptr;
                    sp_ptr = nullptr;
                }
                serial_connected = false;
                // 进入重新扫描串口设备的循环
                scan_usb();
                }
            }
        }else{}
    } catch (const boost::system::system_error& e) {
        std::cout << "系统错误，发送串口数据时出现异常: " << e.what() << std::endl;     
    }
    return true;
}
/*
    发送数据(单一字符串类型)
*/
bool DOGUART::send_usb_string(const std::string& physical_id,const std::string& msg) {
    boost::system::error_code err;
    try{
        auto it = physical_to_sp.find(physical_id);
        if (it != physical_to_sp.end()) {
            boost::asio::serial_port* sp = it->second;
            sp->write_some(boost::asio::buffer(msg), err);
            if (err)
            {
                if (err == boost::asio::error::eof) {
                std::cout << "串口连接已断开，尝试重新扫描串口设备..." << std::endl;
                // 关闭当前串口
                if (sp_ptr!= nullptr) {
                    sp_ptr->close();
                    delete sp_ptr;
                    sp_ptr = nullptr;
                }
                serial_connected = false;
                // 进入重新扫描串口设备的循环
                scan_usb();
                } else {
                std::cout << "串口连接已断开，尝试重新扫描串口设备..." << std::endl;
                // 关闭当前串口
                if (sp_ptr!= nullptr) {
                    sp_ptr->close();
                    delete sp_ptr;
                    sp_ptr = nullptr;
                }
                serial_connected = false;
                // 进入重新扫描串口设备的循环
                scan_usb();
                }
            }
        }else{}
    } catch (const boost::system::system_error& e) {
        std::cout << "系统错误，发送串口数据时出现异常: " << e.what() << std::endl;     
    }
    return true;
}