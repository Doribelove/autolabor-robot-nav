#include <iostream>
#include <autolabor_canbus_driver/canbus_driver.h>


namespace autolabor_driver {

    CanbusDriver::CanbusDriver() : parse_start_index_(0), parse_end_index_(0), point_index_(0), state_(0), check_sum_(0) {}

    CanbusDriver::~CanbusDriver() {
        boost::mutex::scoped_lock look(mutex_);
        if (port_) {
            port_->cancel();
            port_->close();
            port_.reset();
        }
        io_service_.stop();
        io_service_.reset();
    }

    bool CanbusDriver::init() {
        if (port_) {
            ROS_ERROR("error : port is already opened...");
            return false;
        }
        port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
        port_->open(port_name_, ec_);
        if (ec_) {
            ROS_INFO_STREAM("error : port_->open() failed...port_name=" << port_name_ << ", e=" << ec_.message().c_str());
            return false;
        }
        // option settings...
        port_->set_option(boost::asio::serial_port_base::baud_rate(static_cast<unsigned int>(baud_rate_)));
        port_->set_option(boost::asio::serial_port_base::character_size(8));
        port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        return true;
    }

    void CanbusDriver::update_parse_data() {
        size_t count = boost::asio::read(*port_.get(), boost::asio::buffer(receive_data_, 256), boost::asio::transfer_at_least(1), ec_);
        ROS_DEBUG_STREAM("RECEIVE < " << array_to_string(receive_data_, count) << ">");
        size_t remain_byte_number = parse_end_index_ - parse_start_index_;
        if (remain_byte_number > 0) {
            uint8_t tmp_buffer[512];
            std::memcpy(tmp_buffer, parse_data_ + parse_start_index_, remain_byte_number);
            std::memcpy(tmp_buffer + remain_byte_number, receive_data_, count);
            std::memcpy(parse_data_, tmp_buffer, count + remain_byte_number);
        } else {
            std::memcpy(parse_data_, receive_data_, count);
        }
        point_index_ = point_index_ - parse_start_index_;
        parse_start_index_ = 0;
        parse_end_index_ = count + remain_byte_number;

    }

    void CanbusDriver::reset_parse_data() {
        state_ = 0;
        data_tag_ = 0;
        node_type_ = 0;
        node_seq_ = 0;
        msg_type_ = 0;
        frame_id_ = 0;
        payload_.clear();
        check_sum_ = 0;
    }

    uint8_t CanbusDriver::get_and_check(bool needCheck) {
        uint8_t b = parse_data_[point_index_++];
        if (needCheck) {
            check_sum_ = CRC8Table[(check_sum_ ^ b) & 0xff];
        }
        return b;
    }

    uint8_t CanbusDriver::check_data(uint8_t *data, size_t size) {
        uint8_t checksum = 0;
        for (int i = 0; i < size; i++) {
            checksum = CRC8Table[(checksum ^ *(data + i)) & 0xff];
        }
        return checksum;
    }

    void CanbusDriver::parse_msg() {
        ros::Rate loop_rate(parse_rate_);
        while (true) {
            bool continue_flag = true;
            update_parse_data();
            while (point_index_ < parse_end_index_ && continue_flag) {
                switch (state_) {
                    case 0: { // header
                        reset_parse_data();
                        uint8_t header = get_and_check(false);
                        if (header == 0xfe) {
                            state_ = 1;
                            if (parse_end_index_ - point_index_ < 2) {
                                continue_flag = false;
                            }
                        } else {
                            point_index_ = ++parse_start_index_;
                            state_ = 0;
                        }
                        break;
                    }
                    case 1: { // netcode + datatag + priority + node_type + node_seq
                        uint8_t b1 = get_and_check(true);
                        uint8_t b2 = get_and_check(true);
                        data_tag_ = static_cast<unsigned char>((b1 >> 5) & 0x01);
                        node_type_ = static_cast<unsigned char>(((b1 & 0x03) << 4) | ((b2 >> 4) & 0x0f));
                        node_seq_ = static_cast<unsigned char>(b2 & 0x0f);
                        state_ = 2;
                        break;
                    }
                    case 2: { // msg_type
                        msg_type_ = get_and_check(true);
                        state_ = 3;
                        break;
                    }
                    case 3: { // frame_id
                        frame_id_ = get_and_check(true);
                        if (data_tag_ == 1) {
                            state_ = 4;
                            if (parse_end_index_ - point_index_ < 8) {
                                continue_flag = false;
                            }
                        } else {
                            state_ = 5;
                        }
                        break;
                    }
                    case 4: { // payload
                        for (int i = 0; i < 8; i++) {
                            payload_.push_back(get_and_check(true));
                        }
                        state_ = 5;
                        break;
                    }
                    case 5: { // check_sum
                        uint8_t msg_check = get_and_check(false);
                        if (msg_check == check_sum_) {
                            handle_parse_msg();
                            parse_start_index_ = point_index_;
                        } else {
                            point_index_ = ++parse_start_index_;
                        }
                        state_ = 0;
                        break;
                    }
                    default: {
                        break;
                    }
                }
            }
            loop_rate.sleep();
        }
    }

    void CanbusDriver::handle_parse_msg() {
        if (frame_id_ == 0) {
            autolabor_canbus_driver::CanBusMessage msg;
            msg.node_type = node_type_;
            msg.node_seq = node_seq_;
            msg.msg_type = msg_type_;
            if (data_tag_ == 1) {
                msg.payload = payload_;
            }
            canbus_msg_pub_.publish(msg);
        } else {
            ROS_INFO_STREAM("Temporarily do not support multi-frame data.");
        }
    }

    bool CanbusDriver::canbus_service(autolabor_canbus_driver::CanBusService::Request &req, autolabor_canbus_driver::CanBusService::Response &res) {
        if (req.requests.size() > 0) {
            uint8_t service_msg_cache[14];
            for (int i = 0; i < req.requests.size(); i++) {
                autolabor_canbus_driver::CanBusMessage msg = req.requests[i];
                int data_tag = msg.payload.size() > 0 ? 1 : 0;
                service_msg_cache[0] = 0xfe;
                service_msg_cache[1] = static_cast<uint8_t>(((data_tag & 0x01) << 5) | ((msg.node_type >> 4) & 0x03));
                service_msg_cache[2] = static_cast<uint8_t>(((msg.node_type & 0x0f) << 4) | (msg.node_seq & 0x0f));
                service_msg_cache[3] = static_cast<uint8_t >(msg.msg_type & 0xff);
                service_msg_cache[4] = 0x00;
                if (data_tag == 1) {
                    int payload_len = static_cast<int>(fminl(msg.payload.size(), 8));
                    for (int j = 0; j < payload_len; j++) {
                        service_msg_cache[5 + j] = msg.payload[j];
                    }
                    service_msg_cache[13] = check_data(&service_msg_cache[1], 12);
                    boost::asio::write(*port_.get(), boost::asio::buffer(service_msg_cache, 14), ec_);
                    ROS_DEBUG_STREAM("SEND < " << array_to_string(service_msg_cache, 14) << ">"
                                              << " NODE_TYPE:" << (int)msg.node_type << ","
                                              << " NODE_SEQ:" << (int)msg.node_seq << ","
                                              << " MSG_TYPE:" << (int)msg.msg_type);
                } else {
                    service_msg_cache[5] = check_data(&service_msg_cache[1], 4);
                    boost::asio::write(*port_.get(), boost::asio::buffer(service_msg_cache, 6), ec_);
                    ROS_DEBUG_STREAM("SEND < " << array_to_string(service_msg_cache, 6) << ">"
                                             << " NODE_TYPE:" << (int)msg.node_type << ","
                                             << " NODE_SEQ:" << (int)msg.node_seq << ","
                                             << " MSG_TYPE:" << (int)msg.msg_type);
                }
            }
            return true;
        } else {
            return false;
        }

    }

    void CanbusDriver::run() {
        ros::NodeHandle node;
        ros::NodeHandle private_node("~");

        private_node.param<std::string>("port_name", port_name_, std::string("/dev/ttyUSB0"));
        private_node.param<int>("baud_rate", baud_rate_, 115200);
        private_node.param<int>("parse_rate", parse_rate_, 100);

        if (init()) {
            canbus_msg_service_ = node.advertiseService("canbus_server", &CanbusDriver::canbus_service, this);
            canbus_msg_pub_ = node.advertise<autolabor_canbus_driver::CanBusMessage>("canbus_msg", 10);
            boost::thread parse_thread(boost::bind(&CanbusDriver::parse_msg, this));
        }
        ros::spin();
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "autolabor_canbus_driver");
    autolabor_driver::CanbusDriver driver;
    driver.run();
    return 0;
}