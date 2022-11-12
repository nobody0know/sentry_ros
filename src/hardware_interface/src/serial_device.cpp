//
// Created by lxj on 22-11-6.
//

#include "../include/serial_device.h"
#define USING_COMMEND_LINE 1
namespace serial {
    serial_device::serial_device(string port_name, int baud_rate)
    :port_name_(port_name),
    baud_rate_(baud_rate),
    data_bits_(8),
    parity_bits_('N'),
    stop_bits_(1){
        init_serial_port();
    }

    bool serial_device::init_serial_port() {
        ROS_INFO("open device %s with baudrate %d",port_name_,baud_rate_);
        if (port_name_.c_str() == nullptr){
            port_name_ = "/dev/ttyUSB0";
        }
        if (open_device() && config_device()){
            FD_ZERO(&serial_fd_set_);
            FD_SET(serial_fd_,&serial_fd_set_);
            ROS_INFO("serial started successfully");
            return true;
        } else {
            ROS_ERROR("failed to start serial %s",port_name_);
            close_device();
            return false;
        }
    }

    bool serial_device::open_device() {
#ifdef __arm__
        serial_fd_= open(port_name_.c_str(), O_RDWR | O_NONBLOCK);
#elif __x86_64__
        serial_fd_= open(port_name_.c_str(), O_RDWR | O_NOCTTY);
#else
        serial_fd_= open(port_name_.c_str(), O_RDWR | O_NOCTTY);
#endif
        if (serial_fd_ < 0) {
            ROS_ERROR("cannot open device %d %d",serial_fd_,port_name_);
            return false;
        }
        return true;
    }

    bool serial_device::close_device() {
        close(serial_fd_);
        serial_fd_ = -1;
        return true;
    }

    bool serial_device::config_device() {
        int st_baud[] = {B4800, B9600, B19200, B38400,
                         B57600, B115200, B230400, B921600};
        int std_rate[] = {4800, 9600, 19200, 38400, 57600, 115200,
                          230400, 921600, 1000000, 1152000, 3000000};
        int i, j;
        /* save current port parameter */
        if (tcgetattr(serial_fd_, &old_termios_) != 0) {
            ROS_ERROR("fail to save current port");
            return false;
        }
        memset(&new_termios_, 0, sizeof(new_termios_));

        /* config the size of char */
        new_termios_.c_cflag |= CLOCAL | CREAD;
        new_termios_.c_cflag &= ~CSIZE;

        /* config data bit */
        switch (data_bits_) {
            case 7:new_termios_.c_cflag |= CS7;
                break;
            case 8:new_termios_.c_cflag |= CS8;
                break;
            default:new_termios_.c_cflag |= CS8;
                break; //8N1 default config
        }
        /* config the parity bit */
        switch (parity_bits_) {
            /* odd */
            case 'O':
            case 'o':
                new_termios_.c_cflag |= PARENB;
                new_termios_.c_cflag |= PARODD;
                break;
                /* even */
            case 'E':
            case 'e':
                new_termios_.c_cflag |= PARENB;
                new_termios_.c_cflag &= ~PARODD;
                break;
                /* none */
            case 'N':
            case 'n':
                new_termios_.c_cflag &= ~PARENB;
                break;
            default:
                new_termios_.c_cflag &= ~PARENB;
                break; //8N1 default config
        }
        /* config baudrate */
        j = sizeof(std_rate) / 4;
        for (i = 0; i < j; ++i) {
            if (std_rate[i] == baud_rate_) {
                /* set standard baudrate */
                cfsetispeed(&new_termios_, st_baud[i]);
                cfsetospeed(&new_termios_, st_baud[i]);
                break;
            }
        }
        /* config stop bit */
        if (stop_bits_ == 1)
            new_termios_.c_cflag &= ~CSTOPB;
        else if (stop_bits_ == 2)
            new_termios_.c_cflag |= CSTOPB;
        else
            new_termios_.c_cflag &= ~CSTOPB; //8N1 default config

        /* Set input parity option */
        if (parity_bits_ != 'n')
            new_termios_.c_iflag |= INPCK;

        tcflush(serial_fd_, TCIFLUSH); //清除输入缓存区

/* config waiting time & min number of char */
        new_termios_.c_cc[VTIME] = 150;
        new_termios_.c_cc[VMIN] = 0;

        /* using the raw data mode */
        new_termios_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        new_termios_.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
        new_termios_.c_iflag &= ~(ICRNL | IGNCR);
        new_termios_.c_oflag &= ~OPOST;

        /* flush the hardware fifo */
        tcflush(serial_fd_, TCIFLUSH);

        /* activite the configuration */
        if ((tcsetattr(serial_fd_, TCSANOW, &new_termios_)) != 0) {
            ROS_ERROR("failed to activate serial configuration");
            return false;
        }
        return true;
    }

    int serial_device::send(const uint8_t *buf,int len) {
        return write(serial_fd_,buf,len);
    }

    int serial_device::Read(uint8_t *buf,int len) {
        int ret = -1;
          if (NULL == buf) {
            return -1;
          } else {
            ret = read(serial_fd_, buf, len);
                ROS_INFO("Read once length: %d",ret);
            while (ret == 0) {
                ROS_WARN("Connection closed, try to reconnect");
              while (!init_serial_port()) {
                usleep(500000);
              }
                ROS_INFO("Reconnect Success");
              ret = read(serial_fd_, buf, len);
            }
            return ret;
          }
    }

    void serial_device::transformData(const sentry_control &data) {
#if USING_COMMEND_LINE
        char buffer[255];
        buffer[0] = 0xA5;
        sprintf(&buffer[1],"setGimbalAngle -y%f -p%f\n",data.yaw_angle.float_d,data.pitch_angle.float_d);
        write(serial_fd_,buffer,strlen(buffer)+1);
        memset(buffer,0,sizeof(buffer));
        buffer[0] = 0xA5;
        sprintf(&buffer[1],"setChassisVelocity -vx%f -vy%f -vw%f\n",data.chassis_vx.float_d,data.chassis_vy.float_d,data.chassis_vw.float_d);
        write(serial_fd_,buffer, strlen(buffer)+1);
        memset(buffer,0,sizeof(buffer));
#else
        unsigned char buffer[42];
        buffer[0] = 0xA5;
        Append_CRC8_Check_Sum(buffer,2);
        buffer[2] = data.yaw_angle.char_d[0];
        buffer[3] = data.yaw_angle.char_d[1];
        buffer[4] = data.yaw_angle.char_d[2];
        buffer[5] = data.yaw_angle.char_d[3];

        buffer[6] = data.pitch_angle.char_d[0];
        buffer[7] = data.pitch_angle.char_d[1];
        buffer[8] = data.pitch_angle.char_d[2];
        buffer[9] = data.pitch_angle.char_d[3];

        buffer[10] = data.chassis_vx.char_d[0];
        buffer[11] = data.chassis_vx.char_d[1];
        buffer[12] = data.chassis_vx.char_d[2];
        buffer[13] = data.chassis_vx.char_d[3];

        buffer[14] = data.chassis_vy.char_d[0];
        buffer[15] = data.chassis_vy.char_d[1];
        buffer[16] = data.chassis_vy.char_d[2];
        buffer[17] = data.chassis_vy.char_d[3];

        buffer[18] = data.chassis_vw.char_d[0];
        buffer[19] = data.chassis_vw.char_d[1];
        buffer[20] = data.chassis_vw.char_d[2];
        buffer[21] = data.chassis_vw.char_d[3];

        Append_CRC16_Check_Sum(buffer,24);
        write(serial_fd_,buffer, 24);
        memset(buffer,0,sizeof(buffer));

#endif
    }

} // serial