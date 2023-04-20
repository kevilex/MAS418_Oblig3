#include <chrono>
#include <memory>
#include <iostream>
#include <fstream>

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "rclcpp/rclcpp.hpp"                                    

#include <thread>
#include "AdsLib.h"
#include "AdsVariable.h"
using namespace std::chrono_literals;

namespace craneads {

    struct AdsVariables
    {
        AdsVariables() = delete;

        explicit AdsVariables(AdsDevice& route)
            : activateMotion{route, "MAIN.bActivateMotion"}
            , velocityReference{route, "MAIN.fVelRef"}
            , positionReference{route, "MAIN.fPosRef"}
            , positionMeasurement{route, "MAIN.fPosMeas"}
            , PistonPressure{route, "MAIN.PistonPressure"}
            , RodSidePressure{route, "MAIN.RodSidePressure"}
        {
            // Do nothing.
        }

        AdsVariable<bool> activateMotion;
        AdsVariable<double> velocityReference;
        AdsVariable<double> positionReference;
        AdsVariable<double> positionMeasurement;
        AdsVariable<double> PistonPressure;
        AdsVariable<double> RodSidePressure;
        
    };

    class AdsHandler
    {
    public:
        explicit AdsHandler(const AmsNetId remoteNetId, const std::string remoteIpV4)
            : remoteNetId_(remoteNetId)
            , remoteIpV4_(remoteIpV4)
            , route_{remoteIpV4_, remoteNetId_, AMSPORT_R0_PLC_TC3}
            , ads_(route_) { }

        AdsHandler() : AdsHandler({127, 0, 0, 1,  1, 1}, "127.0.0.1") { }


        void activateMotion()
        {
            ads_.activateMotion = true;
        }

        void deactivateMotion()
        {
            ads_.activateMotion = false;
        }

        void setVelocityReference(double value)
        {
            ads_.velocityReference = value;
        }

        void setPositionReference(double value)
        {
            ads_.positionReference = value;
        }

        double getPositionMeasurement()
        {
            return ads_.positionMeasurement;
        }

        double getPistonPressure()
        {
            return ads_.PistonPressure;
        }

        double getRodSidePressure()
        {
            return ads_.RodSidePressure;
        }


        void printState()
        {
            const auto state = route_.GetState();
            std::cout << "ADS state: "
                      << std::dec << static_cast<uint16_t>(state.ads)
                      << " devState: "
                      << std::dec << static_cast<uint16_t>(state.device);
        }

        ~AdsHandler() { }

    private:
        const AmsNetId remoteNetId_;
        const std::string remoteIpV4_;
        AdsDevice route_;
        AdsVariables ads_;
    };

}

class WaveMaker{
  public:
    float gen(){
      time = time + 0.5;
      return (sin(time)*sin(time)*5)+15;
    }

  private:
    float time = 0.0;
};
//instantiating the wavemaker object
WaveMaker waveMaker;


auto adsDelivery(float* information){
  const AmsNetId remoteNetId { 192, 168, 0, 10, 1, 1 };
  const std::string remoteIpV4 = "192.168.0.10";


  craneads::AdsHandler adsHandler(remoteNetId, remoteIpV4);

  adsHandler.deactivateMotion();

  adsHandler.printState();

  adsHandler.activateMotion();


  adsHandler.setPositionReference(static_cast<double>(waveMaker.gen()));

  std::this_thread::sleep_for (std::chrono::seconds(1));
  information[0] = (adsHandler.getPositionMeasurement()/180)*3.141592;
  information[1] = adsHandler.getPistonPressure();
  information[2] = adsHandler.getRodSidePressure();

  
  return;
};

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("Crane_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 200);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&MinimalPublisher::timer_callback, this));

    publisher2_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("pressures", 200);
    timer2_ = this->create_wall_timer(
		10ms, std::bind(&MinimalPublisher::pressure_callback, this));
  }


private:

  void pressure_callback(){
    float information[3] = {0.0, 0.0, 0.0};
    adsDelivery(information);
    pressure.data.resize(2);
    pressure.data[0] = information[1];
    pressure.data[1] = information[2];
    publisher2_->publish(pressure);

  }

  void timer_callback(){
    float information[3] = {0.0, 0.0, 0.0};
    adsDelivery(information);
    auto message = sensor_msgs::msg::JointState();  

    //filling the message
    message.header.stamp = this->get_clock()->now();
    message.name.push_back("base_to_crane_boom");

    message.position.push_back(-information[0]);

    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.position[0] << "'");   
    publisher_->publish(message);

  }
  
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher2_;

  std_msgs::msg::Float32MultiArray pressure;        
  size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
