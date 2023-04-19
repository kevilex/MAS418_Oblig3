#include <iostream>
#include <string>
#include <fstream>

#include <thread>
#include <chrono>
#include "AdsLib.h"
#include "AdsVariable.h"

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


double position = 0;
int main(int /* argc */, char** /*argv*/)
{
  std::cout << "Example ROS2 ADS node starting up.." << std::endl;

  // Real lab PLC IP.
  const AmsNetId remoteNetId { 192, 168, 0, 10, 1, 1 };
  const std::string remoteIpV4 = "192.168.0.10";


  std::cout << "  Create AdsHandler.. ";
  craneads::AdsHandler adsHandler(remoteNetId, remoteIpV4);
  std::cout << "  OK" << std::endl;

  adsHandler.deactivateMotion();

  adsHandler.printState();

  adsHandler.setVelocityReference(0);
  std::this_thread::sleep_for (std::chrono::seconds(5));
  adsHandler.setPositionReference(3.14);

  adsHandler.activateMotion();
  std::ofstream posfile;

  for(uint8_t n = 0; ; ++n)
  {
    adsHandler.setPositionReference(static_cast<double>(n));
    std::cout << "Position measurement from ADS: " << adsHandler.getPositionMeasurement() << std::endl;
    std::cout << "Piston pressure measurement from ADS: " << adsHandler.getPistonPressure() << std::endl;
    std::cout << "Rod side pressure measurement from ADS: " << adsHandler.getRodSidePressure() << std::endl;

    std::this_thread::sleep_for (std::chrono::seconds(2));
  }


  return 0;
}
