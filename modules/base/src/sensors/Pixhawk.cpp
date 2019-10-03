//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#ifdef HAS_MAVSDK

#include <mico/base/sensors/Pixhawk.h>

#include <thread>

#include <iostream>


using namespace mavsdk;

namespace mico{
    void component_discovered(ComponentType component_type) {
        std::cout << "Discovered a component with type " << unsigned(component_type) << std::endl;
    }


    bool Pixhawk::init(std::unordered_map<std::string, std::string> _params){
        if(_params.find("connection") == _params.end())
            return false;

        bool discovered_system = false;
        ConnectionResult connection_result = dc.add_any_connection(_params["connection"]);
    
        if (connection_result != ConnectionResult::SUCCESS) {
            std::cout << "Connection failed: " << connection_result_str(connection_result) << std::endl;
            return false;
        }


        mavsdk::System &system = dc.system();
        // if(_params.find("uuid") == _params.end())
        //     system = dc.system(atoi(_params["uuid"].c_str()));
        // else
        //     system = dc.system();

        std::cout << "Waiting to discover system..." << std::endl;
        dc.register_on_discover([&discovered_system](uint64_t uuid) {
            std::cout << "Discovered system with UUID: " << uuid << std::endl;
            discovered_system = true;
        });

        // We usually receive heartbeats at 1Hz, therefore we should find a system after around 2
        // seconds.
        std::this_thread::sleep_for(std::chrono::seconds(2));

        if (!discovered_system) {
            std::cout << "No system found." << std::endl;
            return false;
        }

        // Register a callback so we get told when components (camera, gimbal) etc are found.
        system.register_component_discovered_callback(component_discovered);

        telemetry_ = std::make_shared<Telemetry>(system);
        auto action = std::make_shared<Action>(system);

        // We want to listen to the altitude of the drone at 1 Hz.
        float hz = 30.0;
        if(_params.find("hz") == _params.end())
            hz = atof(_params["hz"].c_str());

        const Telemetry::Result set_rate_result = telemetry_->set_rate_position(hz);
        if (set_rate_result != Telemetry::Result::SUCCESS) {
            std::cout << "Setting rate failed:" << Telemetry::result_str(set_rate_result)  << std::endl;
            return false;
        }
        return true;
    }


    Eigen::Vector3f Pixhawk::acceleration() const{
        auto pimu = telemetry_->imu_reading_ned();
        return Eigen::Vector3f( pimu.acceleration.north_m_s2, 
                                pimu.acceleration.east_m_s2, 
                                pimu.acceleration.down_m_s2);
    }

    Eigen::Vector3f Pixhawk::angularSpeed() const{
        auto ps = telemetry_->attitude_angular_velocity_body();
        return Eigen::Vector3f(ps.roll_rad_s, ps.pitch_rad_s, ps.yaw_rad_s);
    }

    Eigen::Quaternionf Pixhawk::orientation() const{
        auto pq = telemetry_->attitude_quaternion();
        return Eigen::Quaternionf(pq.w,pq.x,pq.y,pq.z);
    }

    Eigen::Vector3f Pixhawk::position() const{
        auto pq = telemetry_->position_velocity_ned();
        return Eigen::Vector3f(pq.position.north_m,pq.position.east_m, pq.position.down_m);
    }


}

#endif