#pragma once

#include <thread_exo.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated"
#include <declarations_xsens.h>
#pragma GCC diagnostic pop
#include <LowPassFilter2p.h>

#include <utils_plot.hpp>




typedef struct{
    float imus_data[18]; 
} fromXsensRead;

typedef struct{
     
} XsensReadHandler;

class ThreadXsensRead: public ThreadType{
    private:
        fromXsensRead data;

    public:       
      
        std::atomic<bool> plotting;
        
        PlotWindow pw;

        ThreadXsensRead(){
            pw = PlotWindow("Units [ u ]","Time [ s ]","Xsens Data");
            plotting = false;
        }

        XsDevicePtr wirelessMasterDevice = NULL;
        XsControl *control = NULL;
        XsPortInfoArray detectedDevices;
        XsPortInfoArray::const_iterator wirelessMasterPort;
        WirelessMasterCallback wirelessMasterCallback;
        XsDevicePtrArray mtwDevices;
        
        LowPassFilter2pFloat imu_filters[18];
        std::vector<MtwCallback *> mtwCallbacks; 
        std::vector<XsEuler>  eulerData;
        std::vector<XsVector> accData;
        std::vector<XsVector> gyroData;
        std::vector<XsVector> magData;

        const int desiredUpdateRate = 100;
        const int desiredRadioChannel = 25;

        void _setup(){
            // throw "ERROR";
            using namespace std;
            vector<int> imu_headers(4);
            
            

            mtwCallbacks.clear();

            control = XsControl::construct();
            if (control == 0) throw "Failed to construct XsControl instance.";

            try{      
                detectedDevices = XsScanner::scanPorts();
                wirelessMasterPort = detectedDevices.begin();
                while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster()){
                    ++wirelessMasterPort;
                    cout << "Wireless master found @ " << *wirelessMasterPort << endl;
                }
                if (wirelessMasterPort == detectedDevices.end()){
                    throw runtime_error("No wireless masters found");
                }
                cout << "Wireless master found @ " << *wirelessMasterPort << endl;

                if (!control->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate())){
                    ostringstream error;
                    error << "Failed to open port " << *wirelessMasterPort;
                    throw runtime_error(error.str());
                } 

                wirelessMasterDevice = control->device(wirelessMasterPort->deviceId());
                if (wirelessMasterDevice == 0){
                    ostringstream error;
                    error << "Failed to construct XsDevice instance: " << *wirelessMasterPort;
                    throw runtime_error(error.str());
                }
                if (!wirelessMasterDevice->gotoConfig()){
                    ostringstream error;
                    error << "Failed to goto config mode: " << *wirelessMasterDevice;
                    throw runtime_error(error.str());
                }
                wirelessMasterDevice->addCallbackHandler(&wirelessMasterCallback);

                const XsIntArray supportedUpdateRates = wirelessMasterDevice->supportedUpdateRates();
                const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate);

                if (!wirelessMasterDevice->setUpdateRate(newUpdateRate)){
                    ostringstream error;
                    error << "Failed to set update rate: " << *wirelessMasterDevice;
                    throw runtime_error(error.str());
                }
                if (wirelessMasterDevice->isRadioEnabled()){
                    if (!wirelessMasterDevice->disableRadio()){
                        ostringstream error;
                        error << "Failed to disable radio channel: " << *wirelessMasterDevice;
                        throw runtime_error(error.str());
                    }
                }
                if (!wirelessMasterDevice->enableRadio(desiredRadioChannel)){
                    ostringstream error;
                    error << "Failed to set radio channel: " << *wirelessMasterDevice;
                    throw runtime_error(error.str());
                }

                cout << "Waiting for MTW to wirelessly connect..." << endl;
                size_t connectedMTWCount = wirelessMasterCallback.getWirelessMTWs().size();
                bool quitOnMTw = false;
                bool waitForConnections = true;
                int countIMUs = 0;
                for(int countIT = 0 ; countIT < 300 ; countIT++){
                
                    XsTime::msleep(100);

                    while (true)
                    {
                        size_t nextCount = wirelessMasterCallback.getWirelessMTWs().size();
                        if (nextCount != connectedMTWCount){
                            cout << "Number of connected MTWs: " << nextCount <<endl;
                            connectedMTWCount = nextCount;
                            countIMUs ++;
                        }else break;
                    }

                    if(countIMUs >= 3 ) break;
                };
                
                if(countIMUs<3) throw (std::runtime_error("Faltan IMUs"));

                

            }catch (std::exception const &ex){
                std::cout << ex.what() << std::endl;
                std::cout << "****ABORT****" << std::endl;
                throw (std::runtime_error(ex.what()));
            }catch (...){
                std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
                std::cout << "****ABORT****" << std::endl;
                throw "****ABORT IMU 2****";
            }

            {
                std::unique_lock<std::mutex> _(_mtx);
                pw.SCOPE.y_axis.minimum = -20.0f;
                pw.SCOPE.y_axis.maximum = 20.0f;
                pw.SCOPE.x_axis.minimum = 0.0f;
                pw.SCOPE.x_axis.maximum = T_exec;
                pw.clearItems();
                pw.addItem("acc 1 x ");
                pw.addItem("acc 1 y ");
                pw.addItem("acc 1 z ");
            }

            
        }

        void _cleanup(){
            printf("\nSaliendo de las IMUS");
            try{
                
                if (!wirelessMasterDevice->gotoConfig()){
                    std::ostringstream error;
                    error << "Failed to goto config mode: " << *wirelessMasterDevice;
                    throw std::runtime_error(error.str());
                }
                if (!wirelessMasterDevice->disableRadio()){
                    std::ostringstream error;
                    error << "Failed to disable radio: " << *wirelessMasterDevice;
                    throw std::runtime_error(error.str());
                }
                control->closePort(wirelessMasterPort->portName().toStdString());
            }
            catch (std::exception const& ex) { GUI.addLOG(ex.what()); std::cout << "Error CleanUp Xsens... " << std::endl; throw "****ABORT IMU 2****";
            }
            catch(...){std::cout << "Error CleanUp Xsens... " << std::endl; throw "****ABORT IMU 2****";
            }
        }

        void _firstLoop() {
            using namespace std;

            if (!wirelessMasterDevice->gotoMeasurement()) {
                ostringstream error;
                error << "Failed to goto measurement mode: " << *wirelessMasterDevice;
                throw runtime_error(error.str());
            }

            XsDeviceIdArray allDeviceIds = control->deviceIds();
            XsDeviceIdArray mtwDeviceIds;
            for (XsDeviceIdArray::const_iterator i = allDeviceIds.begin(); i != allDeviceIds.end(); ++i) {
                if (i->isMtw()) {
                    mtwDeviceIds.push_back(*i);
                }
            }
            mtwDevices.clear();
            for (XsDeviceIdArray::const_iterator i = mtwDeviceIds.begin(); i != mtwDeviceIds.end(); ++i) {
                XsDevicePtr mtwDevice = control->device(*i);
                if (mtwDevice != 0) {
                    mtwDevices.push_back(mtwDevice);
                }
                else {
                    throw runtime_error("Failed to create an MTW XsDevice instance");
                }
            }


            for (int i = 0; i < sizeof(imu_filters) / sizeof(LowPassFilter2pFloat); i++) {
                imu_filters[i].set_cutoff_frequency(desiredUpdateRate, 16);
                imu_filters[i].reset();
            }

            mtwCallbacks.resize(mtwDevices.size());
            for (int i = 0; i < (int)mtwDevices.size(); ++i) {
                mtwCallbacks[i] = new MtwCallback(i, mtwDevices[i]);
                mtwDevices[i]->addCallbackHandler(mtwCallbacks[i]);
                string imu_id = mtwDevices[i]->deviceId().toString().toStdString();
                if (i == 0)
                    cout << "IMU Usuario Coxa: " << imu_id << "\n";
                if (i == 1)
                    cout << "IMU Usuario Canela: " << imu_id << "\n";
                if (i == 2)
                    cout << "IMU Exo: " << imu_id << "\n";
            }

            eulerData = std::vector<XsEuler>(mtwCallbacks.size());
            accData = std::vector<XsVector>(mtwCallbacks.size());
            gyroData = std::vector<XsVector>(mtwCallbacks.size());
            magData = std::vector<XsVector>(mtwCallbacks.size());

        }

        void _loop(){
            
        
            bool newDataAvailable = false;

            for (size_t i = 0; i < mtwCallbacks.size(); ++i){

                if (mtwCallbacks[i]->dataAvailable()){
                    newDataAvailable = true;
                    XsDataPacket const *packet = mtwCallbacks[i]->getOldestPacket();

                    eulerData[i] = packet->orientationEuler();
                    accData[i]   = packet->calibratedAcceleration();
                    gyroData[i]  = packet->calibratedGyroscopeData();
                    magData[i]   = packet->calibratedMagneticField();

                    mtwCallbacks[i]->deleteOldestPacket();

                    {
                        std::unique_lock<std::mutex> _(_mtx);

                        if (i == 0 || i == 1){
                            data.imus_data[6*i+0] = imu_filters[6*i+0].apply( gyroData[i].value(2) );
                            data.imus_data[6*i+1] = imu_filters[6*i+1].apply(-gyroData[i].value(1) );
                            data.imus_data[6*i+2] = imu_filters[6*i+2].apply( gyroData[i].value(0) );
                            data.imus_data[6*i+3] = imu_filters[6*i+3].apply(  accData[i].value(2) );
                            data.imus_data[6*i+4] = imu_filters[6*i+4].apply( -accData[i].value(1) );
                            data.imus_data[6*i+5] = imu_filters[6*i+5].apply(  accData[i].value(0) );
                        }
                        if (i == 2){
                            data.imus_data[6*i+0] = imu_filters[6*i+0].apply( gyroData[i].value(2) );
                            data.imus_data[6*i+1] = imu_filters[6*i+1].apply(-gyroData[i].value(1) );
                            data.imus_data[6*i+2] = imu_filters[6*i+2].apply( gyroData[i].value(0) );
                            data.imus_data[6*i+3] = imu_filters[6*i+3].apply(  accData[i].value(2) );
                            data.imus_data[6*i+4] = imu_filters[6*i+4].apply( -accData[i].value(1) );
                            data.imus_data[6*i+5] = imu_filters[6*i+5].apply(  accData[i].value(0) );
                        }
                        // std::unique_lock<std::mutex> _(_mtx);
                        // memcpy(imu_shared_data, imus_data, sizeof(imus_data));
                        _datalog[time_index][0] = timer->get_current_time_f();
                        for(int idx  = 0 ; idx < 18 ; idx++){
                            _datalog[time_index][idx +1] = data.imus_data[idx];
                        }
                        
                        
                    }


                }
            }

            if (true) {
                {
                    std::unique_lock<std::mutex> _(_mtx);
                    pw.items[0].data.push_back(ImVec2((float)_datalog[time_index][0], (float)_datalog[time_index][1]));
                    pw.items[1].data.push_back(ImVec2((float)_datalog[time_index][0], (float)_datalog[time_index][2]));
                    pw.items[2].data.push_back(ImVec2((float)_datalog[time_index][0], (float)_datalog[time_index][3]));
                }                
            }


            //_mtx.lock();    
            //_mtx.unlock();
        }

        bool cbPlot = false;
        void _updateGUI() {
            ImGui::Begin(_name.c_str());
            
            ImGui::Checkbox("Graph",&cbPlot);
            plotting = cbPlot;
            if (plotting) {
                {
                    std::unique_lock<std::mutex> _(_mtx);
                    pw.show();
                }
            }

            ImGui::End();
        }


        void getData(void * _data){
            {
                std::unique_lock<std::mutex> _(_mtx);
                memcpy(_data,&data,sizeof(fromXsensRead));
            }
        }
};
