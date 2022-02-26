#pragma once

#include <thread_exo.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated"
#include <declarations_xsens.h>
#pragma GCC diagnostic pop
#include <LowPassFilter2p.h>
#include <xsensLeoUtils.hpp>

#include <Eigen/Core>
#include <Eigen/LU>

#include <utils_plot.hpp>


typedef struct{
    float data[10];
} fromXsensLeo;

class ThreadXsensLeo: public ThreadType{
        
    private:
        fromXsensLeo data;
        std::atomic<bool> plotting;
        PlotWindow pw;

    public:       
        
        ThreadXsensLeo(){
            pw = PlotWindow("Units [ u ]", "Time [ s ]", "Xsens Data");
            plotting = false;
        }

        float omega_ant;
        float q0;
        float q1;
        float q2;
        float q3;
        float imus_data[18];
        float states_data[10];
        float Ts = _Ts;
        const float mi0 = 0.100;
        const float Beta = 1.1400;
        const float Rho = 0.00017;
        const Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
        const Eigen::Matrix4f R = Eigen::Matrix4f::Identity() * 1e-5;

        Eigen::Vector4f qASGD1_qk;
        Eigen::Vector4f qASGD2_qk;
        Eigen::Matrix4f qASGD1_Pk = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f qASGD2_Pk = Eigen::Matrix4f::Identity();
        Eigen::FullPivLU<Eigen::Matrix4f> Covar1;
        Eigen::FullPivLU<Eigen::Matrix4f> Covar2;
        Eigen::Matrix4f Q1 = Eigen::Matrix4f::Identity() * 5.476e-6; // Usar Eq. 19...
        Eigen::Matrix4f Q2 = Q1;

        Eigen::Vector3f gyro1;
        Eigen::Vector3f gyro2;
        Eigen::Vector3f acc1;
        Eigen::Vector3f acc2;
        Eigen::Vector3f F_obj;
        Eigen::Vector4f GradF;
        Eigen::Vector4f z_k;
        Eigen::Vector3f Zc;
        Eigen::Matrix4f OmG;
        Eigen::Matrix4f Psi;
        Eigen::Matrix4f Kg1;
        Eigen::Matrix4f Kg2;
        Eigen::Matrix4f Qy;
        Eigen::Matrix<float, 3, 4> Jq;
        Eigen::Matrix<float, 4, 3> Xi;
        float omg_norm;
        float mi;


        void _setup(){

            ((ThreadXsensRead*)XsensRead->threadType_)->sensorsSxens[0].required = true;
            ((ThreadXsensRead*)XsensRead->threadType_)->sensorsSxens[1].required = true;
            ((ThreadXsensRead*)XsensRead->threadType_)->sensorsSxens[2].required = true;
            ((ThreadXsensRead*)XsensRead->threadType_)->sensorsSxens[3].required = true;
            
            using namespace std;
            using namespace Eigen;

            if(XsensRead->toRUNfromGUI == false )
                 throw std::runtime_error("Precisa iniciar o thread XsensRead");
            
            omega_ant = 0;
            q0 = 0;
            q1 = 0;
            q2 = 0;
            q3 = 0;
            imus_data[18]  = {0};
            states_data[10]  = {0};
            Ts = _Ts;

            qASGD1_qk = Vector4f(1.0f, 0.0f, 0.0f, 0.0f);
            qASGD2_qk = Vector4f(1.0f, 0.0f, 0.0f, 0.0f);
            qASGD1_Pk = Matrix4f::Identity();
            qASGD2_Pk = Matrix4f::Identity();

            Q1 = Matrix4f::Identity() * 5.476e-6; // Usar Eq. 19...
            Q2 = Q1;

            gyro1 = Vector3f (0.0f, 0.0f, 0.0f);
            gyro2 = Vector3f (0.0f, 0.0f, 0.0f);
            acc1 = Vector3f (0.0f, 0.0f, 0.0f);
            acc2 = Vector3f (0.0f, 0.0f, 0.0f);
            F_obj = Vector3f();
            GradF = Vector4f();
            z_k = Vector4f();
            Zc = Vector3f();
            OmG = Matrix4f();
            Psi = Matrix4f();
            Kg1 = Matrix4f();
            Kg2 = Matrix4f();
            Qy = Matrix4f();
            Jq = Matrix<float, 3, 4>();
            Xi = Matrix<float, 4, 3>();
            omg_norm = gyro1.norm();
            mi = 0;

            {
                std::unique_lock<std::mutex> _(_mtx);
                pw.SCOPE.y_axis.minimum = -3.14f;
                pw.SCOPE.y_axis.maximum = 3.14f;
                pw.SCOPE.x_axis.minimum = 0.0f;
                pw.SCOPE.x_axis.maximum = T_exec;
                pw.clearItems();
                pw.addItem("omega_p");
                pw.addItem("theta_p");                
                pw.addItem("alpha_p");
            }
        }

        void _cleanup(){
            
        }
        fromXsensRead imu_shared_data;
        void _loop(){
            
            using namespace std;
            using namespace Eigen;
        
            XsensRead->getData(&imu_shared_data);
            memcpy(imus_data, imu_shared_data.imus_data, sizeof(imus_data));
            
            // printf("\n");
            // for(int i = 0 ; i < 18 ; i++){
            //     printf("\t %f",imus_data[i]);
            // }

            gyro1 << imus_data[0], imus_data[1], imus_data[2];
            acc1 << imus_data[3], imus_data[4], imus_data[5];
            gyro2 << imus_data[6], imus_data[7], imus_data[8];
            acc2 << imus_data[9], imus_data[10], imus_data[11];

            q0 = qASGD1_qk(0);
            q1 = qASGD1_qk(1);
            q2 = qASGD1_qk(2);
            q3 = qASGD1_qk(3);

            Zc << 2 * (q1 * q3 - q0 * q2),
            2 * (q2 * q3 + q0 * q1),
            (q0 * q0 - q1 * q1 - q2 * q2 - q3 * q3);
            F_obj = Zc - acc1.normalized(); // Eq.23

            Jq << -2 * q2, 2 * q3, -2 * q0, 2 * q1,
            2 * q1, 2 * q0, 2 * q3, 2 * q2,
            2 * q0, -2 * q1, -2 * q2, 2 * q3;

            GradF = Jq.transpose() * F_obj; // Eq.25

            omg_norm = gyro1.norm();
            mi = mi0 + Beta * Ts * omg_norm; // Eq.29

            z_k = qASGD1_qk - mi * GradF.normalized(); // Eq.24
            z_k.normalize();

            OmG << 0, -gyro1(0), -gyro1(1), -gyro1(2),
            gyro1(0), 0, gyro1(2), -gyro1(1),
            gyro1(1), -gyro1(2), 0, gyro1(0),
            gyro1(2), gyro1(1), -gyro1(0), 0;
            OmG = 0.5 * OmG;

            Psi = (1 - ((omg_norm * Ts) * (omg_norm * Ts)) / 8) * Matrix4f::Identity() + 0.5 * Ts * OmG;

            // Process noise covariance update (Eq. 19):
            Xi << q0, q3, -q2, -q3, q0, q1, q2, -q1, q0, -q1, -q2, -q3;
            Q1 = 0.5 * Ts * Xi * (Matrix3f::Identity() * 5.476e-6) * Xi.transpose();
            // Projection:
            qASGD1_qk = Psi * qASGD1_qk;
            qASGD1_Pk = Psi * qASGD1_Pk * Psi.transpose() + Q1;
            // Kalman Gain (H is Identity)
            Covar1 = FullPivLU<Matrix4f>(qASGD1_Pk + R);
            if (Covar1.isInvertible())
            Kg1 = qASGD1_Pk * Covar1.inverse();
            // Update (H is Identity)
            qASGD1_qk = qASGD1_qk + Kg1 * (z_k - qASGD1_qk);
            qASGD1_Pk = (Matrix4f::Identity() - Kg1) * qASGD1_Pk;
            qASGD1_qk.normalize();

            // Rotate the quaternion by a quaternion with -(yaw):
            removeYaw(&qASGD1_qk);


            q0 = qASGD2_qk(0);
            q1 = qASGD2_qk(1);
            q2 = qASGD2_qk(2);
            q3 = qASGD2_qk(3);

            Zc << 2 * (q1 * q3 - q0 * q2),
            2 * (q2 * q3 + q0 * q1),
            (q0 * q0 - q1 * q1 - q2 * q2 - q3 * q3);
            F_obj = Zc - acc2.normalized(); // Eq.23

            Jq << -2 * q2, 2 * q3, -2 * q0, 2 * q1,
            2 * q1, 2 * q0, 2 * q3, 2 * q2,
            2 * q0, -2 * q1, -2 * q2, 2 * q3;

            GradF = Jq.transpose() * F_obj; // Eq.25

            omg_norm = gyro2.norm();
            mi = mi0 + Beta * Ts * omg_norm; // Eq.29

            z_k = qASGD2_qk - mi * GradF.normalized(); // Eq.24
            z_k.normalize();

            OmG << 0, -gyro2(0), -gyro2(1), -gyro2(2),
            gyro2(0), 0, gyro2(2), -gyro2(1),
            gyro2(1), -gyro2(2), 0, gyro1(0),
            gyro2(2), gyro2(1), -gyro2(0), 0;
            OmG = 0.5 * OmG;

            Psi = (1 - ((omg_norm * Ts) * (omg_norm * Ts)) / 8) * Matrix4f::Identity() + 0.5 * Ts * OmG;

            // Process noise covariance update (Eq. 19):
            Xi << q0, q3, -q2, -q3, q0, q1, q2, -q1, q0, -q1, -q2, -q3;
            Q2 = 0.5 * Ts * Xi * (Matrix3f::Identity() * 5.476e-6) * Xi.transpose();
            // Projection:
            qASGD2_qk = Psi * qASGD2_qk;
            qASGD2_Pk = Psi * qASGD2_Pk * Psi.transpose() + Q2;
            // Kalman Gain (H is Identity)
            Covar2 = FullPivLU<Matrix4f>(qASGD2_Pk + R);
            if (Covar2.isInvertible())
            Kg2 = qASGD2_Pk * Covar2.inverse();
            // Update (H is Identity)
            qASGD2_qk = qASGD2_qk + Kg2 * (z_k - qASGD2_qk);
            qASGD2_Pk = (Matrix4f::Identity() - Kg2) * qASGD2_Pk;
            qASGD2_qk.normalize();

            // Rotate the quaternion by a quaternion with -(yaw):
            removeYaw(&qASGD2_qk);

            // Euler angles between IMUs:
            Vector3f euler_ned = quatDelta2Euler(&qASGD2_qk, &qASGD1_qk);
            // Relative Omega between IMUs:
            Vector3f omega_ned = RelOmegaNED(&qASGD1_qk, &qASGD2_qk, &gyro1, &gyro2);

            states_data[0] = euler_ned(0);    // ang_knee_x
            states_data[1] = euler_ned(1);    // ang_knee_y
            states_data[2] = euler_ned(2);    // ang_knee_z
            states_data[3] = omega_ned(0);    // omg_knee_x
            states_data[4] = omega_ned(1);    // omg_knee_y
            states_data[5] = omega_ned(2);    // omg_knee_z
            states_data[6] = imus_data[12];   // gyro_exo_x
            states_data[7] = imus_data[13];   // gyro_exo_y
            states_data[8] = imus_data[14];   // gyro_exo_z

            { // sessao critica
                unique_lock<mutex> _(_mtx);
                memcpy(data.data, states_data, sizeof(states_data));
            } // fim da sessao critica

            _datalog[time_index][0] = timer->get_current_time_f();
            for(int i  = 0 ; i < 10 ; i++){
                _datalog[time_index][i+1] = states_data[i];
            }

            if (true) {
                {
                    std::unique_lock<std::mutex> _(_mtx);
                    pw.items[0].data.push_back(ImVec2((float)_datalog[time_index][0], (float)_datalog[time_index][1]));
                    pw.items[1].data.push_back(ImVec2((float)_datalog[time_index][0], (float)_datalog[time_index][4]));
                    /*pw.items[2].data.push_back(ImVec2((float)_datalog[time_index][0], (float)(omega_ant - _datalog[time_index][1] ) / _Ts));
                    omega_ant = _datalog[time_index][1];*/
                }
            }
        }

        bool cbPlot = false;
        void _updateGUI() {
            ImGui::Begin(_name.c_str());

            ImGui::Checkbox("Graph", &cbPlot);
            plotting = cbPlot;
            if (plotting) {
                {
                    std::unique_lock<std::mutex> _(_mtx);
                    pw.showNewWindow();
                }
            }

            ImGui::End();
        }

        void getData(void * _data){
            _mtx.lock();
                memcpy(_data,&data,sizeof(fromXsensLeo));
            _mtx.unlock();
        }
};
