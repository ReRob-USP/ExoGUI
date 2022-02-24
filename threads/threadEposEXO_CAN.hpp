#pragma once


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"
#include <AXIS.h>
#include <EPOS_NETWORK.h>
#pragma GCC diagnostic pop

#include <imgui.h>

#include <thread_exo.h>
#include <string>
#include <thread>
#include <mutex>

#include <atomic>

#include <chrono>
using namespace std::chrono;

#define ID_ZERO  0

typedef struct{

    long  getPosm;
    long  getPosl;
    long  setVelm;
 
} fromEposEXO_CAN;

class ThreadEposEXO_CAN: public ThreadType{
    private:
        
        fromEposEXO_CAN data;

    public:       

        std::atomic<bool> _running_aux;
        std::thread _aux_thread;

        char *CAN_INTERFACE = (char *)"CAN1";
        char *CAN_DATABASE = (char *)"database";
        char *CAN_CLUSTER = (char *)"NETCAN";
        char* NET_ID_SERVO_01 = (char*)"1";
        char* NET_ID_SENSOR_01 = (char*)"2";
        char* NET_ID_SERVO_02 = (char *)"3";
        char* NET_ID_SERVO_03 = (char *)"4";
        char* NET_ID_SERVO_04 = (char *)"5";

        EPOS_NETWORK epos;

        AXIS servo_hip_right    ;
        AXIS servo_hip_left     ;
        AXIS servo_knee_right   ;
        AXIS encoder_knee_right ;
        AXIS servo_knee_left    ;
        
        int ZERO_SENSOR_KNEE_right = 0;
        int ZERO_SERVO_KNEE_right = 0;

        int ZERO_SENSOR_KNEE_left = 0;
        int ZERO_SERVO_KNEE_left = 0;

        int ZERO_SENSOR_HIP_right = 0;
        int ZERO_SERVO_HIP_right = 0;

        int ZERO_SENSOR_HIP_left = 0;
        int ZERO_SERVO_HIP_left = 0;

        long endwait;

        void start_transmissao_rede_epos()
        {
            epos.StartPDOS(1);
            epos.StartPDOS(2);
            epos.StartPDOS(3);
            epos.StartPDOS(4);
            epos.StartPDOS(5);
            epos.StartPDOS(1);
            epos.StartPDOS(2);
            epos.StartPDOS(3);
            epos.StartPDOS(4);
            epos.StartPDOS(5);

        }

        void init_comm_eixos()
        {
            _running_aux = true;
            loop_timers lt(1);
            for (int i = 0; i < 10; i++)
            {

                lt.start_timer();

                //Sincroniza as epos
                epos.sync();
                encoder_knee_right.ReadPDO01();
                servo_knee_right.ReadPDO01();

                //        encoder_knee_left.ReadPDO01();
                servo_knee_left.ReadPDO01();

                servo_hip_right.ReadPDO01();
                servo_hip_left.ReadPDO01();

                printf(".");
                lt.wait_final_time();
            }
            _running_aux = false;
        }

        void Habilita_Eixo(int ID)
        {
            _running_aux = true;
            if ((ID == 2) | (ID == 0))
            {

                servo_hip_right.PDOsetControlWord_SwitchOn(false);
                servo_hip_right.PDOsetControlWord_EnableVoltage(true);
                servo_hip_right.PDOsetControlWord_QuickStop(true);
                servo_hip_right.PDOsetControlWord_EnableOperation(false);
                servo_hip_right.WritePDO01();

                servo_hip_left.PDOsetControlWord_SwitchOn(false);
                servo_hip_left.PDOsetControlWord_EnableVoltage(true);
                servo_hip_left.PDOsetControlWord_QuickStop(true);
                servo_hip_left.PDOsetControlWord_EnableOperation(false);
                servo_hip_left.WritePDO01();

                servo_knee_right.PDOsetControlWord_SwitchOn(false);
                servo_knee_right.PDOsetControlWord_EnableVoltage(true);
                servo_knee_right.PDOsetControlWord_QuickStop(true);
                servo_knee_right.PDOsetControlWord_EnableOperation(false);
                servo_knee_right.WritePDO01();

                servo_knee_left.PDOsetControlWord_SwitchOn(false);
                servo_knee_left.PDOsetControlWord_EnableVoltage(true);
                servo_knee_left.PDOsetControlWord_QuickStop(true);
                servo_knee_left.PDOsetControlWord_EnableOperation(false);
                servo_knee_left.WritePDO01();

                printf("\nENERGIZANDO O MOTOR 2 E HABILITANDO O CONTROLE");

                endwait = clock() + 0.5 * CLOCKS_PER_SEC;
                while (clock() < endwait)
                {
                }

                servo_hip_right.PDOsetControlWord_SwitchOn(true);
                servo_hip_right.PDOsetControlWord_EnableVoltage(true);
                servo_hip_right.PDOsetControlWord_QuickStop(true);
                servo_hip_right.PDOsetControlWord_EnableOperation(false);
                servo_hip_right.WritePDO01();

                servo_hip_left.PDOsetControlWord_SwitchOn(true);
                servo_hip_left.PDOsetControlWord_EnableVoltage(true);
                servo_hip_left.PDOsetControlWord_QuickStop(true);
                servo_hip_left.PDOsetControlWord_EnableOperation(false);
                servo_hip_left.WritePDO01();

                servo_knee_right.PDOsetControlWord_SwitchOn(true);
                servo_knee_right.PDOsetControlWord_EnableVoltage(true);
                servo_knee_right.PDOsetControlWord_QuickStop(true);
                servo_knee_right.PDOsetControlWord_EnableOperation(false);
                servo_knee_right.WritePDO01();

                servo_knee_left.PDOsetControlWord_SwitchOn(true);
                servo_knee_left.PDOsetControlWord_EnableVoltage(true);
                servo_knee_left.PDOsetControlWord_QuickStop(true);
                servo_knee_left.PDOsetControlWord_EnableOperation(false);
                servo_knee_left.WritePDO01();

                endwait = clock() + 0.5 * CLOCKS_PER_SEC;
                while (clock() < endwait)
                {
                }

                servo_hip_right.PDOsetControlWord_SwitchOn(true);
                servo_hip_right.PDOsetControlWord_EnableVoltage(true);
                servo_hip_right.PDOsetControlWord_QuickStop(true);
                servo_hip_right.PDOsetControlWord_EnableOperation(true);
                servo_hip_right.WritePDO01();

                servo_hip_left.PDOsetControlWord_SwitchOn(true);
                servo_hip_left.PDOsetControlWord_EnableVoltage(true);
                servo_hip_left.PDOsetControlWord_QuickStop(true);
                servo_hip_left.PDOsetControlWord_EnableOperation(true);
                servo_hip_left.WritePDO01();

                servo_knee_right.PDOsetControlWord_SwitchOn(true);
                servo_knee_right.PDOsetControlWord_EnableVoltage(true);
                servo_knee_right.PDOsetControlWord_QuickStop(true);
                servo_knee_right.PDOsetControlWord_EnableOperation(true);
                servo_knee_right.WritePDO01();

                servo_knee_left.PDOsetControlWord_SwitchOn(true);
                servo_knee_left.PDOsetControlWord_EnableVoltage(true);
                servo_knee_left.PDOsetControlWord_QuickStop(true);
                servo_knee_left.PDOsetControlWord_EnableOperation(true);
                servo_knee_left.WritePDO01();

            }
            _running_aux = false;
        }

        void Desabilita_Eixo(int ID)
        {
            _running_aux = true;
            if ((ID == 2) | (ID == 0))
            {
                printf("\nDESABILITANDO O MOTOR E CONTROLE");

                servo_hip_right.PDOsetControlWord_SwitchOn(true);
                servo_hip_right.PDOsetControlWord_EnableVoltage(true);
                servo_hip_right.PDOsetControlWord_QuickStop(true);
                servo_hip_right.PDOsetControlWord_EnableOperation(false);
                servo_hip_right.WritePDO01();

                servo_hip_left.PDOsetControlWord_SwitchOn(true);
                servo_hip_left.PDOsetControlWord_EnableVoltage(true);
                servo_hip_left.PDOsetControlWord_QuickStop(true);
                servo_hip_left.PDOsetControlWord_EnableOperation(false);
                servo_hip_left.WritePDO01();

                servo_knee_right.PDOsetControlWord_SwitchOn(true);
                servo_knee_right.PDOsetControlWord_EnableVoltage(true);
                servo_knee_right.PDOsetControlWord_QuickStop(true);
                servo_knee_right.PDOsetControlWord_EnableOperation(false);
                servo_knee_right.WritePDO01();

                servo_knee_left.PDOsetControlWord_SwitchOn(true);
                servo_knee_left.PDOsetControlWord_EnableVoltage(true);
                servo_knee_left.PDOsetControlWord_QuickStop(true);
                servo_knee_left.PDOsetControlWord_EnableOperation(false);
                servo_knee_left.WritePDO01();

                endwait = clock() + 0.5 * CLOCKS_PER_SEC;
                while (clock() < endwait)
                {
                }

                servo_hip_right.PDOsetControlWord_SwitchOn(false);
                servo_hip_right.PDOsetControlWord_EnableVoltage(true);
                servo_hip_right.PDOsetControlWord_QuickStop(true);
                servo_hip_right.PDOsetControlWord_EnableOperation(false);
                servo_hip_right.WritePDO01();

                servo_hip_left.PDOsetControlWord_SwitchOn(false);
                servo_hip_left.PDOsetControlWord_EnableVoltage(true);
                servo_hip_left.PDOsetControlWord_QuickStop(true);
                servo_hip_left.PDOsetControlWord_EnableOperation(false);
                servo_hip_left.WritePDO01();

                servo_knee_right.PDOsetControlWord_SwitchOn(false);
                servo_knee_right.PDOsetControlWord_EnableVoltage(true);
                servo_knee_right.PDOsetControlWord_QuickStop(true);
                servo_knee_right.PDOsetControlWord_EnableOperation(false);
                servo_knee_right.WritePDO01();

                servo_knee_left.PDOsetControlWord_SwitchOn(false);
                servo_knee_left.PDOsetControlWord_EnableVoltage(true);
                servo_knee_left.PDOsetControlWord_QuickStop(true);
                servo_knee_left.PDOsetControlWord_EnableOperation(false);
                servo_knee_left.WritePDO01();

            }
            _running_aux = false;
        }

        void setVelocityZero() {
            servo_knee_right.PDOsetVelocitySetpoint(0);
            servo_knee_right.WritePDO02();

            servo_knee_left.PDOsetVelocitySetpoint(0);
            servo_knee_left.WritePDO02();

            servo_hip_right.PDOsetVelocitySetpoint(0);
            servo_hip_right.WritePDO02();

            servo_hip_left.PDOsetVelocitySetpoint(0);
            servo_hip_left.WritePDO02();
        }

        void reset_falhas()
        {
            _running_aux = true;
            //EPOS 04
            servo_hip_right.PDOsetControlWord_FaultReset(true);
            servo_hip_right.WritePDO01();

            printf("\nResetando as falhas.");

            esperar_n_seg(1);

            printf("..");

            //EPOS 04
            servo_hip_right.PDOsetControlWord_FaultReset(false);
            servo_hip_right.WritePDO01();

            printf("..");

            esperar_n_seg(1);

            printf("..");


            //EPOS 05
            servo_hip_left.PDOsetControlWord_FaultReset(true);
            servo_hip_left.WritePDO01();

            printf("\nResetando as falhas.");

            esperar_n_seg(1);

            printf("..");

            //EPOS 05
            servo_hip_left.PDOsetControlWord_FaultReset(false);
            servo_hip_left.WritePDO01();

            printf("..");

            esperar_n_seg(1);

            printf("..");


            //EPOS 02
            encoder_knee_right.PDOsetControlWord_FaultReset(true);
            encoder_knee_right.WritePDO01();

            printf("\nResetando as falhas.");

            esperar_n_seg(1);

            printf("..");

            //EPOS 02
            encoder_knee_right.PDOsetControlWord_FaultReset(false);
            encoder_knee_right.WritePDO01();

            printf("..");

            esperar_n_seg(1);

            printf("..");

            //EPOS 03
            servo_knee_left.PDOsetControlWord_FaultReset(true);
            servo_knee_left.WritePDO01();

            printf("..");

            esperar_n_seg(1);

            printf("..");

            //EPOS 03
            servo_knee_left.PDOsetControlWord_FaultReset(false);
            servo_knee_left.WritePDO01();

            printf("..");

            esperar_n_seg(1);

            printf("..");

            //EPOS 01
            servo_knee_right.PDOsetControlWord_FaultReset(true);
            servo_knee_right.WritePDO01();

            printf("..");

            esperar_n_seg(1);

            printf("..");

            //EPOS 01
            servo_knee_right.PDOsetControlWord_FaultReset(false);
            servo_knee_right.WritePDO01();

            printf("..");


            esperar_n_seg(1);

            printf("OK");
            _running_aux = false;
        }

        void define_origen()
        {
            _running_aux = true;
            int total_time = 0;
           
            epos.sync();

            printf("Definindo Origem... ");

            esperar_n_seg(1);

            printf("...");

            servo_hip_right.ReadPDO01();
            ZERO_SERVO_HIP_right = servo_hip_right.PDOgetActualPosition();

            servo_hip_left.ReadPDO01();
            ZERO_SERVO_HIP_left = servo_hip_left.PDOgetActualPosition();

            encoder_knee_right.ReadPDO01();
            ZERO_SENSOR_KNEE_right = -encoder_knee_right.PDOgetActualPosition();

            servo_knee_right.ReadPDO01();
            ZERO_SERVO_KNEE_right = servo_knee_right.PDOgetActualPosition();

            servo_knee_left.ReadPDO01();
            ZERO_SERVO_KNEE_left = servo_knee_left.PDOgetActualPosition();

            _running_aux = false;
        }


        ThreadEposEXO_CAN(){
            
            //init_comm_eixos();
        }

        bool init_comunication = false;

        void initAllComunications() {
            _running_aux = true;
            if (!init_comunication) {

                data = fromEposEXO_CAN{ 0 };
                std::cout << "INICIALIZANDO COMUNICACAO CANOpen COM AS EPOS" << std::endl;
                epos = EPOS_NETWORK(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER);

                servo_hip_right = AXIS(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_03);
                servo_hip_left = AXIS(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_04);
                servo_knee_right = AXIS(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_01);
                encoder_knee_right = AXIS(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SENSOR_01);
                servo_knee_left = AXIS(CAN_INTERFACE, CAN_DATABASE, CAN_CLUSTER, NET_ID_SERVO_02);

                start_transmissao_rede_epos();
                                
                init_comunication = true;

                init_comm_eixos();
            }
            
            _running_aux = false;
        }
 
        void _setup(){
            initAllComunications();
            reset_falhas();
            define_origen();

        }

        void _cleanup(){
         
            if (_aux_thread.joinable())
                _aux_thread.join();
            setVelocityZero();
            Desabilita_Eixo(0);
        }

        void _loop(){

            epos.sync();
              
            _mtx.lock();   
             
            _mtx.unlock();
        }

        void setVelocityMode(AXIS* node) {
            node->VCS_SetOperationMode(VELOCITY_MODE);
        }

        

        void setVelocity(AXIS* node, long vel) {
            _mtx.lock();
                node->PDOsetVelocitySetpoint(vel);
                node->WritePDO02();
            _mtx.unlock();
        }

        void getPosition(AXIS* node, long  * pos) {
            _mtx.lock();
                node->ReadPDO01();
                (*pos) = node->PDOgetActualPosition();
            _mtx.unlock();
        }
     
        void _updateGUI(){
            ImGui::Begin(_name.c_str());
                if(isAlive)
                    ImGui::Text("EposEXO_CAN : RUNNING");
                else
                    ImGui::Text("EposEXO_CAN : PAUSED");
               
                if (!_running_aux) {
                    if (ImGui::Button("Init CAN Network")) {
                        if (_aux_thread.joinable())
                            _aux_thread.join();
                        _aux_thread = std::thread(&ThreadEposEXO_CAN::initAllComunications, this);
                    }

                    if (ImGui::Button("Reset falhas")) {
                        if (_aux_thread.joinable())
                            _aux_thread.join();
                        _aux_thread = std::thread(&ThreadEposEXO_CAN::reset_falhas, this);
                    }

                    if (ImGui::Button("Define Origen")) {
                        if (_aux_thread.joinable())
                            _aux_thread.join();
                        _aux_thread = std::thread(&ThreadEposEXO_CAN::define_origen, this);
                    }

                    if (ImGui::Button("Set Velocity Mode ")) {
                        setVelocityZero();
                    }
                    ImGui::SameLine();

                    if (ImGui::Button("Activate motors ")) {
                        Habilita_Eixo(2);
                    }

                    ImGui::SameLine();

                    if (ImGui::Button("Set Zero Velocity")) {
                        setVelocityMode(&servo_knee_right);
                        setVelocityMode(&servo_knee_left );
                        setVelocityMode(&servo_hip_left  );
                        setVelocityMode(&servo_hip_right);
                        setVelocityZero();
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Desactivate motors ")) {
                        Desabilita_Eixo(0);
                    }
                    if (ImGui::Button("SetVelocity 100 ")) {
                        setVelocity(&servo_knee_right,100);
                    }


                }
                

            ImGui::End();
        }

    
        void getData(void * _data){
            _mtx.lock();
                memcpy(_data,&data,sizeof(fromEposEXO_CAN));
            _mtx.unlock();
        }
};
