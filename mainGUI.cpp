
#include <string>
#include <thread>
#include <mutex>
#include <imgui_demo.cpp>

#include <thread_exo.h>
ThreadExo * ATIMX1;
ThreadExo * XsensRead;
ThreadExo * XSENSLeo;
ThreadExo * EposEXO1;
ThreadExo * EposEXOCAN1;
ThreadExo * MarkovMao1;

#include "threads/threadATIMX.hpp"
#include "threads/threadXsensRead.hpp"
#include "threads/threadEposEXO.hpp"
#include "threads/threadEposEXO_CAN.hpp"

#include "threads/threadXsensLeo.hpp"
#include "threads/threadMarkovMauricio.hpp"

int mainGUI(){
    ATIMX1 = new ThreadExo(new ThreadATIMX(),"ATIMX 1",1/200.0f,7);
    XsensRead = new ThreadExo(new ThreadXsensRead(),"XSens Read 1",1/100.0f,25);
    EposEXO1 = new ThreadExo(new ThreadEposEXO(),"EposEXO 1",1/200.0f,4);
    EposEXOCAN1 = new ThreadExo(new ThreadEposEXO_CAN(),"EposEXO CAN 1",1/400.0f,4);
    MarkovMao1 = new ThreadExo(new ThreadMarkovMao1(),"MarkovMao 1",1/200.0f,20);
    XSENSLeo = new ThreadExo(new ThreadXsensLeo(),"XSens Leo 1",1/75.0f,11);
    return 0;
}

void loopGUI(bool * exit){
    static bool show = true;
    static fromATIMX atimx{0};
    static char tex [255];
    static bool isRunning = false;
    
    isRunning = threadsExo.RUNNING();
    show = !isRunning;

    ImGui::ShowDemoWindow();

    ImGui::Begin("ControlPanel",&show,0);
       
        if (ImGui::Button("Exit")) {
            (*exit) = true;
            return;
        }
      
        if (ImGui::CollapsingHeader(isRunning? "RUNNING" : "PAUSED" , ImGuiTreeNodeFlags_Leaf)){
           
            
            if(!isRunning){

                ImGui::Columns(2, NULL, false);

                if(ImGui::Button("CLEAR")){
                    ItALL{
                        cur_->toRUNfromGUI = false;             
                    }
                }ImGui::NextColumn();
                
                if(ImGui::Button("TEST ATIMX 1")){
                        ATIMX1->toRUNfromGUI = true;
                }ImGui::NextColumn();

                if(ImGui::Button("TEST Xsens Leo")){
                        XsensRead->toRUNfromGUI = true;
                        XSENSLeo->toRUNfromGUI = true;
                }ImGui::NextColumn();

                if(ImGui::Button("Controle Markoviano")){
                    ATIMX1->toRUNfromGUI     = true;
                    EposEXOCAN1->toRUNfromGUI   = true;
                    XsensRead->toRUNfromGUI  = true;
                    XSENSLeo->toRUNfromGUI   = true;
                    MarkovMao1->toRUNfromGUI = true;
                }ImGui::NextColumn();
                
                ImGui::Columns(1);
                
            }

            if(!isRunning){
                ImGui::Columns(3, NULL, false);
                ItALL{
                    ImGui::Checkbox(("RUN : " + cur_->getName()).c_str(),&cur_->toRUNfromGUI);     
                    ImGui::NextColumn();              
                }
                ImGui::Columns(1);
            }

            if(ImGui::Button("FORCE STOP")){
                GUI.FORCE_STOP = true;                
            }
            if(!isRunning){
                ImGui::Columns(4, NULL, false);

                static float Time = 1.0f;
                static bool INIT = false;
                ImGui::DragFloat("##time_count", &Time, 0.1f, 2, 60, "%.2f Segundos");
                if(Time <= 0) Time = 10;
                ImGui::SameLine();
                if(ImGui::Button("RUN")){
                    INIT = true;
                }

                static float ti[] = {5.0f,10.0f,20.0f,30.0f,40.0f,50.0f,60.0f};
                static char ti_labels[50];

                for(int i_ti = 0 ; i_ti < 7 ; i_ti ++){
                    sprintf(ti_labels,"RUN: %.2f s ##%d",ti[i_ti],i_ti);
                    ImGui::NextColumn(); 
                    if(ImGui::Button(ti_labels)){
                        INIT = true;
                        Time = ti[i_ti];
                    }
                    
                }
             
                ImGui::Columns(1);

                if(INIT){
                    INIT = false;
                    threadsExo.clear();
                    ItALL{
                        if(cur_->toRUNfromGUI)  threadsExo.add(cur_);
                    }
                    threadsExo.run(Time);
                }
    
                
             }

        }

       



	ImGui::End();

    ItALL{
        if(cur_->toRUNfromGUI)
            cur_->updateGUI();
    }

    //threadsExo.updateGUI();

    return;
}