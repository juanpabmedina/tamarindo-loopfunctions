/**
  * @file <loop-functions/IcraLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @license MIT License
  */

#include "CommTskLoopFunc.h"
#include <fstream>
#include <iostream>
#include <numeric>
using namespace std;

/****************************************/
/****************************************/

CommTskLoopFunction::CommTskLoopFunction() {
    m_unClock = 0;
    m_unStopTime = 0;
    m_unStopBlock = 0;
    m_fObjectiveFunction = 0;
    m_fPheromoneParameter = 0;
}

/****************************************/
/****************************************/

CommTskLoopFunction::CommTskLoopFunction(const CommTskLoopFunction& orig) {
}

/****************************************/
/****************************************/

CommTskLoopFunction::~CommTskLoopFunction() {}

/****************************************/
/****************************************/

void CommTskLoopFunction::Destroy() {

    m_tRobotStates.clear();
    m_tLEDStates.clear();
}

/****************************************/
/****************************************/

void CommTskLoopFunction::Init(TConfigurationNode& t_tree) {

    CoreLoopFunctions::Init(t_tree);
    TConfigurationNode cParametersNode;
    try {
      cParametersNode = GetNode(t_tree, "params");
      GetNodeAttributeOrDefault(cParametersNode, "maximization", m_bMaximization, (bool) false);
      // m_bMaximization = true;
    } catch(std::exception e) {
    }

    m_cUVColor.SetRed(128);
    m_cUVColor.SetGreen(0);
    m_cUVColor.SetBlue(128);

    visitedId={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    InitRobotStates();
    InitPhormicaState();
    InitMocaState();

}

/****************************************/
/****************************************/

void CommTskLoopFunction::Reset() {
    CoreLoopFunctions::Reset();

    m_pcPhormica->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
    m_unClock = 0;
    m_unStopBlock = 0;
    m_unStopTime = 0;
    m_fObjectiveFunction = 0;

    m_tRobotStates.clear();
    m_tLEDStates.clear();

    visitedId={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    InitMocaState();
    InitRobotStates();
    InitPhormicaState();
}

/****************************************/
/****************************************/

void CommTskLoopFunction::PostStep() {

    m_unClock = GetSpace().GetSimulationClock();
    GetRobotScore();
    TimerControl();
    MocaControl();
    UpdatePhormicaState();
}

/****************************************/
/****************************************/

void CommTskLoopFunction::PostExperiment() {  

    UInt16 count1 = 0;
    UInt16 count2 = 0;
    for(std::size_t i = 0; i < visitedId.size(); i++){
        if (visitedId[i]==1){
            count1 += 1;
        }
        if (visitedId[i]>1){
            count2 += 1;
        }
       // LOG << visitedId[i] << std::endl;
    }
    m_fObjectiveFunction = count1 - count2;

    // ofstream score;
    // score.open("data/score_tasking.txt", ofstream::app);

    if (m_bMaximization == true){
        LOG << -m_fObjectiveFunction << std::endl;
        // score <<-m_fObjectiveFunction << std::endl;
    }
    else {
        LOG << m_fObjectiveFunction << std::endl;
        // score <<m_fObjectiveFunction << std::endl;

    }
}

/****************************************/
/****************************************/

Real CommTskLoopFunction::GetObjectiveFunction() {
    if (m_bMaximization == true){
        return -m_fObjectiveFunction;
    }
    else {
        return m_fObjectiveFunction;
    }
}

/****************************************/
/****************************************/

void CommTskLoopFunction::MocaControl() {

    if (m_unClock == m_unStopTime) {
        CSpace::TMapPerType& tBlocksMap = GetSpace().GetEntitiesByType("block");
        UInt32 unBlocksID = 0;
        for (CSpace::TMapPerType::iterator it = tBlocksMap.begin(); it != tBlocksMap.end(); ++it) {
            CBlockEntity* pcBlock = any_cast<CBlockEntity*>(it->second);
             pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);

            switch (unBlocksID)
            {
            case 5:
                pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
                break;

            case 6:
                pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
                break;

            case 7:
                pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
                break;

            case 13:
                pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
                break;

            case 14:
                pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
                break;

            case 15:
                pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
                break;

            default:
                break;
            }

            unBlocksID += 1;
        }
    }
}

/****************************************/
/****************************************/

void CommTskLoopFunction::TimerControl(){
}

/****************************************/
/****************************************/

void CommTskLoopFunction::GetRobotScore() {

  UpdateRobotPositions();
  CVector2 robotInStation;
  Real unScore = 0;
  int idStation;
  bool isInStation;
  TRobotStateMap::iterator it;
  for (it = m_tRobotStates.begin(); it != m_tRobotStates.end(); ++it) {
    std::pair<bool, int> station = IsRobotInStation(it->second.cPosition);
    isInStation = station.first;
    idStation = station.second;
    if (idStation != -1){
        it->second.unTimer = it->second.unTimer - 1;
        if (it->second.unTimer > 0){
            it->second.visitedStation = false;
        }
        else if (it->second.unTimer == 0){
            it->second.visitedStation = true;
            visitedId[it->second.idStation] = visitedId[it->second.idStation] + 1;
        //    LOG << "Visited station: "<< it->second.idStation << std::endl;
        }
       it->second.idStation = idStation;
    //    LOG << it->second.unTimer << std::endl;
    }
    else {
        it->second.unTimer = 30;
  
        if ( it->second.visitedStation == true){
            it->second.visitedStation = false;
        }
    }
    // LOG << "Visited station: "<< visitedStation << std::endl;


 
    
    // LOG << id << std::endl;
    
  }


  m_fObjectiveFunction += unScore;
}

/****************************************/
/****************************************/

// Real CommTskLoopFunction::GetRobotOutScore() {
//
//     UpdateRobotPositions();
//
//     Real unScore = 0;
//     TRobotStateMap::iterator it;
//     for (it = m_tRobotStates.begin(); it != m_tRobotStates.end(); ++it) {
//         if (it->second.cPosition.GetY() >= -0.60)
//             unScore+=1;
//     }
//
//     return unScore;
// }

/****************************************/
/****************************************/

argos::CColor CommTskLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
    float a = 0.06;
    float b = 0;
    float block = 0.125;
    float separation = block*8;
    float offset = 0;
    std::vector<int> interSpace = {3,6,9,12};
    ///// Left side /////
    if(c_position_on_plane.GetX() <= separation-block*(interSpace[0]) -b -offset && c_position_on_plane.GetX() >= separation -block*(interSpace[0]+1) +b && c_position_on_plane.GetY() >= separation - a && c_position_on_plane.GetY() <= separation ){
        return CColor::GRAY50;
    }
    else if(c_position_on_plane.GetX() <= separation-block*(interSpace[1]) - b -offset && c_position_on_plane.GetX() >= separation -block*(interSpace[1]+1) +b && c_position_on_plane.GetY() >= separation - a  && c_position_on_plane.GetY() <= separation){
        return CColor::GRAY50;
    }
    else if(c_position_on_plane.GetX() <= separation-block*(interSpace[2]) - b -offset&& c_position_on_plane.GetX() >= separation -block*(interSpace[2]+1) +b && c_position_on_plane.GetY() >= separation - a  && c_position_on_plane.GetY() <= separation){
        return CColor::GRAY50;
    }
    else if(c_position_on_plane.GetX() <= separation-block*(interSpace[3]) - b -offset&& c_position_on_plane.GetX() >= separation -block*(interSpace[3]+1) +b && c_position_on_plane.GetY() >= separation - a && c_position_on_plane.GetY() <= separation ){
        return CColor::GRAY50;
    }

    ///// Right side /////
    else if(c_position_on_plane.GetX() <= separation-block*(interSpace[0]) -b && c_position_on_plane.GetX() >= separation -block*(interSpace[0]+1) +b +offset&& c_position_on_plane.GetY() <= -separation + a  && c_position_on_plane.GetY() >= -separation){
        return CColor::GRAY50;
    }

    else if(c_position_on_plane.GetX() <= separation-block*(interSpace[1])  - b && c_position_on_plane.GetX() >= separation -block*(interSpace[1]+1) +b +offset&& c_position_on_plane.GetY() <= -separation + a  && c_position_on_plane.GetY() >= -separation){
        return CColor::GRAY50;
    }

    else if(c_position_on_plane.GetX() <= separation-block*(interSpace[2])  - b && c_position_on_plane.GetX() >= separation -block*(interSpace[2]+1) +b +offset&& c_position_on_plane.GetY() <= -separation + a  && c_position_on_plane.GetY() >= -separation){
        return CColor::GRAY50;
    }

    else if(c_position_on_plane.GetX() <= separation-block*(interSpace[3])  - b && c_position_on_plane.GetX() >= separation -block*(interSpace[3]+1) +b +offset&& c_position_on_plane.GetY() <= -separation + a  && c_position_on_plane.GetY() >= -separation){
        return CColor::GRAY50;
    }


    ///// Top side /////
    else if(c_position_on_plane.GetY() <= separation-block*(interSpace[0]) - b && c_position_on_plane.GetY() >= separation -block*(interSpace[0]+1) +b +offset&& c_position_on_plane.GetX() >= separation - a && c_position_on_plane.GetX() <= separation  ){
        return CColor::GRAY50;
    }

    else if(c_position_on_plane.GetY() <= separation-block*(interSpace[1]) - b && c_position_on_plane.GetY() >= separation -block*(interSpace[1]+1) +b +offset&& c_position_on_plane.GetX() >= separation - a  && c_position_on_plane.GetX() <= separation ){
        return CColor::GRAY50;
    }

    else if(c_position_on_plane.GetY() <= separation-block*(interSpace[2]) - b && c_position_on_plane.GetY() >= separation -block*(interSpace[2]+1) +b +offset&& c_position_on_plane.GetX() >= separation - a  && c_position_on_plane.GetX() <= separation ){
        return CColor::GRAY50;
    }
    else if(c_position_on_plane.GetY() <= separation-block*(interSpace[3]) - b && c_position_on_plane.GetY() >= separation -block*(interSpace[3]+1) +b +offset&& c_position_on_plane.GetX() >= separation - a  && c_position_on_plane.GetX() <= separation ){
        return CColor::GRAY50;
    }



    ///// Bottom side /////
    else if(c_position_on_plane.GetY() <= separation-block*(interSpace[0]) - b -offset&& c_position_on_plane.GetY() >= separation -block*(interSpace[0]+1) +b && c_position_on_plane.GetX() <= -separation + a  && c_position_on_plane.GetX() >= -separation){
        return CColor::GRAY50;
    }
     else if(c_position_on_plane.GetY() <= separation-block*(interSpace[1]) - b -offset&& c_position_on_plane.GetY() >= separation -block*(interSpace[1]+1) +b && c_position_on_plane.GetX() <= -separation + a  && c_position_on_plane.GetX() >= -separation){
        return CColor::GRAY50;
    }
    else if(c_position_on_plane.GetY() <= separation-block*(interSpace[2]) - b -offset&& c_position_on_plane.GetY() >= separation -block*(interSpace[2]+1) +b && c_position_on_plane.GetX() <= -separation + a  && c_position_on_plane.GetX() >= -separation){
        return CColor::GRAY50;
    }
    else if(c_position_on_plane.GetY() <= separation-block*(interSpace[3]) - b -offset&& c_position_on_plane.GetY() >= separation -block*(interSpace[3]+1) +b && c_position_on_plane.GetX() <= -separation + a  && c_position_on_plane.GetX() >= -separation){
        return CColor::GRAY50;
    }

    // else if(c_position_on_plane.GetX() <= -0.125 && c_position_on_plane.GetX() <= -0.375 && c_position_on_plane.GetY() > 0 ){
    //     return CColor::WHITE;
    // }
    else {
        return CColor::WHITE;
    }
    
}

std::pair<bool, int> CommTskLoopFunction::IsRobotInStation(CVector2 c_position_on_plane) {

    float a = 0.1;
    float b = -0.02;
    float block = 0.125;
    int id;
    bool inStation;
    float separation = block*8;
    float offset = 0;
    std::vector<int> interSpace = {3,6,9,12};

 ///// Left side /////
    if(c_position_on_plane.GetX() <= separation-block*interSpace[0] -b -offset&& c_position_on_plane.GetX() >= separation -block*(interSpace[0]+1) +b && c_position_on_plane.GetY() >= separation - a && c_position_on_plane.GetY() <= separation ){
        id = 0;
        inStation = true;
        return std::make_pair(inStation,id);
    }

    else if(c_position_on_plane.GetX() <= separation-block*interSpace[1] - b -offset&& c_position_on_plane.GetX() >= separation -block*(interSpace[1]+1) +b && c_position_on_plane.GetY() >= separation - a  && c_position_on_plane.GetY() <= separation){
        id = 1;
        inStation = true;
        return std::make_pair(inStation,id);
    }

    else if(c_position_on_plane.GetX() <= separation-block*(interSpace[2]) - b -offset&& c_position_on_plane.GetX() >= separation -block*(interSpace[2]+1) +b && c_position_on_plane.GetY() >= separation - a  && c_position_on_plane.GetY() <= separation){
        id = 2;
        inStation = true;
        return std::make_pair(inStation,id);
    }

        else if(c_position_on_plane.GetX() <= separation-block*(interSpace[3]) - b -offset&& c_position_on_plane.GetX() >= separation -block*(interSpace[3]+1) +b && c_position_on_plane.GetY() >= separation - a  && c_position_on_plane.GetY() <= separation){
        id = 3;
        inStation = true;
        return std::make_pair(inStation,id);
    }

    ///// Right side /////
    else if(c_position_on_plane.GetX() <= separation-block*(interSpace[0]) -b && c_position_on_plane.GetX() >= separation -block*(interSpace[0]+1) +b +offset&& c_position_on_plane.GetY() <= -separation + a  && c_position_on_plane.GetY() >= -separation){
        id = 4;
        inStation = true;
        return std::make_pair(inStation,id);
    }

    else if(c_position_on_plane.GetX() <= separation-block*(interSpace[1]) -b && c_position_on_plane.GetX() >= separation -block*(interSpace[1]+1) +b +offset&& c_position_on_plane.GetY() <= -separation + a  && c_position_on_plane.GetY() >= -separation){
        id = 5;
        inStation = true;
        return std::make_pair(inStation,id);
    }

    else if(c_position_on_plane.GetX() <= separation-block*(interSpace[2]) - b && c_position_on_plane.GetX() >= separation -block*(interSpace[2]+1) +b +offset&& c_position_on_plane.GetY() <= -separation + a  && c_position_on_plane.GetY() >= -separation){
        id = 6;
        inStation = true;
        return std::make_pair(inStation,id);
    }
    else if(c_position_on_plane.GetX() <= separation-block*(interSpace[3]) -b && c_position_on_plane.GetX() >= separation -block*(interSpace[3]+1) +b +offset&& c_position_on_plane.GetY() <= -separation + a  && c_position_on_plane.GetY() >= -separation){
        id = 7;
        inStation = true;
        return std::make_pair(inStation,id);
    }

    ///// Top side /////
    else if(c_position_on_plane.GetY() <= separation-block*(interSpace[0]) - b && c_position_on_plane.GetY() >= separation -block*(interSpace[0]+1) +b +offset&& c_position_on_plane.GetX() >= separation - a && c_position_on_plane.GetX() <= separation  ){
        id = 8;
        inStation = true;
        return std::make_pair(inStation,id);
    }
    else if(c_position_on_plane.GetY() <= separation-block*(interSpace[1]) - b && c_position_on_plane.GetY() >= separation -block*(interSpace[1]+1) +b +offset&& c_position_on_plane.GetX() >= separation - a && c_position_on_plane.GetX() <= separation  ){
        id = 9;
        inStation = true;
        return std::make_pair(inStation,id);
    }
    else if(c_position_on_plane.GetY() <= separation-block*(interSpace[2]) - b && c_position_on_plane.GetY() >= separation -block*(interSpace[2]+1) +b +offset&& c_position_on_plane.GetX() >= separation - a  && c_position_on_plane.GetX() <= separation ){
        id = 10;
        inStation = true;
        return std::make_pair(inStation,id);
    }
    else if(c_position_on_plane.GetY() <= separation-block*(interSpace[3]) - b && c_position_on_plane.GetY() >= separation -block*(interSpace[3]+1) +b +offset&& c_position_on_plane.GetX() >= separation - a && c_position_on_plane.GetX() <= separation  ){
        id = 11;
        inStation = true;
        return std::make_pair(inStation,id);
    }


    ///// Bottom side /////
    else if(c_position_on_plane.GetY() <= separation-block*(interSpace[0]) - b -offset&& c_position_on_plane.GetY() >= separation -block*(interSpace[0]+1) +b && c_position_on_plane.GetX() <= -separation + a  && c_position_on_plane.GetX() >= -separation){
        id = 12;
        inStation = true;
        return std::make_pair(inStation,id);
    }

    else if(c_position_on_plane.GetY() <= separation-block*(interSpace[1]) - b -offset&& c_position_on_plane.GetY() >= separation -block*(interSpace[1]+1) +b && c_position_on_plane.GetX() <= -separation + a  && c_position_on_plane.GetX() >= -separation){
        id = 13;
        inStation = true;
        return std::make_pair(inStation,id);
    }

    else if(c_position_on_plane.GetY() <= separation-block*(interSpace[2]) - b -offset&& c_position_on_plane.GetY() >= separation -block*(interSpace[2]+1)+b && c_position_on_plane.GetX() <= -separation + a  && c_position_on_plane.GetX() >= -separation){
        id = 14;
        inStation = true;
        return std::make_pair(inStation,id);
    }

    else if(c_position_on_plane.GetY() <= separation-block*(interSpace[3]) - b -offset&& c_position_on_plane.GetY() >= separation -block*(interSpace[3]+1) +b && c_position_on_plane.GetX() <= -separation + a  && c_position_on_plane.GetX() >= -separation){
        id = 15;
        inStation = true;
        return std::make_pair(inStation,id);
    }
    // else if(c_position_on_plane.GetX() <= -0.125 && c_position_on_plane.GetX() <=  -0.375 && c_position_on_plane.GetY() > 0 ){
    //     return CColor::WHITE;
    // }
    else {
        id = -1;
        inStation = false;
        return std::make_pair(inStation,id);
    }
}
/****************************************/
/****************************************/

void CommTskLoopFunction::UpdateRobotPositions() {
    CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
        CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                           pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        m_tRobotStates[pcEpuck].cLastPosition = m_tRobotStates[pcEpuck].cPosition;
        m_tRobotStates[pcEpuck].cPosition = cEpuckPosition;

        // Updating the Pheromone trail width parameter w.r.t ON UV LEDs

        CColor cEpuckUVOne = pcEpuck->GetLEDEquippedEntity().GetLED(11).GetColor();
        CColor cEpuckUVTwo = pcEpuck->GetLEDEquippedEntity().GetLED(12).GetColor();
        CColor cEpuckUVThree = pcEpuck->GetLEDEquippedEntity().GetLED(13).GetColor();

        if (cEpuckUVOne == CColor::BLACK && cEpuckUVTwo == m_cUVColor && cEpuckUVThree == CColor::BLACK){
            //m_fPheromoneParameter = 0.02; // Thin pheromone trail
            m_tRobotStates[pcEpuck].unPheromoneLEDs = 3;
        }
        else if (cEpuckUVOne == m_cUVColor && cEpuckUVTwo == m_cUVColor && cEpuckUVThree == m_cUVColor){
            //m_fPheromoneParameter = 0.045; // Thick pheromone trails
            m_tRobotStates[pcEpuck].unPheromoneLEDs = 9;
        }
        else {
            //m_fPheromoneParameter = 0; // No pheromone trail
            m_tRobotStates[pcEpuck].unPheromoneLEDs = 0;
        }
        //LOG<< m_fPheromoneParameter<< "Robot update LED color" << std::endl;
    }
}

/****************************************/
/****************************************/

void CommTskLoopFunction::InitRobotStates() {

    CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
        CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
        std::string strRobotId = pcEpuck->GetId().substr(5,1);
        UInt32 nRobotId = std::stoi(strRobotId);
        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                           pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        m_tRobotStates[pcEpuck].cLastPosition = cEpuckPosition;
        m_tRobotStates[pcEpuck].cPosition = cEpuckPosition;
        m_tRobotStates[pcEpuck].unPheromoneLEDs = 0;
        m_tRobotStates[pcEpuck].unItem = 0;
        m_tRobotStates[pcEpuck].unId = nRobotId;
        m_tRobotStates[pcEpuck].unTimer = 20;
        m_tRobotStates[pcEpuck].visitedStation = false;
    }
}

/****************************************/
/****************************************/

void CommTskLoopFunction::InitPhormicaState() {

    CSpace::TMapPerType& tPhormicaMap = GetSpace().GetEntitiesByType("phormica");
    CVector2 cLEDPosition(0,0);
    // Change the first argument to change the quantity of layers of pheromone
    std::vector<int> pheromoneLayers(30,0);
    for (CSpace::TMapPerType::iterator it = tPhormicaMap.begin(); it != tPhormicaMap.end(); ++it) {
        CPhormicaEntity* pcPhormica = any_cast<CPhormicaEntity*>(it->second);
        m_pcPhormica = pcPhormica;
        m_pcPhormica->GetLEDEquippedEntity().Enable();
        m_pcPhormica->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
        m_unNumberLEDs = m_pcPhormica->GetLEDEquippedEntity().GetLEDs().size();
        for (UInt32 i = 0; i < m_unNumberLEDs; ++i) {
            cLEDPosition.Set(m_pcPhormica->GetLEDEquippedEntity().GetLED(i).GetPosition().GetX(),
                             m_pcPhormica->GetLEDEquippedEntity().GetLED(i).GetPosition().GetY());
            m_tLEDStates[i].unLEDIndex = i;
            m_tLEDStates[i].cLEDPosition = cLEDPosition;
            m_tLEDStates[i].unTimer = 0;
            m_tLEDStates[i].pheromoneLayers = pheromoneLayers;
            m_tLEDStates[i].layersEmpty = true;
        }
    }
}

/****************************************/
/****************************************/

void CommTskLoopFunction::UpdatePhormicaState() {

    CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);

    TLEDStateMap::iterator itLED;
    TRobotStateMap::iterator it;
    for (itLED = m_tLEDStates.begin(); itLED != m_tLEDStates.end(); ++itLED) {
        
        for (it = m_tRobotStates.begin(); it != m_tRobotStates.end(); ++it) {
            Real d = (itLED->second.cLEDPosition - it->second.cPosition).Length();
            Real fPheromone = 0;
            // to slove reset bug
            if (m_unClock == 1)
                it->second.unPheromoneLEDs = 0;

            if (it->second.unPheromoneLEDs <= 0)
                fPheromone = 0;
            else if (it->second.unPheromoneLEDs <= 3)
                fPheromone = 0.02;
            else
                fPheromone = 0.045;
            
            // fPheromone = 0.01;
           
            UInt32 swarmId = it->second.unId;
            
            //if (d <= m_fPheromoneParameter) {
            if (d <= fPheromone) {
                itLED->second.unTimer = 500; // Pheromone decay time
                if (itLED->second.pheromoneLayers[itLED->second.pheromoneLayers.size() - 1] == 0)
                    // Find the first empty layer
                    for (UInt16 i = 0; i <  itLED->second.pheromoneLayers.size(); ++i) {
                        
                        if (itLED->second.pheromoneLayers[i] == 0) {
                            itLED->second.pheromoneLayers[i] = itLED->second.unTimer;
                            itLED->second.layersEmpty = false;
                            break;
                        } 
                    }
                // If the vector is full shift layers down adn assign to the last position a new counter
                else {
                    for (UInt16 i = 1; i < itLED->second.pheromoneLayers.size(); ++i) {
                        itLED->second.pheromoneLayers[i - 1] = itLED->second.pheromoneLayers[i];
                        //  LOG << "SHIFT" << std::endl;
                    }
                    itLED->second.pheromoneLayers[itLED->second.pheromoneLayers.size() - 2] = itLED->second.unTimer;
                }
                // LOG << pheromoneLayers[1] << std::endl;
                itLED->second.unCount = itLED->second.unCount + 1;
            }
        }
        // Decrease decay time for non-empty layers
        for (UInt16 i = 0; i < itLED->second.pheromoneLayers.size(); ++i) {
            if (itLED->second.pheromoneLayers[i] != 0) {
                itLED->second.pheromoneLayers[i] = itLED->second.pheromoneLayers[i] - 1;
            }
        }

        // Shift layers down if first is empty and a robot passed before
        if (itLED->second.pheromoneLayers[0] == 0 && itLED-> second.layersEmpty  == false) {
            for (UInt16 i = 1; i < itLED->second.pheromoneLayers.size(); ++i) {
                itLED->second.pheromoneLayers[i - 1] = itLED->second.pheromoneLayers[i];
                //  LOG << "SHIFT" << std::endl;
            }
        }

         // Reset if all layers are empty
        if (accumulate(itLED->second.pheromoneLayers.begin(), itLED->second.pheromoneLayers.end(), 0) == 0) {
            itLED-> second.layersEmpty  = true;
        }
        // LOG << pheromoneLayers[1] << std::endl;
        // Check for active pheromone (optional)
        // pheromoneLayers.size() - 2
        if (itLED->second.pheromoneLayers[itLED->second.pheromoneLayers.size() - 1] != 0) {
            m_pcPhormica->GetLEDEquippedEntity().SetLEDColor(itLED->second.unLEDIndex,CColor::MAGENTA);
            // LOG << "SHIFT" << std::endl;
        }
        else {
            m_pcPhormica->GetLEDEquippedEntity().SetLEDColor(itLED->second.unLEDIndex,CColor::BLACK);
            // LOG << "BLACK" << std::endl;
        }

        // UInt32 unLEDTimer = itLED->second.unTimer;
        // UInt32 unLEDCount = itLED->second.unCount;
        
        // // Pheromone intensity parameter
        // if (unLEDCount > 20){
        //     m_pcPhormica->GetLEDEquippedEntity().SetLEDColor(itLED->second.unLEDIndex,CColor::MAGENTA);
        // }
    
        // if (unLEDTimer == 0){
        //     m_pcPhormica->GetLEDEquippedEntity().SetLEDColor(itLED->second.unLEDIndex,CColor::GRAY70);
        //     itLED->second.unCount = 0;
        // }
        // else {
        //     itLED->second.unTimer = unLEDTimer - 1;
        // }
    }
}

/****************************************/
/****************************************/

void CommTskLoopFunction::InitMocaState() {

  CSpace::TMapPerType& tBlocksMap = GetSpace().GetEntitiesByType("block");
  UInt32 unBlocksID = 0;
  for (CSpace::TMapPerType::iterator it = tBlocksMap.begin(); it != tBlocksMap.end(); ++it) {
      CBlockEntity* pcBlock = any_cast<CBlockEntity*>(it->second);
      std::string strBlockId = pcBlock->GetId().substr(6,2);
        UInt32 nBlockId = std::stoi(strBlockId);
      pcBlock->GetLEDEquippedEntity().Enable();
      pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
    if (nBlockId == 1) {
        pcBlock->GetLEDEquippedEntity().SetLEDColor(1,CColor::GREEN);
    }
    else  if (nBlockId == 3) {
        pcBlock->GetLEDEquippedEntity().SetLEDColor(0,CColor::GREEN);
    }
    else  if (nBlockId == 4) {
        pcBlock->GetLEDEquippedEntity().SetLEDColor(1,CColor::GREEN);
    }
    else  if (nBlockId == 6) {
        pcBlock->GetLEDEquippedEntity().SetLEDColor(0,CColor::GREEN);
    }

    else  if (nBlockId == 9) {
        pcBlock->GetLEDEquippedEntity().SetLEDColor(1,CColor::GREEN);
    }
    else  if (nBlockId == 11) {
        pcBlock->GetLEDEquippedEntity().SetLEDColor(0,CColor::GREEN);
    }
    else  if (nBlockId == 12) {
        pcBlock->GetLEDEquippedEntity().SetLEDColor(1,CColor::GREEN);
    }
    else  if (nBlockId == 14) {
        pcBlock->GetLEDEquippedEntity().SetLEDColor(0,CColor::GREEN);
    }


    else  if (nBlockId == 17) {
        pcBlock->GetLEDEquippedEntity().SetLEDColor(1,CColor::GREEN);
    }
    else  if (nBlockId == 19) {
        pcBlock->GetLEDEquippedEntity().SetLEDColor(0,CColor::GREEN);
    }
    else  if (nBlockId == 20) {
        pcBlock->GetLEDEquippedEntity().SetLEDColor(1,CColor::GREEN);
    }
    else  if (nBlockId == 22) {
        pcBlock->GetLEDEquippedEntity().SetLEDColor(0,CColor::GREEN);
    }


    else  if (nBlockId == 25) {
        pcBlock->GetLEDEquippedEntity().SetLEDColor(1,CColor::GREEN);
    }
    else  if (nBlockId == 27) {
        pcBlock->GetLEDEquippedEntity().SetLEDColor(0,CColor::GREEN);
    }
    else  if (nBlockId == 28) {
        pcBlock->GetLEDEquippedEntity().SetLEDColor(1,CColor::GREEN);
    }
    else  if (nBlockId == 30) {
        pcBlock->GetLEDEquippedEntity().SetLEDColor(0,CColor::GREEN);
    }


    unBlocksID += 1;
  }
}

/****************************************/
/****************************************/

// CVector3 CommTskLoopFunction::GetRandomPosition() {
//   Real temp;
//   Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
//   Real b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
//   Real c = m_pcRng->Uniform(CRange<Real>(-1.0f, 1.0f));
//   Real d = m_pcRng->Uniform(CRange<Real>(-1.0f, 1.0f));
//   // If b < a, swap them
//   if (b < a) {
//     temp = a;
//     a = b;
//     b = temp;
//   }
//   m_fDistributionRadius = 0.35;
//   Real fPosX = (c * m_fDistributionRadius / 2) + m_fDistributionRadius * cos(2 * -CRadians::PI_OVER_TWO .GetValue() * (a/b));
//   Real fPosY = (d * 1.4 * m_fDistributionRadius / 2) + 1.4 * m_fDistributionRadius * sin(2 * -CRadians::PI_OVER_TWO.GetValue() * (a/b));
//
//   return CVector3(fPosX, fPosY, 0);
// }
CVector3 CommTskLoopFunction::GetRandomPosition() {

  Real a;
  Real b;

  a = m_pcRng->Uniform(CRange<Real>(-0.455f, 0.455f));
  b = m_pcRng->Uniform(CRange<Real>(-0.705f, 0.705f));

  Real fPosX = a;
  Real fPosY = b;

  return CVector3(fPosX, fPosY, 0);
}

/****************************************/
/****************************************/

UInt32 CommTskLoopFunction::GetRandomTime(UInt32 unMin, UInt32 unMax) {
  UInt32 unStopAt = m_pcRng->Uniform(CRange<UInt32>(unMin, unMax));
  return unStopAt;

}

/****************************************/
/****************************************/

bool CommTskLoopFunction::IsEven(UInt32 unNumber) {
    bool even;
    if((unNumber%2)==0)
       even = true;
    else
       even = false;

    return even;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CommTskLoopFunction, "communication_tsk_loop_function");