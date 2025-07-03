/**
  * @file <loop-functions/IcraLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @license MIT License
  */

#include "CommAggLoopFunc.h"
#include <fstream>
#include <iostream>
#include <numeric>
using namespace std;
/****************************************/
/****************************************/

CommAggLoopFunction::CommAggLoopFunction() {
    m_unClock = 0;
    m_unStopTime = 0;
    m_unStopBlock = 0;
    m_fObjectiveFunction = 0;
    m_fPheromoneParameter = 0;
}

/****************************************/
/****************************************/

CommAggLoopFunction::CommAggLoopFunction(const CommAggLoopFunction& orig) {
}

/****************************************/
/****************************************/

CommAggLoopFunction::~CommAggLoopFunction() {}

/****************************************/
/****************************************/

void CommAggLoopFunction::Destroy() {

    m_tRobotStates.clear();
    m_tLEDStates.clear();
}

/****************************************/
/****************************************/

void CommAggLoopFunction::Init(TConfigurationNode& t_tree) {

    CoreLoopFunctions::Init(t_tree);
    TConfigurationNode cParametersNode;
    try {
    cParametersNode = GetNode(t_tree, "params");
    GetNodeAttributeOrDefault(cParametersNode, "maximization", m_bMaximization, (bool) false);
    GetNodeAttributeOrDefault(cParametersNode, "agg_corner", m_uAggCorner, (int) 0);
      // m_bMaximization = true;
    } catch(std::exception e) {
    }


    if (m_uAggCorner == 0){
        agg_number = GetRandomTime(1,3);
    }
    else if (m_uAggCorner == 1){
        agg_number = GetRandomTime(10,14);
    }
    else if (m_uAggCorner < 0){
        agg_number = -m_uAggCorner;
    }

    m_cUVColor.SetRed(128);
    m_cUVColor.SetGreen(0);
    m_cUVColor.SetBlue(128);

    InitRobotStates();
    InitPhormicaState();
    InitMocaState();

}

/****************************************/
/****************************************/

void CommAggLoopFunction::Reset() {
    CoreLoopFunctions::Reset();

    if (m_uAggCorner == 0){
        agg_number = GetRandomTime(1,3);
    }
    else if (m_uAggCorner == 1){
        agg_number = GetRandomTime(10,14);
    }
    else if (m_uAggCorner < 0){
        agg_number = -m_uAggCorner;
    }
    
    m_pcPhormica->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
    m_unClock = 0;
    m_unStopBlock = 0;
    m_unStopTime = 0;
    m_fObjectiveFunction = 0;

    m_tRobotStates.clear(); 
    m_tLEDStates.clear();

    InitMocaState();
    InitRobotStates();
    InitPhormicaState();
}

/****************************************/
/****************************************/

void CommAggLoopFunction::PostStep() {

    m_unClock = GetSpace().GetSimulationClock();
    // TimerControl();
    //  UpdateRobotPositions();
    GetRobotScore();
    MocaControl();
    UpdatePhormicaState();
   
    
    
    // LOG << m_fObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

void CommAggLoopFunction::PostExperiment() {
    
    // ofstream score;
    // score.open("/home/robotmaster/argos3-installation/habanero/habanero-loopfunctions/data/score_aggregation.txt", ofstream::app);
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

Real CommAggLoopFunction::GetObjectiveFunction() {
    if (m_bMaximization == true){
        return -m_fObjectiveFunction;
    }
    else {
        return m_fObjectiveFunction;
    }
}

/****************************************/
/****************************************/

void CommAggLoopFunction::MocaControl() {

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

void CommAggLoopFunction::TimerControl(){

    if (m_unClock == 1) {
        m_unStopTime = 600;
    }
}

/****************************************/
/****************************************/

void CommAggLoopFunction::GetRobotScore() {

    UpdateRobotPositions();

  Real unScore = 0;
  TRobotStateMap::iterator it1;
  TRobotStateMap::iterator it2;
  float distX=0;
  float distY=0;
  float centerX;
  float centerY;
  float sumX=0;
  float sumY=0;
  int count=0;

    for (it1 = m_tRobotStates.begin(); it1 != m_tRobotStates.end(); ++it1) {
        sumX += it1->second.cPosition.GetX();
        sumY += it1->second.cPosition.GetY();
        count += 1; 
    }
    centerX = sumX/count;
    centerY = sumY/count;
    // LOG<< "center x: "<<centerX<< std::endl;

    // LOG<< "center y: "<<centerY<< std::endl;

    for (it1 = m_tRobotStates.begin(); it1 != m_tRobotStates.end(); ++it1) {
        distX += abs(it1->second.cPosition.GetX() - centerX);
        distY += abs(it1->second.cPosition.GetY() - centerY);
    }
    // LOG<< "Total: "<<sqrt(pow(distX/count,2) + pow(distY/count,2))<< std::endl;

  m_fObjectiveFunction += sqrt(pow(distX/count,2) + pow(distY/count,2));

  
}

/****************************************/
/****************************************/

// Real CommAggLoopFunction::GetRobotOutScore() {
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

argos::CColor CommAggLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
    // if (c_position_on_plane.GetY() >= 0.25 && c_position_on_plane.GetY() <= 0.75+0.1 && c_position_on_plane.GetX() >= 0.25 && c_position_on_plane.GetX() <= 0.75+0.1){
    //     return CColor::GRAY30;
    // } 

    return CColor::WHITE;
}

/****************************************/
/****************************************/

void CommAggLoopFunction::UpdateRobotPositions() {
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

void CommAggLoopFunction::InitRobotStates() {

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
    }
}

/****************************************/
/****************************************/

void CommAggLoopFunction::InitPhormicaState() {

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

void CommAggLoopFunction::UpdatePhormicaState() {

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

    //     UInt32 unLEDTimer = itLED->second.unTimer;
    //     UInt32 unLEDCount = itLED->second.unCount;
        

    //     if (unLEDCount > 20){
    //         m_pcPhormica->GetLEDEquippedEntity().SetLEDColor(itLED->second.unLEDIndex,CColor::MAGENTA);
    //     }

    //     if (unLEDTimer == 0){
    //         m_pcPhormica->GetLEDEquippedEntity().SetLEDColor(itLED->second.unLEDIndex,CColor::BLACK);
    //         itLED->second.unCount = 0;
    //     }
    //     else {
    //         itLED->second.unTimer = unLEDTimer - 1;
    //     }
    }
}

/****************************************/
/****************************************/

void CommAggLoopFunction::InitMocaState() {

  CSpace::TMapPerType& tBlocksMap = GetSpace().GetEntitiesByType("block");
  UInt32 unBlocksID = 0;
  for (CSpace::TMapPerType::iterator it = tBlocksMap.begin(); it != tBlocksMap.end(); ++it) {
      CBlockEntity* pcBlock = any_cast<CBlockEntity*>(it->second);
      std::string strBlockId = pcBlock->GetId().substr(6,2);
        UInt32 nBlockId = std::stoi(strBlockId);
      pcBlock->GetLEDEquippedEntity().Enable();
      pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
    if (((nBlockId >= 0 && nBlockId <= 1) || (nBlockId >= 22 && nBlockId <= 23)) && agg_number == 10) {
        pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::CYAN);
    }
    else if ((nBlockId >= 4 && nBlockId <= 7 && agg_number == 11)) {
        pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::CYAN);
    }
    else if ((nBlockId >= 10 && nBlockId <= 13) && agg_number == 12) {
        pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::CYAN);
    }
    else if ((nBlockId >= 16 && nBlockId <= 19) && agg_number == 13) {
        pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::CYAN);
    }

    else if (((nBlockId >= 0 && nBlockId <= 1) || (nBlockId >= 22 && nBlockId <= 23)) && agg_number == 1) {
        pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::CYAN);
    }
    else if ((nBlockId >= 4 && nBlockId <= 7 && agg_number == 2)) {
        pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::CYAN);
    }
    else if ((nBlockId >= 10 && nBlockId <= 13) && agg_number == 1) {
        pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::CYAN);
    }
    else if ((nBlockId >= 16 && nBlockId <= 19) && agg_number == 2) {
        pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::CYAN);
    }

    unBlocksID += 1;
  }


//     CSpace::TMapPerType& tBlocksMap = GetSpace().GetEntitiesByType("cylinder");
//     UInt32 unBlocksID = 0;
//     for (CSpace::TMapPerType::iterator it = tBlocksMap.begin(); it != tBlocksMap.end(); ++it) {
//       CCylinderEntity* pcBlock = any_cast<CBlockEntity*>(it->second);
//       std::string strBlockId = pcBlock->GetId().substr(6,2);
//         UInt32 nBlockId = std::stoi(strBlockId);
//       pcBlock->GetLEDEquippedEntity().Enable();
//       pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
//     // if ((nBlockId >= 0 && nBlockId <= 1) || (nBlockId >= 22 && nBlockId <= 23)) {
//     //     pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::RED);
//     // }

//     // unBlocksID += 1;
//  }
}

/****************************************/
/****************************************/

// CVector3 CommAggLoopFunction::GetRandomPosition() {
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
CVector3 CommAggLoopFunction::GetRandomPosition() {

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

UInt32 CommAggLoopFunction::GetRandomTime(UInt32 unMin, UInt32 unMax) {
  UInt32 unStopAt = m_pcRng->Uniform(CRange<UInt32>(unMin, unMax));
  return unStopAt;

}

/****************************************/
/****************************************/

bool CommAggLoopFunction::IsEven(UInt32 unNumber) {
    bool even;
    if((unNumber%2)==0)
       even = true;
    else
       even = false;

    return even;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CommAggLoopFunction, "comm_agg_loop_function");
