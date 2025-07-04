/**
  * @file <loop-functions/IcraLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @license MIT License
  */

#include "HeteroAggLoopFunc.h"

/****************************************/
/****************************************/

HabDecLoopFunction::HabDecLoopFunction() {
    m_unClock = 0;
    m_unStopTime = 0;
    m_unStopBlock = 0;
    m_fObjectiveFunction = 0;
    m_fPheromoneParameter = 0;
}

/****************************************/
/****************************************/

HabDecLoopFunction::HabDecLoopFunction(const HabDecLoopFunction& orig) {
}

/****************************************/
/****************************************/

HabDecLoopFunction::~HabDecLoopFunction() {}

/****************************************/
/****************************************/

void HabDecLoopFunction::Destroy() {

    m_tRobotStates.clear();
    m_tLEDStates.clear();
}

/****************************************/
/****************************************/

void HabDecLoopFunction::Init(TConfigurationNode& t_tree) {

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

    InitRobotStates();
    InitPhormicaState();
    InitMocaState();

}

/****************************************/
/****************************************/

void HabDecLoopFunction::Reset() {
    CoreLoopFunctions::Reset();

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

void HabDecLoopFunction::PostStep() {

    m_unClock = GetSpace().GetSimulationClock();
    GetRobotScore();
    TimerControl();
    MocaControl();
    UpdatePhormicaState();
    // LOG << m_unClock << std::endl;
}

/****************************************/
/****************************************/

void HabDecLoopFunction::PostExperiment() {

    if (m_bMaximization == true){
        LOG << -m_fObjectiveFunction << std::endl;
    }
    else {
        LOG << m_fObjectiveFunction << std::endl;

    }
}

/****************************************/
/****************************************/

Real HabDecLoopFunction::GetObjectiveFunction() {
    if (m_bMaximization == true){
        return -m_fObjectiveFunction;
    }
    else {
        return m_fObjectiveFunction;
    }
}

/****************************************/
/****************************************/

void HabDecLoopFunction::MocaControl() {

  CSpace::TMapPerType& tBlocksMap = GetSpace().GetEntitiesByType("block");
  UInt32 unBlocksID = 0;
  for (CSpace::TMapPerType::iterator it = tBlocksMap.begin(); it != tBlocksMap.end(); ++it) {
      CBlockEntity* pcBlock = any_cast<CBlockEntity*>(it->second);
        std::string strBlockId = pcBlock->GetId().substr(6,2);
        UInt32 nBlockId = std::stoi(strBlockId);
      pcBlock->GetLEDEquippedEntity().Enable();
      pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
    //   LOG << nBlockId << std::endl;
    if (m_unClock >= 0 && m_unClock <= 2000){
            if (nBlockId >= 0 && nBlockId <= 1 || nBlockId >= 14 && nBlockId <= 15) {
            pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::RED);
        }
    }
    else if (unBlocksID >= 2 && unBlocksID <= 5 && m_unClock > 2000 && m_unClock <= 4000) {
        pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::YELLOW);
    }
    else if (unBlocksID >= 6 && unBlocksID <= 9 && m_unClock >4000 && m_unClock <= 6000) {
        pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::GREEN);
    }
    else if (unBlocksID >= 10 && unBlocksID <= 13 && m_unClock > 6000 && m_unClock <= 8000) {
        pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLUE);
    }


    unBlocksID += 1;
  }
}

/****************************************/
/****************************************/

void HabDecLoopFunction::TimerControl(){

    if (m_unClock == 1) {
        m_unStopTime = GetRandomTime(850, 901);
    }
}

/****************************************/
/****************************************/

void HabDecLoopFunction::GetRobotScore() {

  UpdateRobotPositions();

  Real unScore = 0;
  TRobotStateMap::iterator it;
  for (it = m_tRobotStates.begin(); it != m_tRobotStates.end(); ++it) {
      if (it->second.cPosition.GetX() <= -0.125 && it->second.cPosition.GetY() <= -0.375){
        unScore+=1;
      }
      else if(it->second.cPosition.GetX() >= 0.125 && it->second.cPosition.GetY() >= 0.375){
        unScore+=2;
      }

  }

  m_fObjectiveFunction += unScore;
}

/****************************************/
/****************************************/

// Real HabDecLoopFunction::GetRobotOutScore() {
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

argos::CColor HabDecLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {

    return CColor::WHITE;
}

/****************************************/
/****************************************/

void HabDecLoopFunction::UpdateRobotPositions() {
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

void HabDecLoopFunction::InitRobotStates() {

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

void HabDecLoopFunction::InitPhormicaState() {

    CSpace::TMapPerType& tPhormicaMap = GetSpace().GetEntitiesByType("phormica");
    CVector2 cLEDPosition(0,0);
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
        }
    }
}

/****************************************/
/****************************************/

void HabDecLoopFunction::UpdatePhormicaState() {

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
            
 
           
            UInt32 swarmId = it->second.unId;
            //if (d <= m_fPheromoneParameter) {
            if (d <= fPheromone) {
                itLED->second.unTimer = 200; // Pheromone decay time
                // m_pcPhormica->GetLEDEquippedEntity().SetLEDColor(itLED->second.unLEDIndex,CColor::RED);
                if (swarmId == 1){
                    m_pcPhormica->GetLEDEquippedEntity().SetLEDColor(itLED->second.unLEDIndex,CColor::GREEN );
                }
                if (swarmId == 2){
                    m_pcPhormica->GetLEDEquippedEntity().SetLEDColor(itLED->second.unLEDIndex,CColor::BLUE);
                }
                if (swarmId == 3){
                    m_pcPhormica->GetLEDEquippedEntity().SetLEDColor(itLED->second.unLEDIndex,CColor::YELLOW);
                }
                if (swarmId == 4){
                    m_pcPhormica->GetLEDEquippedEntity().SetLEDColor(itLED->second.unLEDIndex,CColor::RED);
                }
                
            }
        }
        

        UInt32 unLEDTimer = itLED->second.unTimer;
        if (unLEDTimer == 0){
            m_pcPhormica->GetLEDEquippedEntity().SetLEDColor(itLED->second.unLEDIndex,CColor::BLACK);
        }
        else {
            itLED->second.unTimer = unLEDTimer - 1;
        }
    }
}

/****************************************/
/****************************************/

void HabDecLoopFunction::InitMocaState() {

  CSpace::TMapPerType& tBlocksMap = GetSpace().GetEntitiesByType("block");
  UInt32 unBlocksID = 0;
  for (CSpace::TMapPerType::iterator it = tBlocksMap.begin(); it != tBlocksMap.end(); ++it) {
      CBlockEntity* pcBlock = any_cast<CBlockEntity*>(it->second);
        std::string strBlockId = pcBlock->GetId().substr(6,2);
        UInt32 nBlockId = std::stoi(strBlockId);
      pcBlock->GetLEDEquippedEntity().Enable();
      pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
  }
}

/****************************************/
/****************************************/

// CVector3 HabDecLoopFunction::GetRandomPosition() {
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
CVector3 HabDecLoopFunction::GetRandomPosition() {

  Real a;
  Real b;

  a = m_pcRng->Uniform(CRange<Real>(-0.255f, 0.255f));
  b = m_pcRng->Uniform(CRange<Real>(-0.505f, 0.505f));

  Real fPosX = a;
  Real fPosY = b;

  return CVector3(fPosX, fPosY, 0);
}

/****************************************/
/****************************************/

UInt32 HabDecLoopFunction::GetRandomTime(UInt32 unMin, UInt32 unMax) {
  UInt32 unStopAt = m_pcRng->Uniform(CRange<UInt32>(unMin, unMax));
  return unStopAt;

}

/****************************************/
/****************************************/

bool HabDecLoopFunction::IsEven(UInt32 unNumber) {
    bool even;
    if((unNumber%2)==0)
       even = true;
    else
       even = false;

    return even;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(HabDecLoopFunction, "hetero_agg_loop_function");
