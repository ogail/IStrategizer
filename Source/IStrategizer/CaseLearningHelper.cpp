#include "CaseLearningHelper.h"
#include "DataMessage.h"
#include "GameStateEx.h"
#include "MessagePump.h"
#include "IStrategizerException.h"
#include "RtsGame.h"
#include "SVector.h"
#include "GoalFactory.h"

using namespace IStrategizer;
using namespace std;

CaseLearningHelper::CaseLearningHelper()
{
    g_MessagePump.RegisterForMessage(MSG_GameActionLog, this);
    g_MessagePump.RegisterForMessage(MSG_GameEnd, this);
    g_MessagePump.RegisterForMessage(MSG_EntityDestroy, this);

    for(unsigned i = START(GoalType); i < END(GoalType); ++i)
    {
        m_goals.push_back(g_GoalFactory.GetGoal((GoalType)i, true));
    }
}
//--------------------------------------------------------------------------------------------------------------------------------------------
std::vector<GoalEx*> CaseLearningHelper::GetSatisfiedGoals() const
{
    vector<GoalEx*> satisfiedGoals;

    for (size_t i = 0; i < m_goals.size(); ++i)
    {
        GoalEx* goal = m_goals[i]->GetSucceededInstance(*g_Game);

        if (goal)
        {
            satisfiedGoals.push_back(goal);
        }
    }

    return satisfiedGoals;
}
//--------------------------------------------------------------------------------------------------------------------------------------------
void CaseLearningHelper::NotifyMessegeSent(Message* p_message)
{
    DataMessage<GameTrace>* pTraceMsg = nullptr;
    GameTrace trace;
    bool dummy = false;

    if (p_message == nullptr)
        throw InvalidParameterException(XcptHere);

    for (size_t i = 0; i < m_goals.size(); ++i)
    {
        m_goals[i]->HandleMessage(*g_Game, p_message, dummy);
    }

    switch(p_message->MessageTypeID())
    {
    case MSG_GameActionLog:
        pTraceMsg = reinterpret_cast<DataMessage<GameTrace>*>(p_message);

        if (pTraceMsg ->Data() == nullptr)
            throw InvalidParameterException(XcptHere);

        trace = *pTraceMsg->Data();
        m_observedTraces.push_back(trace);
        m_goalMatrix[p_message->GameCycle()] = GetSatisfiedGoals();

        LogInfo("Received game trace for action=%s", Enums[trace.Action()]);
        break;

        case MSG_GameEnd:
            LogInfo("Received game end mmessage");
            m_goalMatrix[p_message->GameCycle()] = GetSatisfiedGoals();
            break;
    }
}

