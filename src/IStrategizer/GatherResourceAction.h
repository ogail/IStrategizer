///> [Serializable]
#ifndef GATHERRESOURCEACTION_H
#define GATHERRESOURCEACTION_H

#include "Action.h"
#include "And.h"
#include "Vector2.h"

namespace IStrategizer
{
    ///> class=GatherResourceAction
    ///> parent=Action
    class GatherResourceAction : public Action
    {
        OBJECT_SERIALIZABLE_P(GatherResourceAction, Action);

    public:
        GatherResourceAction();
        GatherResourceAction(const PlanStepParameters& p_parameters);
        bool AliveConditionsSatisfied(RtsGame& pRtsGame);
        bool SuccessConditionsSatisfied(RtsGame& pRtsGame);
        bool Equals(PlanStepEx* p_planStep);

    protected:
        bool Execute(RtsGame& pRtsGame, const WorldClock& p_clock);
        void InitializePreConditions();
        void InitializePostConditions();
        void FreeResources(RtsGame &game);
        void HandleMessage(RtsGame& pRtsGame, Message* p_msg, bool& p_consumed);

    private:
        TID m_gathererId;
        TID	m_resourceId;
        int	m_resourceAmount;
        bool m_gatherIssued;
        bool m_gatherStarted;
        float m_gatheredAmount;
    };
}

#endif	// GATHERRESOURCEACTION_H