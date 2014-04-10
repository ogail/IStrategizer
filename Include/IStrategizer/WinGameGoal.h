///> [Serializable]
#ifndef WINGAMEGOAL_H
#define WINGAMEGOAL_H

#include "GoalEx.h"

namespace IStrategizer
{
    ///> class=WinGameGoal
    ///> parent=GoalEx
    class WinGameGoal : public GoalEx
    {
        OBJECT_SERIALIZABLE(WinGameGoal);

    public:
        WinGameGoal();
        WinGameGoal(const PlanStepParameters& p_parameters);
        std::vector<GoalEx*> GetSucceededInstances(RtsGame &game);
        
    protected:
        void InitializePostConditions();
        bool SuccessConditionsSatisfied(RtsGame& game);
    };
}

#endif // WINGAMEGOAL_H