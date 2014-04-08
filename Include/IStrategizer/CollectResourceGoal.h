///> [Serializable]
#ifndef COLLECTRESOURCEGOAL_H
#define COLLECTRESOURCEGOAL_H

#include "GoalEx.h"
#include "EntityClassExist.h"

namespace IStrategizer
{
    ///> class=CollectResourceGoal
    ///> parent=GoalEx
    class CollectResourceGoal : public GoalEx
    {
        OBJECT_SERIALIZABLE(CollectResourceGoal);

    public:
        CollectResourceGoal();
        CollectResourceGoal(const PlanStepParameters& p_parameters);
        GoalEx* GetSucceededInstance(const RtsGame &rtGame) const;
        
    protected:
        void InitializePostConditions();
        bool SuccessConditionsSatisfied(RtsGame& game);
    };
}
#endif // COLLECTRESOURCEGOAL_H
