#ifndef GOALFACTORY_H
#define GOALFACTORY_H

#include <vector>
#include "MetaData.h"

namespace IStrategizer
{
    class GoalEx;

    class GoalFactory
    {
    private:
        GoalFactory() {};

    public:
        static GoalFactory& Instance() { static GoalFactory m_instance; return m_instance; }
        GoalEx* GetGoal(GoalType p_goalType, const PlanStepParameters& p_paramaters, bool p_initConditions = true);
        GoalEx* GetGoal(GoalType p_goalType, bool p_initConditions = true);
    };
#define g_GoalFactory IStrategizer::GoalFactory::Instance()
}

#endif // GOALFACTORY_H
