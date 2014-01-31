#include "DestroyEntityTypeGoal.h"

#include "Not.h"
#include "And.h"
#include "EntityClassExist.h"
#include <cstdlib>
#include <ctime>
#include <cassert>
#include "RtsGame.h"
#include "GamePlayer.h"
#include "GameEntity.h"

using namespace IStrategizer;

DestroyEntityTypeGoal::DestroyEntityTypeGoal() : GoalEx(GOALEX_DestroyEntityType)
{
    /*_forceDescription = ForceDescriptionEx(FORCESIZE_SmallForce, PRCNT_0, PRCNT_0, PRCNT_100, PRCNT_0, PRCNT_0, PRCNT_0);
    _params[PARAM_ForceSizeId] = FORCESIZE_START;
    _params[PARAM_AttackTypeId] = ATTACK_START;*/
}
//----------------------------------------------------------------------------------------------
DestroyEntityTypeGoal::DestroyEntityTypeGoal(const PlanStepParameters& p_parameters): GoalEx(GOALEX_DestroyEntityType, p_parameters)
{
     // FIXME: There should be a commander that specify force description in details
    //_forceDescription = ForceDescriptionEx(FORCESIZE_SmallForce, PRCNT_0, PRCNT_0, PRCNT_100, PRCNT_0, PRCNT_0, PRCNT_0);
}
//----------------------------------------------------------------------------------------------
void DestroyEntityTypeGoal::InitializePostConditions()
{
    //m_terms.push_back(new CheckEntityClassAttribute(PLAYER_Enemy, ECLASS_END, ECATTR_Count, RELOP_Equal, 0));

    //for (int index = 0; index < _params[PARAM_Value]; ++index)
    //{
    // //FIXME : LFHD use this condition
    // //m_terms.push_back(new CheckPositionFilterCount(PLAYER_Enemy, FILTER_AnyUnit, RELOP_Equal, 0, PositionFeatureVector::Null()));
    //}

    /*this gaol works on just one entity or we can take parameters of force size*/

    vector<TID>            entityIds;
    GameEntity            *entity;
    int                    numberOfUnits = 0;

    //get the number of enemy units from the given type.
    g_Game->Enemy()->Entities(entityIds);   
    for (size_t i = 0, size = entityIds.size(); i < size; ++i)
    {
      entity = g_Game->Enemy()->GetEntity(entityIds[i]);
      if (entity->Type() == _params[PARAM_TargetEntityClassId])
      {
          numberOfUnits++;
      }
    }
    EntityClassType targetType = (EntityClassType)_params[PARAM_TargetEntityClassId];
    _postCondition = new Not(new EntityClassExist(PLAYER_Enemy, targetType, numberOfUnits, true));
}   
//----------------------------------------------------------------------------------------------
void DestroyEntityTypeGoal::Copy(IClonable* p_dest)
{
    GoalEx::Copy(p_dest);

    DestroyEntityTypeGoal* m_dest = static_cast<DestroyEntityTypeGoal*>(p_dest);

    m_dest->_forceDescription   = _forceDescription;
}
//----------------------------------------------------------------------------------------------
bool DestroyEntityTypeGoal::SuccessConditionsSatisfied(RtsGame& pRtsGame)
{
    return false;
}