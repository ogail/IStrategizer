#ifndef LEARNINGFROMHUMANDEMONSTRATION_H
#define LEARNINGFROMHUMANDEMONSTRATION_H

#include "CompositeExpression.h"
#include "CaseLearningHelper.h"
#include "CaseEx.h"
#include <vector>
#include <map>

namespace IStrategizer
{
    class GameTrace;
    class Action;
    class RetainerEx;
    class SubsetPlanData;

    const int TIME_STAMP_SIZE_EX = 1;

    typedef std::vector<PlanStepEx*> SequentialPlan;
    
    typedef std::vector<SubsetPlanData> SubsetPlanDataList;

    typedef std::map<unsigned, SubsetPlanDataList> SubplansMap;

    class SubsetPlanData
    {
    public:
        SubsetPlanData(unsigned index, const OlcbpPlan::NodeSet& nodes) : Index(index), Nodes(nodes) {}
        
        unsigned Index;
        OlcbpPlan::NodeSet Nodes;
    };

    class RawPlanEx
    {
    public:
        GoalEx* Goal;
        SequentialPlan sPlan;

        RawPlanEx(){}
        RawPlanEx(GoalEx* p_goal, SequentialPlan p_sPlan): Goal(p_goal), sPlan(p_sPlan){}
    };

    class RawCaseEx
    {
    public:
        RawPlanEx rawPlan;
        RtsGame* gameState;

        RawCaseEx() {}
        RawCaseEx(RawPlanEx p_rawPlan, RtsGame* p_gameState): rawPlan(p_rawPlan), gameState(p_gameState) {}
    };

    class CookedPlan
    {
    public:
        GoalEx* Goal;
        RtsGame* gameState;
        OlcbpPlan* pPlan;

        CookedPlan(){}
        CookedPlan(GoalEx* p_goal, OlcbpPlan* p_pPlan, RtsGame* p_gameState): Goal(p_goal), pPlan(p_pPlan), gameState(p_gameState) {}
    };

    class CookedCase
    {
    public:
        RawCaseEx* rawCase;
        OlcbpPlan* plan;

        CookedCase(){}
        CookedCase(RawCaseEx* p_rawCase, OlcbpPlan* p_plan): rawCase(p_rawCase), plan(p_plan) {}
    };

    class LearningFromHumanDemonstration
    {
    private:
        CaseLearningHelper* _helper;
        RetainerEx* _retainer;
        std::map<unsigned, RtsGame*> _gameStateMapping;

        void UnnecessaryStepsElimination(CookedCase* p_case);
        void HierarchicalComposition(std::vector<CookedPlan*>& p_cookedPlans) const;
        void IdenticalPlanDetection(std::vector<CookedPlan*> &p_cookedPlans) const;
        void RetainLearntCases(std::vector<CookedPlan*>& p_cookedPlans);
        bool Depends(CompositeExpression* p_candidateParent, CompositeExpression* p_candidateChild) const;
        bool IdenticalSequentialPlan(SequentialPlan left, SequentialPlan right) const;
        bool IsIdenticalPlan(OlcbpPlan* leftPlan, OlcbpPlan* rightPlan) const;
        bool IsSuperGraph(OlcbpPlan *pSuperGraph, OlcbpPlan *pSubGraph, OlcbpPlan::NodeSet& superGraphMatchedIds) const;
        bool SubplansDetection(std::vector<CookedPlan*>& p_cookedPlans) const;
        bool IsSubset(const OlcbpPlan::NodeSet superset,  const OlcbpPlan::NodeSet candidateSubset) const;
        CookedCase* DependencyGraphGeneration(RawCaseEx* p_rawCases);
        std::vector<RawCaseEx*> LearnRawCases(GameTrace::List p_traces);

    public:
        LearningFromHumanDemonstration(PlayerType p_player, PlayerType p_enemy);
        virtual ~LearningFromHumanDemonstration();
        void Learn();
        void Init() { _helper->Init(); }
    };
}

#endif // LEARNINGFROMHUMANDEMONSTRATION_H
