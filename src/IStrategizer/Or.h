///> [Serializable]
#ifndef OR_H
#define OR_H

#include "CompositeExpression.h"

namespace IStrategizer
{
    ///> class=Or
    ///> parent=CompositeExpression
    class Or : public CompositeExpression
    {
        OBJECT_SERIALIZABLE_P(Or, CompositeExpression);

    public:
        Or() {}
        Or(const std::vector<Expression*>& p_expressions) : CompositeExpression(p_expressions) {}
        bool Evaluate(RtsGame& game);
    };
}

#endif // OR_H