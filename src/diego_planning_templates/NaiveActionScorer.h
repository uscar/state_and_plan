#include "ActionScorer.h"

namespace planning {
    /** ActionScorer that uses only the heuristic as a source of score */
    class NaiveActionScorer : public ActionScorer {
        double score(const Action& action) override;
    }
}
