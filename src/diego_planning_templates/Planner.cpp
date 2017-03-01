
namespace planning {
    Action Planner::generateOnePlan() {
        std::vector<Action> actions;
        // TODO populate vector with possible actions

        // Find action with highest benefit score
        Action* maxAction = actions[0];
        double maxActionScore = as.score(actions[0]);
        for (Action& currAction : actions) {
            double currScore = as.score(action);
            if (maxActionScore < currScore) {
                maxAction = &currAction;
                maxActionScore = currScore;
            }
        }
        
        return maxAction;
    }
}
