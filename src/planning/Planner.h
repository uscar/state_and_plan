namespace planning {
    /** The class responsible of coming up with the next action */
    class Planner {
    public:
        /** Generates the next action by looking at the current state */
        Action generateOnePlan();

    private:
        /** The object in charge of scoring an action */
        SelectedScorer as;
    }
}
