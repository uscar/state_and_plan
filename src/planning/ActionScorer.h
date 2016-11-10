namespace planning {
    /** Abstract base class for classes that give actions a score */
    class ActionScorer {
    public:
        virtual double score(const Action& action) = 0;
    }
}
