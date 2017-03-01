#ifndef START_HEADER_H_
#define START_HEADER_H_





//MAIN TODOS
//Finish collision detection engine
//Check the collision state logic and add a method for states to ask each other whether they need to add a collision
//(separate from the collision engine)
//keep collision engine general enough to be able to handle quad
//add timeout state and randomnoise state, as well as timeout and randomnoise clocks
//double check state logic
//find a way to check for de-collisions
//make sure to add a little delay in the stickbots before transitioning from stopped to moving
//transfer const values
//collision engine make sure tests height of thing it'd run into (can use knowledge of it being a roomba)
//finish initializing quad and quad states
//link to eigen
//add other state specific values to roomba box
//find safe landing areas
//add standard collision check
//change it so deleted things re-add parents to queue instead of reinitialize because that feels a lot cleaner


//maybe add something to malloc tons of memory beforehand like an array of roomba states/state value container,
//and then just keep malloc-ing more of those


//maybe delete sufficiently old memories? we don't need to know that far into the past. It would make everything more general
//find ways of combining sims to keep value relevant, or allow for a 
//certain degree of flexibility in order to keep things reasonable
//flexibility will also allow for errors to occur in system
//allow for arbitrary changes, don't try to predict everything


//for mem management:
//keep 2 arrays, same size, one of datamembers and one of unsigned shorts
//have unsigned short array start with each element as its index
//assign each index from the second array referencing the first array as IDs into the first array?
//perhaps just get ID from list pointer start minus index pointer
//have 2 numbered (unsigned short) iterators into second array, one at start to allocate and one at end to deallocate
//move both numbers cyclically, each allocation moving the first one up, each deallocation moving the second one up. 
//Distance between them on each side indicates free memory vs used
//whenever a memory is freed (on destruction), put that ID into the second array at
//the second iterator spot and increment, to keep track of free mem. Each index first iterator comes to will be free.
//free memory in const time
//on destruction, put ID bac
//delete history past x seconds

//idea: add 'diversifier', basically takes old states and tries to find new paths up to current observations to diversify filter
//would be fairly low memory but potentially expensive processing-wise


//NOTES ABOUT CODE:

//states do not start out with all their information, and become initialized/accepted later
//once they become initialized, they get a vals object that contains extra information about them and methods

//a nullptr child for an initialized state indicates the state has no ending
//thus, getEndTime() on these will yield the round end time
//otherwise, state end time is calculated as the next state's start time.
//by default that's the terminal state, unless another state happens first
//in the event another state happens first, any dependent states for this that begin
//after the new end state starts are erased









//TODO: remove abundant error checks as code matures to streamline it?

//a lot of this code is kind of poorly encapsulated, but it's readable and functional so
#define _USE_MATH_DEFINES
#include <memory>
#include <vector>
#include <map>
#include <set>
#include <list>
#include <algorithm>
#include <cmath>
#include <queue>
#include <iostream>

//I really hate cast syntax.
//#define cast(type,name) ((type)name)
//#define cast(type,name) reinterpret_cast<type>(name)


//generic vector math implementation that took like ~20 mins to write
//only using for testing
//TODO link to eigen lib
namespace spaceMath {
	struct vector2 { //i would simd this or something but I intend to replace it anyway
		double x;
		double y;
		vector2() : x(0.0), y(0.0) {}
		vector2(double x_, double y_) :
			x(x_),y(y_) {}
		double square_components() {
			return x*x + y*y;
		}
		double length() {
			return sqrt(square_components());
		}
		void normalize() {
			double l = length();
			x /= l;
			y /= l;
		}
		vector2 normalized() {
			vector2 copy(*this);
			copy.normalize();
			return copy; //such copy
		}
		vector2(double a) { //angle constructor
			x = acos(a);
			y = asin(a);
		}
		double getAngle() {
			return atan2(y, x);
		}
		vector2 operator*(double s) {
			vector2 out(*this);
			out.x *= s;
			out.y *= s;
		}
		double dot(vector2 & other) {
			return x*other.x + y*other.y;
		}
		vector2 operator-(vector2 & other) {
			vector2 out(*this);
			out.x -= other.x;
			out.y -= other.y;
		}
		vector2 operator+(vector2 & other) {
			vector2 out(*this);
			out.x += other.x;
			out.y += other.y;
		}
	};

	struct vector3 : public vector2 {
		double z;

		vector3(double x_, double y_, double z_) :
			vector2(x_,y_), z(z_) {}
		vector3() : z(0.0) {}

		//don't wanna make things virtual ok
		//i like my pointers to be what I pretend they are
		double square_components() {
			return x*x + y*y + z*z;
		}
		double length() {
			return sqrt(square_components()); 
		}
		void normalize() {
			double l = length();
			x /= l;
			y /= l;
			z /= l;
		}
		vector3 normalized() {
			vector3 copy(*this);
			copy.normalize();
			return copy;
		}
		vector3 operator*(double s) {
			vector3 out(*this);
			out.x *= s;
			out.y *= s;
			out.z *= s;
		}
		double dot(vector3 & other) {
			return x*other.x + y*other.y + z*other.z;
		}
		vector3 operator-(vector3 & other) {
			vector3 out(*this);
			out.x -= other.x;
			out.y -= other.y;
			out.z -= other.z;
		}
		vector3 operator+(vector3 & other) {
			vector3 out(*this);
			out.x += other.x;
			out.y += other.y;
			out.z += other.z;
		}
	};

	vector2 operator*(double s, vector2 & v) {
		return v*s;
	}

	vector3 operator*(double s, vector3 & v) {
		return v*s;
	}

	struct qPose {
		vector3 position;
		double orientation;
	};

	struct rPose {
		vector2 position;
		double orientation;
	};

}

//--------------------------------------------------------------------PLANNER LIB START

namespace plannerlib {


	//##enum definitions##


	enum class robotType : unsigned char { Roomba, Quad, Obstacle };

	enum class stateType : unsigned char { //perhaps make these bot type localized?
		//generic states
		Stopped,     //stopped because match has yet to start
		OutOfBounds, //serves as 'failsafe' mode for quad, scoring mode for roombas

		//normal roomba states
		Run,
		FullRotate,
		QuarterRotate,
		NoiseRun,

		//obstacle states
		ObstacleRun,
		ObstacleStopped,

		//quad states
		Flying,
		TapLanding,
		Tapping,
		Landing,
		Landed,
		Lifting, //quad is swole af
		NumStates //for utility, always at end
	};

	//##define constant members##


	//roomba software behavior parameters


	/*//-----------------------------------from old implementation-----------------------

	//roomba speed
	const double robotSpeed = (double)330 / (double)1000; // m/s
														  // Time between trajectory noise injections
	const double noiseInterval = (double)5000 / (double)1000;
	// Time between auto-reverse
	const double reverseInterval = (double)20000 / (double)1000;
	// Time needed to affect trajectory
	const double noiseLength = (double)850 / (double)1000;
	// Time needed to reverse trajectory
	const double reverseLength = (double)2456 / (double)1000; // .33/2 m/s * pi * wheelbase / 2
															  // Time needed to spin 45 degrees
	const double topTouchTime = reverseLength / 4;

	//physical system constants
	const int numroombas = 10;
	const double roombaDiameter = 0.34;
	const double wheelbase_width = 0.25798379655423864; //worked backwards to this

														//const double FOV; //we need this value to find the vield of view as a function of height

														//-----------------------PARAMETERS---------------------------------------------

														//debug stuff
	const bool bestmove_test = false;

	//prediction parameters
	const int branching = 5;
	double predictUntil = 15.0; //10s predictive capacity?

								//simulation parameters
	const int num_sims = 10;//we'll find a better way to manage the simulations later
	const double runendtime = 600; //default state end time

								   //quad interaction parameters
	const double quadSpeedEst = 3.0; //m/s
	const double quadvSpeedEst = 1;
	const double quadvSpeedEstInteracting = -quadSpeedEst / 3;
	const double topTouchTimereq = 100;
	const double bumperTimereq = 100;
	const double roombaTapHeight = 0.05; //just a guess, i believe
	const double roombaBumpHeight = 0.0;

	//quad behavior parameters
	const double cruiseHeight = 1.0;

	//calculation parameters
	const double newton_accuracy = 0.1; //within 10 cm for now
	const int newton_iterations = 0;


	//---------------------------------------------------------------------------------------------*/



	//roomba speed
	const double roombaSpeed = (double)330 / (double)1000; // m/s
	// Time between trajectory noise states
	const double noiseInterval = (double)5000 / (double)1000;
	// Time between auto-reverse
	const double reverseInterval = (double)20000 / (double)1000;
	// Time needed to affect trajectory
	const double noiseLength = (double)850 / (double)1000;
	// Time needed to reverse trajectory
	const double reverseLength = (double)2456 / (double)1000;
	//obstacle wheel speed offset from straight to go in a circle
	const double obstacleSpeedOffset = (double)9 / (double)1000;
	// Time needed to spin 45 degrees
	const double topTouchTime = reverseLength / 4;
	const unsigned char maxLevels = 3;
	const double roombaDiameter = 0.34;
	const double arcDiameter = 0.34001637879;
	const double roombaHeight = 1234; //TODO: correct value
	const double runEndTime = 600; //round ends after ten minutes
	const unsigned short storageSize = 1000; //size of storage array (TODO: implement)

	//TODO move these values to implementation
	//const double zeroRadius = 0.02; //somewhat arbitrary;
	const double fudgeFactor = 0.001; //also somewhat arbitrary
	const double NANd = (double)NAN; //internal linkage to avoid confusion since this is not an important constant
	//max val for rng generator
	const int randomMax = 64;
	inline bool nearlyZero(double & toTest) { //useful
		return abs(toTest) < fudgeFactor;
	}


	//useful vals
	const double roombaDiameterSquared = roombaDiameter*roombaDiameter;



	//##state property stuff##
	//so we can store state properties in one place without a lot of mem usage
	//state properties here are used to optimize things like collision detection without
	//making the code awful with a bunch of inheritance,switch-case checks or ugly conditionals
	/*struct statePropertyContainer {
		bool staticCheck; //do we need to check for 
		bool dynamicCheck;
	};

	const statePropertyContainer stateProperties[stateType::NumStates] { //maybe put this in its own file

	}*/

	//##generic classes and structs##

	class robot;
	class robotState;
	class world;
	class roomba;
	class quad;

	//observation input class
	struct observation {
		double time;
		struct observedState {
			std::pair<double, double> coords;
			double orientation;
			double probability;
		};
		std::vector<observedState> seen;
	}; //end observation

	struct queueWrapper { //level of indirection to allow nullifying queue elements
		queueWrapper(robotState * state_,double atTime_) : state(state_), atTime(atTime_) {}
		double atTime;
		robotState * state;
		bool operator<(const queueWrapper & other) {
			return atTime < other.atTime;
		}
	}; //end queueWrapper

	struct Sorter { //sorts the priority queue
		bool operator() (queueWrapper * a, queueWrapper * b) {
			return (*b) < (*a); //reversed (smallest to greatest)
		}
	};


	//##typedefs##
	//so changes of type won't drastically modify codebase

	using queueType = std::priority_queue<queueWrapper*, std::vector<queueWrapper*>, Sorter>;
	using vector2 = spaceMath::vector2;
	using vector3 = spaceMath::vector3;
	using roombaPose = spaceMath::rPose;
	using quadPose = spaceMath::qPose;

	//##main classes##
	//---------------------------------------------------------------------ROBOT
	class robot {

	friend class robotState;

	public:

		robot(world * currentWorld_) {
			currentWorld = currentWorld_;
			//initialize stuff
		}

		robotState * getStateAt(double time) {
			//check boundary conditions here
			if (time < 0) return nullptr;
			if (time >= endState->getEndTime()) return nullptr;

			return recentState->getStateAt(time);
		}
		robotState * getStartState() {
			return startState;
		}
		robotState * getRecentState() { //is this necessary?
			return recentState;
		}
		robotState * getEndState() {
			return endState;
		}
		robotState * getCurrentState() {
			return currentState;
		}
		world * getCurrentWorld() { //is this necessary?
			return currentWorld;
		}
		robotType getType() {
			return type;
		}
		void pushState(robotState * toEvaluate) { //not sure why this is necessary
			toEvaluate->wrapper = new queueWrapper(toEvaluate,toEvaluate->getStartTime());
			currentWorld->pushToQueue(toEvaluate->wrapper);
		}
		void checkCollisions(robotState * toEvaluate) {
			currentWorld->addCollisionStates(toEvaluate);
		}

		bool isQuadrotor() {
			return type == robotType::Quad;
		}

		bool isObstacle() {
			return type == robotType::Obstacle;
		}

		unsigned char getID() {
			return ID;
		}

	protected:
		robotState * startState;
		robotState * recentState;
		robotState * currentState;
		robotState * endState;
		//robotState * genericState;
		world * currentWorld;
		robotType type;

		//collision properties (should be set by member)
		double diameter;
		double height; //constant for both quad and roomba
		unsigned char ID;
		
	}; //end robot

	//------------------------------------------------------------------ROBOT_STATE
	class robotState {
	friend class robot;
	public:
		struct robotStateVals {
			std::set<robotState *> dependentStates;
			robotState * child;
			//TODO maybe put these methods back
			//i am sorry, it's just too darn slow
			/*virtual void get2DPositionAtTime(double atTime,vector2 & out) = 0;
			virtual void get3DPositionAtTime(double atTime,vector3 & out) = 0;
			virtual void getOrientationAtTime(double atTime,double & out) = 0;*/
		}; //end robotState::robotStateVals

		robotState(robotState * parent_,robotState * dependency_,double startTime_) { //other args too
			parent = parent_;
			parent->addDependent(this);
			dependency = dependency_;
			if (dependency != nullptr) {
				dependency->addDependent(this);
			}
			startTime = startTime_;
		}

		virtual ~robotState() {//I'm sorry
			//there is almost certainly a cleaner way to do this
			//using shared_ptr or something but I really can't be bothered to do it rn			
			deleting = true;
			
			//deal with possible dependency
			if (dependency != nullptr) {
				dependency->vals->dependentStates.erase(this);
			}

			if (isInitialized()) { //if vals exists
				//deal with children
				for (robotState * depState : vals->dependentStates) { //this includes the child
					depState->dependency = nullptr;
					delete depState;
				}
				//deal with parent
				if (parent != nullptr) {
					if (container->recentState == this) {
						container->recentState = parent;
					}
					if (container->currentState == this) {
						container->currentState = parent;
					}
					//tell parent to reinitialize 
					if (!(parent->deleting)) {
						container->endState = parent;
						if (this == parent->getChild()) { //this is important, keeps from re-initializing on queue pops n stuff
							parent->initialize();
						}

						//parent->vals->child = nullptr; //don't do this because child is needed to do.. stuff
					}
				}
				else {
					std::cerr << "deleting *what looks like* a base state" << std::endl;
				}
				delete vals; //delete optimization vals
			}

			//invalidate possible queue wrapper
			if (wrapper != nullptr) {
				wrapper->state = nullptr; //invalidate queue index
			}
			
		}

		//keep in mind usage of nullptr as generic newChild
		virtual void replaceChild(robotState * newChild) {
			if (isInitialized()) {
				robotState * childOld = vals->child;
				vals->child = newChild; //must switch away if we want to delete old child because of weird deletion behavior
				if (newChild != nullptr) { //if not a generic endmarker
					if (childOld != newChild) {//if we're replacing a different existing child
						for (robotState * dependent : vals->dependentStates) { //delete all dependents that come chronologically after the new state starts (no longer our dependents)
							if (dependent->startTime >= newChild->startTime) { //^ this includes the old child state
								delete dependent;
							}
						}
					}
					vals->dependentStates.insert(newChild);
				}
			}
			else {
				std::cerr << "Trying to give an uninitialized node a child" << std::endl;
				return;
			}
		}

		void addDependent(robotState * dependent) {
			if (isInitialized()) {
				vals->dependentStates.insert(dependent);
			}
			else {
				std::cerr << "Trying to add a dependent to uninitialized node" << std::endl;
			}
		}

		virtual void initialize() = 0 { //resolves stuff with generic container members/children and parents
			container->endState = this; //probably?

			//re-initialize dependencies on this node

			//clean up possible dependent states

			if (parent != nullptr) { //if not a starter node
				parent->replaceChild(this);
			}

			for (robotState * dependent : vals->dependentStates) { //if any dependents because we're the result of weird deletion madness
				//if (dependent != vals->child) { //technically, in the deletion madness case the child has yet to be deleted, because IT invokes this method
				if (!dependent->deleting) { //wow this is so much more conceptually simple AND safe.
					delete dependent;
				}
			}

			vals->dependentStates.clear();
			wrapper = nullptr;
		
			//now we go and insert states in the other version
			
		}

		robotState * getDependency() {
			return dependency;
		}

		robotState * getParent() {
			return parent;
		}

		robot * getContainer() {
			return container;
		}

		robotState * getChild() {
			if (!isInitialized()) {
				std::cerr << "Attempted to access child in uninitialized state" << std::endl;
				return nullptr;
			}
			else {
				return vals->child;
			}
		}

		robotState * getStateAt(double time) {
			if (!isInitialized()) {
				std::cerr << "getStateAt boundary check failure" << std::endl;
				return nullptr;
			}
			if (time >= getStartTime()) {
				if (time < getEndTime()) {
					container->recentState = this; //most recent state
					return this; //in range
				}
				else {
					return vals->child; //child range
				}
			}
			else {
				return parent;
			}
		}

		virtual double testCollision(robotState * other, std::list<robotState*> * collisionStates, double collidesAt = -1.0) = 0;

		stateType getType() {
			return type;
		}

		double getStartTime() {
			return startTime;
		}

		double getEndTime() {
			if (getChild() == nullptr) {

			}
			return getChild()->getStartTime();
		}

		/*void get2DPositionAtTime(double atTime,vector2 & out) { //TODO: when debug messages are out, make these inline?
			if (!isInitialized()) {
				std::cerr << "Attempted to access position in uninitialized state" << std::endl;
			}
			else {
				vals->get2DPositionAtTime(atTime, out);
			}
		}

		void get3DPositionAtTime(double atTime, vector3 & out) { //TODO: when debug messages are out, make these inline?
			if (!isInitialized()) {
				std::cerr << "Attempted to access position in uninitialized state" << std::endl;
			}
			else {
				vals->get3DPositionAtTime(atTime, out);
			}
		}

		void getOrientationAtTime(double atTime, double & out) { //TODO: when debug messages are out, make these inline?
			if (!isInitialized()) {
				std::cerr << "Attempted to access orientation in uninitialized state" << std::endl;
			}
			else {
				vals->getOrientationAtTime(atTime, out);
			}
		}*/

		/*void queueSelf() { //kind of gross encapsulation but easier to handle (do we need this?)
			container->pushState(this);
		}*/

		bool isInitialized() {
			return (vals != nullptr);
		}

	protected:
		robot * container;
		queueWrapper * wrapper;
		robotState * parent;
		robotState * dependency;
		robotStateVals * vals = nullptr; //must be initialized to nullptr
		stateType type;
		bool deleting = false; //this feels gross, also must be initialized
		double startTime;
	}; //end robotState


	//---------------------------------------------------------------------WORLD
	class world {
	public:
		void simTo(double time) {

		}

		virtual ~world() {
			for (robot * bot : robots) {
				delete bot; //delete everything
			}
			
		}

		world() {

			//initialize field

			quadPose init = quadPose(); //TODO: put something realistic for this
			quadRotor = new quad(init,this); //quad has ID 0
			robots.push_back(quadRotor);

			double roombaAngleOffset = 0.0; //maybe make this a random value and use that sampling to determine the original angle? (between 0 and M_PI/5.0)
			double obstacleAngleOffset = 0.0; //same as above (between 0 and M_PI/2.0)

			for (int i = 1; i <= 10; ++i) { //IDs 1 through 10

				roombaPose startPose;
				double angle = ((double)(i - 1))*(M_PI / 5.0) + roombaAngleOffset;
				//no scaling because roombas are already 1 meter out
				startPose.position.x = 10.0 + acos(angle);
				startPose.position.y = 10.0 + asin(angle);
				startPose.orientation = angle;

				roomba * thisRoomba = new roomba(startPose, false, i, this);
				roombas.push_back(thisRoomba);
				robots.push_back(thisRoomba);

			}

			for (int i = 11; i <= 14; ++i) { //IDs 11 through 14

				roombaPose startPose;
				double angle = ((double)(i - 1))*(M_PI / 2.0) + obstacleAngleOffset;
				//no scaling because roombas are already 1 meter out
				startPose.position.x = 10.0 + 5.0*acos(angle);
				startPose.position.y = 10.0 + 5.0*asin(angle);
				startPose.orientation = angle;

				roomba * thisRoomba = new roomba(startPose, true, i, this);
				roombas.push_back(thisRoomba);
				robots.push_back(thisRoomba);

			}

			//initialize other pertinent world values

			lastObservationTime = 0.0;
			latestEvaluation = 0.0;

		}

		void incorporate(observation * observed) {
			lastObservationTime = observed->time;
			//TODO: other stuff
		}

		double getLastObservationTime() {
			return lastObservationTime;
		}

		double getLatestEvaluation() {
			return latestEvaluation;
		}

		void popFromQueue() {
			//get top element and pop
			queueWrapper * nextEvent = queue.top();
			queue.pop();

			latestEvaluation = nextEvent->atTime; //set most recent eval
			robotState * toEvaluate, * depends;
			toEvaluate = nextEvent->state;

			delete nextEvent; //clean up wrapper

			//evaluate it
			if (toEvaluate == nullptr) return; //check for invalid state
			depends = toEvaluate->getDependency();
			if (depends != nullptr) { //if it depends on another state
				if (toEvaluate->getStartTime() > depends->getEndTime()) { //if dependency no longer valid for state
					return;												  //(has to be greater than, not >= or will fali on collision
				}
			}
			//if it's already inserted or earlier (more valid) than the inserted state;
			//hopefully it actually skips others if first is true
			if ((toEvaluate->getParent() == nullptr) || (toEvaluate->getStartTime() < toEvaluate->getParent()->getEndTime()) || (toEvaluate == toEvaluate->getParent->getChild())) {
				toEvaluate->initialize(); //accept as a valid state
				addCollisionStates(toEvaluate);
			}
			else{
				delete toEvaluate; //delete failed state
			}
			
		}

		void pushToQueue(queueWrapper * toInsert) {
			queue.push(toInsert);
		}

		void addCollisionStates(robotState * state) {
			robotState * current;
			double collisionTime;
			for (robot * bot : robots) { //for each container
				if (state->getContainer() != bot) {
					current = bot->getStateAt(state->getStartTime()); //start with first state at current time
					if (current == nullptr) continue; //stop if nothing matches
					std::list<robotState*> statesToAdd;
					while ((current->isInitialized()) && (current->getStartTime() < state->getEndTime())) { //if exists and in range
						double collidesAt = current->testCollision(state, &statesToAdd,-1.0);
						double collidesAt2 = state->testCollision(current, &statesToAdd, collidesAt);
						for (robotState * toAdd : statesToAdd) {
							toAdd->getContainer()->pushState(toAdd);
						}
						current = current->getChild();
					}
				}
			}
		}

	protected:
		double lastObservationTime; //how many observations
		double latestEvaluation; //how far is world correctly simulated until
		std::vector<roomba *> roombas;
		std::vector<robot *> robots;
		quad * quadRotor;
		queueType queue;

	}; //end world (kek)


	//---------------------------------------------------------------------ROOMBA
	class roomba : public robot { 

	friend class roombaState;

	public:
		roomba(roombaPose startPose_, bool isObstacle_, unsigned char ID_,world * currentWorld_) :
			robot(currentWorld_) {

			//just assign a bunch of things
			startPose = startPose_;
			diameter = roombaDiameter;
			ID = ID_;
			height = roombaHeight;
			if (isObstacle_) {
				type = robotType::Obstacle;
			}
			else {
				type = robotType::Roomba;
			}

			//create initial roomba state
			//startState = new roombaState(nullptr, nullptr, 0.0);
			startState = new runState(nullptr, nullptr, 0.0);
			//thing
			startState->initialize(); //let start state be queue'd
			recentState, endState, currentState = startState;

			/*
			//collision properties
			unsigned char ID;
			*/
		}

	protected:
		roombaPose startPose;
	}; //end roomba

	//----------------------------------------------------------------------QUAD
	class quad : public robot {
	public:
		quad(quadPose startPose, world * currentWorld_) :
			robot(currentWorld_) {
			type = robotType::Quad;

		}
	protected:
	}; //end quad


	//------------------------------------------------------------------ROOMBA STATE
	class roombaState : public robotState {

	friend class roomba;

	public:
		roombaState(robotState * parent_, robotState * dependency_, double startTime_) :
			robotState(parent_,dependency_,startTime_) {}
		struct roombaStateVals : public robotState::robotStateVals {
			double rightWheel;
			double leftWheel;
			double nextRotation;
			double nextRandomRotation;
			roombaPose startPose;
			void get2DPositionAtTime(double atTime, vector2 & out) {
				//math be heer
			}
			void get3DPositionAtTime(double atTime, vector3 & out) {
				get2DPositionAtTime(atTime, out);
				out.z = roombaHeight / 2.0; //just take roomba center of mass as position
			}
			void getOrientationAtTime(double atTime, double & out) {}
			void getPoseAtTime(double atTime, roombaPose & out) {
				get2DPositionAtTime(atTime, out.position);
				getOrientationAtTime(atTime, out.orientation);
			}
			
		}; //end roombaState::roombaStateVals


		void initialize() { //lots of casts because setting up type-specific information

			//---set up the value container---
			//generate a val container if not already
			if (vals == nullptr) {
				vals = new roombaStateVals();
			}
			else { //don't need to (or want to) regen all those values
				//calculate relevant values
				if (parent == nullptr) {
					//set to system start pose
					getVals()->startPose = static_cast<roomba*>(container)->startPose; //set to default pose
					getVals()->nextRotation = getStartTime()+reverseInterval;
					getVals()->nextRandomRotation = getStartTime()+noiseInterval;
				}
				else {
					//set to parent end pose
					static_cast<roombaState*>(parent)->getVals()->getPoseAtTime(getStartTime(), getVals()->startPose);
					getVals()->nextRotation = static_cast<roombaState*>(parent)->getVals()->nextRotation;
					getVals()->nextRandomRotation = static_cast<roombaState*>(parent)->getVals()->nextRandomRotation;
				}
			}


			//---call the common reference handling initializer---
			//all values in val container will be manually set
			robotState::initialize();

			//---put proto states in the queue---
			//container


			//TODO: PUT PRIMARY END STATE IN QUEUE AND CHILD POINTER HERE

			setupState(); //virtual, calls subclass initialization stuff

			//test generic collisions
			container->checkCollisions(this);

		}

		virtual void setupState() = 0;
		//this is both convienent for extracting code and also the most general form of the collision engine anyway
		inline virtual double testCollision(robotState * other, std::list<robotState*> * collisionStates, double collidesAt = -1.0) {
			if (collidesAt == -1.0) { //if unknown collision time
				if (other->getContainer()->isQuadrotor()) { //let's pretend we don't break the abstraction :)
															//too hard to do this just based off general position calculations
					if (other->getType() == stateType::Landed) {
						//do collision checks here
					}
					else {
						return NANd;
					}
				}
				else {

				}
			}
			else {
				return collidesAt; //evaluated or NAN
			}
		}

		//redefining these here to make sure compiler can do optimizations and non-virtual access
		void get2DPositionAtTime(double atTime, vector2 & out) { //TODO: when debug messages are out, make these inline?
			if (!isInitialized()) {
				std::cerr << "Attempted to access position in uninitialized state" << std::endl;
			}
			else {
				getVals()->get2DPositionAtTime(atTime, out);
			}
		}

		void get3DPositionAtTime(double atTime, vector3 & out) { //TODO: when debug messages are out, make these inline?
			if (!isInitialized()) {
				std::cerr << "Attempted to access position in uninitialized state" << std::endl;
			}
			else {
				getVals()->get3DPositionAtTime(atTime, out);
			}
		}

		void getOrientationAtTime(double atTime, double & out) { //TODO: when debug messages are out, make these inline?
			if (!isInitialized()) {
				std::cerr << "Attempted to access orientation in uninitialized state" << std::endl;
			}
			else {
				getVals()->getOrientationAtTime(atTime, out);
			}
		}

	protected:
		inline roombaStateVals * getVals() {  //to avoid excessive casting
			return static_cast<roombaStateVals*>(vals);
		}
	}; //end roombaState

	class quadState : public robotState {
		quadState(robotState * parent_, robotState * dependency_, double atTime) :
			robotState(parent_,dependency_,atTime) {
			//initialize quad state stuff here
		}

		double testCollision(robotState * other, std::list<robotState*> * collisionStates, double collidesAt) {
			if (getType() == stateType::Landed) {
				return -1.0; //collision not known if landed and a valid collision target
			}
			return NANd; //cannot run into a quad still in the air
		}
	}; //end quadState


	//put this in its own file


	//if collision detection fails return NAN

	//states have no unique internal values, instead just used to specify the virtual functions cleanly

	class runState : public roombaState{
	public:
		runState(robotState * parent_, robotState * dependency_, double atTime)
			: roombaState(parent_, dependency_, atTime) {}

		void setupState() { //was going to overload initialize() but trying to keep these as minimal as possible to allow compiler optimization
			//specify the end state
		}
		//the idea behind passing the collision time is that I was really annoyed that
		//these loops would be running collision detection code twice with this formulation.
		//This is a pretty straightforward way to keep this version while also keeping the optimization.
		double testCollision(robotState * other, double collidesAt, std::list<robotState*> * collisionStates) {
			return -1.0;
		}
	};

	class rotateState : public roombaState {
	public:
		rotateState(robotState * parent_, robotState * dependency_, double atTime)
			: roombaState(parent_, dependency_, atTime) {}
		void setupState() {

		}
		double testCollision(robotState * other, std::list<robotState*> * collisionStates, double collidesAt = -1.0) {
			return -1.0;
		}
	};

	class quarterRotateState : public roombaState {
	public:
		quarterRotateState(robotState * parent_, robotState * dependency_, double atTime)
			: roombaState(parent_, dependency_, atTime) {}
		void setupState() { 

		}
		double testCollision(robotState * other, std::list<robotState*> * collisionStates, double collidesAt = -1.0) {
			collidesAt = roombaState::testCollision(other, collisionStates, collidesAt);

			return -1.0;
		}
	};

	class noiseRunState : public roombaState {
	public:
		noiseRunState(robotState * parent_, robotState * dependency_, double atTime)
			: roombaState(parent_, dependency_, atTime) {}
		void setupState() {
			
		}
		double testCollision(robotState * other, std::list<robotState*> * collisionStates, double collidesAt = -1.0) {
			collidesAt = roombaState::testCollision(other, collisionStates, collidesAt);
			if (std::isnormal(collidesAt)) {
				collisionStates->push_back(new rotateState(this,other,collidesAt));
			}
			return collidesAt;
		}
	};

	class stoppedState : public roombaState {
	public:
		stoppedState(robotState * parent_, robotState * dependency_, double atTime)
			: roombaState(parent_, dependency_, atTime) {}
		void setupState() { 
			getVals()->leftWheel = 0;
			getVals()->rightWheel = 0;
			getVals()->child = nullptr;
			//static_cast<roombaStateVals*>(vals)->
		}
		double testCollision(robotState * other, std::list<robotState*> * collisionStates, double collidesAt = -1.0) {
			return NANd; //can't collide with a stopped roomba, because they never stop?
		}
	};

	class outOfBoundsState : public roombaState {
	public:
		outOfBoundsState(robotState * parent_, robotState * dependency_, double atTime)
			: roombaState(parent_, dependency_, atTime) {}
		void setupState() { 
			getVals()->leftWheel = 0;
			getVals()->rightWheel = 0;
			getVals()->child = nullptr;
		}
		double testCollision(robotState * other, std::list<robotState*> * collisionStates, double collidesAt = -1.0) {
			return NANd; //cannot collide
		}
	};

	class obstacleStoppedState : public roombaState {
	public:
		obstacleStoppedState(robotState * parent_, robotState * dependency_, double atTime)
			: roombaState(parent_, dependency_, atTime) {}
		void setupState() {
			getVals()->leftWheel = 0;
			getVals()->rightWheel = 0;
			getVals()->child = nullptr;
		}
		double testCollision(robotState * other, std::list<robotState*> * collisionStates, double collidesAt = -1.0) {
			//this is the weird one
		}
	};

	class obstacleRunState : public roombaState {
	public:
		obstacleRunState(robotState * parent_, robotState * dependency_, double atTime)
			: roombaState(parent_, dependency_, atTime) {}
		void setupState() {
			getVals()->rightWheel = roombaSpeed - obstacleSpeedOffset;
			getVals()->leftWheel = roombaSpeed + obstacleSpeedOffset;
			getVals()->child = nullptr;
		}
		double testCollision(robotState * other, std::list<robotState*> * collisionStates, double collidesAt = -1.0) {
			if (other->getContainer()->getType() == robotType::Obstacle) { //this bit of optimization is necessary to make the insane state length of these manageable
				//TODO: check the following logic, this is a potential source of bugs if incorrect
				if (other->getType() == stateType::ObstacleStopped) {
					if ((getContainer()->getID() - 11) == ((other->getContainer()->getID() - 10) % 4)) { //if it is the *next* obstacle
						double distance, t;
						getOrientationAtTime(getStartTime(), t);
						static_cast<roombaState*>(other)->getOrientationAtTime(getStartTime(), t);
						distance -= t;  //here distance is an angle
						if (distance < 0) { //correct it
							distance += 2.0*M_PI;
						}
						distance *= 5.0; //angle times radius equals distance around arc
						distance -= arcDiameter; //account for them not being in the same spot
						t = (distance / roombaSpeed);
						//TODO: gen state here
					}
				}
			}
		}
	};

}

#endif

