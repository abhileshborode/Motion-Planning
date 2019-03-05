
//ROS
#include "ros/ros.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
// OMPL
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"

#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include "ompl/geometric/planners/cforest/CForest.h"
#include <ompl/geometric/planners/rrt/TRRT.h>
#include "ompl/geometric/planners/bitstar/BITstar.h"
#include <ompl/config.h>
#include <iostream>
//FCL
#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;


double penaliseZ(const ob::State* state);
ob::OptimizationObjectivePtr get2(const ob::SpaceInformationPtr& si);


//ROS publishers
ros::Publisher vis_pub;
ros::Publisher traj_pub;

std::shared_ptr<fcl::CollisionGeometry> Quadcopter(new fcl::Box(0.3, 0.3, 0.1));
fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.1)));
fcl::CollisionObject treeObj((std::shared_ptr<fcl::CollisionGeometry>(tree)));
fcl::CollisionObject aircraftObject(Quadcopter);


bool isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
	const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
	const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
	const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // check validity of state Fdefined by pos & rot
	fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
	fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
	aircraftObject.setTransform(rotation, translation);
	fcl::CollisionRequest requestType(1,false,1,false);
	fcl::CollisionResult collisionResult;
	fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

	return(!collisionResult.isCollision());
}


double penaliseZ(const ob::State* state) 
    {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ob::RealVectorStateSpace::StateType* state3D =
            state->as<ob::RealVectorStateSpace::StateType>();
        double x = state3D->values[0];
        double y = state3D->values[1];
        double z = state3D->values[2];
  
        return sqrt((x*x + y*y + 1000*z*z));
    }




class Penz :  public ob::StateCostIntegralObjective
{
public:
    Penz(const ob::SpaceInformationPtr& si) :
         ob::StateCostIntegralObjective(si )
    {
    }
    ob::Cost stateCost(const ob::State* state) const override
    {
      		const ob::SE3StateSpace::StateType *se3stat = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
	const ob::RealVectorStateSpace::StateType *state2D = se3stat->as<ob::RealVectorStateSpace::StateType>(0);
         //const auto* state2D =state->as<ob::RealVectorStateSpace::StateType>();
 
         // Extract the robot's (x,y) position from its state
         double x = state2D->values[0];
         double y = state2D->values[1];
         double z = state2D->values[2];
 			std::cout<< x << " " << y <<" "<< z<< std::endl;
 			std::cout<<" "<< ob::Cost(100-z) <<std::endl;
        return ob::Cost(sqrt(pow((z - 50), 2)));
    }


};


 


class Penz1 :  public ob::OptimizationObjective
{
public:
    Penz1(const ob::SpaceInformationPtr& si) :
          ob::OptimizationObjective(si )
    {
    }
    ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override
    {
    	ob::Cost C1(stateCost(s1));
    	ob::Cost C2(stateCost(s2));
    	return ob::Cost(C1.value() + C2.value());

    }


    	
    
    ob::Cost stateCost(const ob::State* s) const
    {
    	 const ob::SE3StateSpace::StateType* Cstate3D =
            s->as<ob::SE3StateSpace::StateType>();
            const ob::RealVectorStateSpace::StateType *state3D = Cstate3D->as<ob::RealVectorStateSpace::StateType>(0);
    	double z = state3D->values[2];
    	double y = state3D->values[1];
        return ob::Cost( 100*z);
        //return ob::Cost( 1);
    }
};


/*
ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	obj->setCostThreshold(ob::Cost(0));
	return obj;
}

ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
{
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
	return obj;
}*/

ob::OptimizationObjectivePtr get2(const ob::SpaceInformationPtr& si)
{
	
	//ob::OptimizationObjectivePtr obj(new Penz1(si));

	ob::OptimizationObjectivePtr obj = std::make_shared<Penz1>(si) ;
	//ob::OptimizationObjectivePtr obj (new Penz(si));
	//const ob::State* s;
		//obj->setCostToGoHeuristic(ob::Cost(0));
	obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);


	return obj;
	//return std::make_shared<Penz>(si);

}



void plan(void)
{
	// construct the state space we are planning in
	ob::StateSpacePtr space(new ob::SE3StateSpace());

    // set the bounds for the R^3 part of SE(3)
	ob::RealVectorBounds bounds(3);
    // bounds.setLow(-1);
    // bounds.setHigh(1);
    
	bounds.setLow(0,-10);
	bounds.setHigh(0,10);
	bounds.setLow(1,-10);
	bounds.setHigh(1,10);
	//bounds.setLow(2,0);
	//bounds.setHigh(2,1);
	bounds.setLow(2,0);
	bounds.setHigh(2,18);

	space->as<ob::SE3StateSpace>()->setBounds(bounds);

    // construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    // set state validity checking for this space
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));
	//si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));

	//si->setValidStateSamplerAllocator(allocOBValidStateSampler);
    // create a random start state
	ob::ScopedState<ob::SE3StateSpace> start(space);

	//start->setXYZ(-5,-2,1);
	//start->setXYZ(-5,-5,10); // mine like
    //start->setXYZ(3,-3,0);

    start->setXYZ(6.5,7.5,1); // reverse minelike

	start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	// start.random();

    // create a random goal state
	ob::ScopedState<ob::SE3StateSpace> goal(space);
	//goal->setXYZ(6.5,7.5,1); // minelike and closed
	//goal->setXYZ(7,3.7,1);
    //goal->setXYZ(1,3,1);
    goal->setXYZ(-5,-5,10); // reverse minelike
	 goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	//goal.random();

    // create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    // set the start and goal states
	pdef->setStartAndGoalStates(start, goal);

	//pdef->setOptimizationObjective(getClearanceObjective(si));
	pdef->setOptimizationObjective(get2(si));

    // create a planner for the defined space
	//ob::PlannerPtr planner(new og::RRTConnect(si));
    ob::PlannerPtr planner(new og::RRTstar(si));
	//ob::PlannerPtr planner(new og::PRMstar(si));
	//ob::PlannerPtr planner(new og::InformedRRTstar(si)); // best uptill now
	//ob::PlannerPtr planner(new og::LazyPRMstar(si));

	//ob::PlannerPtr planner(new og::TRRT(si));  

	//ob::PlannerPtr planner(new og::CForest(si)); 

	//ob::PlannerPtr planner(new og::BITstar(si));



    // set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);


    // perform setup steps for the planner
	planner->setup();


    // print the settings for this space
	si->printSettings(std::cout);

    // print the problem settings
	pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
	ob::PlannerStatus solved = planner->solve(20.0);


	std::cout << "Reached: " << std::endl;
	if (solved)
	{
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
		std::cout << "Found solution:" << std::endl;
		ob::PathPtr path = pdef->getSolutionPath();
		og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
		pth->printAsMatrix(std::cout);
        // print the path to screen
        // path->print(std::cout);
		trajectory_msgs::MultiDOFJointTrajectory msg;
		trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;

		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "world";
		msg.joint_names.clear();
		msg.points.clear();
		msg.joint_names.push_back("Quadcopter");
		
		for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++)
		{
			const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

			point_msg.time_from_start.fromSec(ros::Time::now().toSec());
			point_msg.transforms.resize(1);

			point_msg.transforms[0].translation.x= pos->values[0];
			point_msg.transforms[0].translation.y = pos->values[1];
			point_msg.transforms[0].translation.z = pos->values[2];

			point_msg.transforms[0].rotation.x = rot->x;
			point_msg.transforms[0].rotation.y = rot->y;
			point_msg.transforms[0].rotation.z = rot->z;
			point_msg.transforms[0].rotation.w = rot->w;

			msg.points.push_back(point_msg);

		}
		traj_pub.publish(msg);

		// Clear memory
		pdef->clearSolutionPaths();

		
        //Path smoothing using bspline

		og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
		og::PathGeometric path_smooth(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
		pathBSpline->smoothBSpline(path_smooth,3);
		std::cout << "Smoothed Path" << std::endl;
		path_smooth.print(std::cout);

		
		//Publish path as markers

		visualization_msgs::Marker marker;
		marker.action = visualization_msgs::Marker::DELETEALL;
		vis_pub.publish(marker);

		for (std::size_t idx = 0; idx < path_smooth.getStateCount (); idx++)
		{
                // cast the abstract state type to the type we expect
			const ob::SE3StateSpace::StateType *se3state = path_smooth.getState(idx)->as<ob::SE3StateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
			
			//marker.header.frame_id = "world";
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time();
			marker.ns = "path";
			marker.id = idx;
			marker.type = visualization_msgs::Marker::CUBE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = pos->values[0];
			marker.pose.position.y = pos->values[1];
			marker.pose.position.z = pos->values[2];
			marker.pose.orientation.x = rot->x;
			marker.pose.orientation.y = rot->y;
			marker.pose.orientation.z = rot->z;
			marker.pose.orientation.w = rot->w;
			marker.scale.x = 0.5;
			marker.scale.y = 0.5;
			marker.scale.z = 0.5;
			marker.color.a = 1.0;
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			vis_pub.publish(marker);
			// ros::Duration(0.1).sleep();
			std::cout << "Published marker: " << idx << std::endl;  
		}
		

	}
	else
		std::cout << "No solution found" << std::endl;
}

void octomapCallback(const octomap_msgs::Octomap &msg)
{


    //loading octree from binary
    std::cout<<"hi"<<std::endl;
	const std::string filename = "/home/abhilesh/minelike.bt";
    //const std::string filename = "/home/abhilesh/closed.bt";

	octomap::OcTree temp_tree(0.1);
	temp_tree.readBinary(filename);
	fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(&temp_tree));
	

	// convert octree to collision object
	// octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(msg));
	// fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));
	fcl::CollisionObject temp((std::shared_ptr<fcl::CollisionGeometry>(tree)));
	treeObj = temp;
	plan();

	// ros::Duration(10).sleep(); //Plan once every ten seconds
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "octomap_planner");
	ros::NodeHandle n;
	ros::Subscriber octree_sub = n.subscribe("/octomap_binary", 1, octomapCallback);
	vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",10);
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

	ros::spin();

	return 0;
}