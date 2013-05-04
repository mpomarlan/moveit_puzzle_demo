#ifndef __VRVPLANNER_H__

#define __VRVPLANNER_H__

#include <vector>
#include <set>

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateStorage.h>
#include <ompl/base/ValidStateSampler.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

#include <moveit/ompl_interface/VRVTypes.h>
#include <moveit/ompl_interface/VRVEdge.h>
#include <moveit/ompl_interface/VRVVertex.h>
#include <moveit/ompl_interface/SetUnion.h>

/*
#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/ompl_interface/constraints_library.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/BenchmarkPluginRequest.h>
#include <moveit_msgs/BenchmarkPluginResponse.h>
#include <string>
#include <map>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/ompl_interface/detail/constrained_valid_state_sampler.h>
#include <ompl/tools/debug/Profiler.h>
#include <fstream>
*/

#include <ompl/base/State.h>
#include <ompl/base/StateStorage.h>

#include <fstream>


namespace ompl
{
namespace LTLVis
{    

    class VRVPlanner: public ompl::base::Planner
    {
      public:

  
        typedef ompl::LTLVis::VRVVertex VertexType;
        typedef ompl::LTLVis::VRVEdge EdgeType;
        typedef std::vector<VertexType*> WorkingVertexVector;
        typedef std::vector<EdgeType*> WorkingEdgeVector;
 
        VRVPlanner(const ompl::base::SpaceInformationPtr &si);
        virtual ~VRVPlanner();
        virtual void setProblemDefinition (const ompl::base::ProblemDefinitionPtr &pdef);
        virtual void checkValidity(void);
        virtual base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);
        virtual void clear(void);
        // optional, if additional setup/configuration is needed, the setup() method can be implemented
        virtual void setup(void);
        virtual void getPlannerData(ompl::base::PlannerData &data) const;
        virtual void printProperties(std::ostream &out) const;
        virtual void printSettings(std::ostream &out) const;

        void addVertex(const ompl::base::State *newState);

        //Loading from/storing to file: the slow versions, as edges are not loaded, rather retried
        void storeVertices(void) const;
        void loadVertices(void);
        bool verifyCC(tIndex vIndex) const;
        //Loading from/storing to file: full graph load/store, edges included. Faster load
        void storeVerticesWithMetaData(void) const;
        void loadVerticesWithMetaData(void);

//void getAllStateValues(ompl_interface::ModelBasedPlanningContextPtr &context, robot_state::RobotState &start_state) const;
//void loadAllStateValues(ompl_interface::ModelBasedPlanningContextPtr &context, robot_state::RobotState &start_state);

      protected:

      private:
        void resetVisitation(void);

        tIndex getValidKNeighborhood(tIndex index, tIndex k, std::vector<tIndex> &neighbors) const;
        tIndex getValidRNeighborhood(tIndex index, double r, std::vector<tIndex> &neighbors) const;
        tIndex getValidNeighborhood(tIndex index, std::vector<tIndex> &neighbors) const;

        tIndex getNeighborCCs(std::vector<tIndex> const &neighbors, std::set<tIndex> &neighborCCs) const;
        tIndex getCommonCCs(std::set<tIndex> const& neighborACCs, std::set<tIndex> const& neighborBCCs, std::set<tIndex> & commonCCs) const;
        bool areInSameCC(tIndex A, tIndex B) const;

        void addEdges(tIndex index, std::vector<tIndex> const& neighbors, bool i2n, bool n2i);

        void sampleAndConnect(const ompl::base::PlannerTerminationCondition &ptc);

        bool DijkstraSearch(const ompl::base::PlannerTerminationCondition &ptc, ompl::geometric::PathGeometric &solution);
        bool AStarSearch(const ompl::base::PlannerTerminationCondition &ptc, ompl::geometric::PathGeometric &solution);
        bool GraphSearch(const ompl::base::PlannerTerminationCondition &ptc, ompl::geometric::PathGeometric &solution, bool useAStar = true);

        void clearScratch(void);

        static double distance(VRVPlanner const* planner, tIndex a, tIndex b);

        ompl::base::ValidStateSamplerPtr validSampler_;
        mutable ComponentMap CCs_;
        NearQueryStructure nearFinder_;
        std::vector<VRVVertex> vertices_;
        VRVVertex workingVertex_;
        tIndex crVisitation_;
    };
}
}

#endif //ndef __VRVPLANNER_H__
