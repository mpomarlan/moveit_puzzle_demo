#ifndef __VRVTYPES_H__

#define __VRVTYPES_H__

#include <vector>

#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateStorage.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

namespace ompl
{
namespace LTLVis
{

  typedef unsigned long int tIndex;

  typedef std::vector<ompl::base::State *> StatePtrVector;
  typedef std::vector<ompl::base::State const*> StateConstPtrVector;

  typedef ompl::NearestNeighborsGNAT<tIndex> NearQueryStructure;
  typedef ompl::NearestNeighborsGNAT<tIndex>::DistanceFunction DistanceFunction;
}
}

#endif //ndef __VRVVERTEX_H__
