#include <moveit/ompl_interface/VRVPlanner.h>

#include <ompl/tools/debug/Profiler.h>

#include <cmath>
#include <string>

#include <ompl/base/PlannerData.h>

#include <ros/package.h>

namespace ompl
{
  namespace LTLVis
  {
    typedef ompl::base::StateStorageWithMetadata<ompl::LTLVis::VRVVertexMetaData> VertexQuickStorage;
  }
}

ompl::LTLVis::VRVPlanner::VRVPlanner(const ompl::base::SpaceInformationPtr &si):
  ompl::base::Planner(si, "VRV"),
  validSampler_(),
  CCs_(),
  nearFinder_(4, 2, 6, 50, 50, true),
  vertices_(),
  workingVertex_(si),
  crVisitation_(0)
{

std::cout << "Constructor" <<std::endl;
  nearFinder_.setDistanceFunction(boost::bind(VRVPlanner::distance, this, _1, _2));
std::cout << "Constructor" <<std::endl;
  workingVertex_.setStateData(si_->allocState());
std::cout << "Constructor" <<std::endl;

  validSampler_ = si_->allocValidStateSampler();
std::cout << "Constructor" <<std::endl;

  specs_.approximateSolutions = false;
  specs_.multithreaded = false;
  specs_.optimizingPaths = false;
  specs_.directed = false;
  specs_.provingSolutionNonExistence = false;
  specs_.recognizedGoal = (ompl::base::GOAL_ANY);
std::cout << "Constructor" <<std::endl;
}
ompl::LTLVis::VRVPlanner::~VRVPlanner()
{
}

void ompl::LTLVis::VRVPlanner::resetVisitation(void)
{
  crVisitation_ = 0;
  ompl::LTLVis::tIndex kMax = vertices_.size();
  for(ompl::LTLVis::tIndex k = 0; k < kMax; k++)
  {
    vertices_[k].clearVisitation();
    vertices_[k].clearInitAlg();
  }
}

double ompl::LTLVis::VRVPlanner::distance(VRVPlanner const* planner, ompl::LTLVis::tIndex a, ompl::LTLVis::tIndex b)
{
  return planner->si_->distance(planner->vertices_[a].getStateData(), planner->vertices_[b].getStateData());
}


ompl::LTLVis::tIndex ompl::LTLVis::VRVPlanner::getValidNeighborhood(ompl::LTLVis::tIndex index, std::vector<ompl::LTLVis::tIndex> &neighbors) const
{
  for(std::vector<ompl::LTLVis::tIndex>::iterator iter = neighbors.begin(); iter != neighbors.end(); )
  {
    if(si_->checkMotion(vertices_[index].getStateData(), vertices_[(*iter)].getStateData()))
    {
      iter++;
    }
    else
    {
      iter = neighbors.erase(iter);
    }
  }
  return neighbors.size();
}
ompl::LTLVis::tIndex ompl::LTLVis::VRVPlanner::getValidKNeighborhood(ompl::LTLVis::tIndex index, ompl::LTLVis::tIndex k, std::vector<ompl::LTLVis::tIndex> &neighbors) const
{
  neighbors.clear();
  nearFinder_.nearestK(index, k, neighbors);
  return getValidNeighborhood(index, neighbors);
}
ompl::LTLVis::tIndex ompl::LTLVis::VRVPlanner::getValidRNeighborhood(ompl::LTLVis::tIndex index, double r, std::vector<ompl::LTLVis::tIndex> &neighbors) const
{
  neighbors.clear();
  nearFinder_.nearestR(index, r, neighbors);
  return getValidNeighborhood(index, neighbors);
}

ompl::LTLVis::tIndex ompl::LTLVis::VRVPlanner::getNeighborCCs(std::vector<ompl::LTLVis::tIndex> const &neighbors, std::set<ompl::LTLVis::tIndex> &neighborCCs) const
{
  neighborCCs.clear();
  ompl::LTLVis::tIndex kMax = neighbors.size();
  for(ompl::LTLVis::tIndex k = 0; k < kMax; k++)
  {
    neighborCCs.insert(CCs_.find(vertices_[neighbors[k]].getCCIndex()));
  }
  return neighborCCs.size();
}
ompl::LTLVis::tIndex ompl::LTLVis::VRVPlanner::getCommonCCs(std::set<ompl::LTLVis::tIndex> const& neighborACCs, std::set<ompl::LTLVis::tIndex> const& neighborBCCs, std::set<ompl::LTLVis::tIndex> & commonCCs) const
{
  commonCCs.clear();
  for(std::set<ompl::LTLVis::tIndex>::const_iterator iter = neighborACCs.begin(); iter != neighborACCs.end(); iter++)
  {
    if(neighborBCCs.count((*iter)))
    {
      commonCCs.insert((*iter));
    }
  }
  return commonCCs.size();
}

void ompl::LTLVis::VRVPlanner::addEdges(ompl::LTLVis::tIndex index, std::vector<ompl::LTLVis::tIndex> const& neighbors, bool i2n, bool n2i)
{
  //Only merge CCs if adding bidirectional edges (index <-> neighbor)
  bool trackCCs = (i2n && n2i);
  ompl::LTLVis::tIndex maxK = neighbors.size();
  for(ompl::LTLVis::tIndex k = 0; k < maxK; k++)
  {
    double weight = si_->distance(vertices_[index].getStateData(), vertices_[neighbors[k]].getStateData());
    if(i2n)
    {
      vertices_[index].addEdge(weight, neighbors[k]);
    }
    if(n2i)
    {
      vertices_[neighbors[k]].addEdge(weight, index);
    }
    if(trackCCs)
    {
      ompl::LTLVis::tIndex CCA, CCB, CCR;
      CCA = CCs_.find(vertices_[index].getCCIndex());
      CCB = CCs_.find(vertices_[neighbors[k]].getCCIndex());
      //CC index 0 is reserved. May be used in the future as temp vertex marker, for example.
      if(CCA && CCB)
      {
        CCs_.setUnion(CCA, CCB);
        CCR = CCs_.find(CCA);
        vertices_[index].setCCIndex(CCR);
        vertices_[neighbors[k]].setCCIndex(CCR);
      }
    }
  }
}

bool ompl::LTLVis::VRVPlanner::areInSameCC(ompl::LTLVis::tIndex A, ompl::LTLVis::tIndex B) const
{
  ompl::LTLVis::tIndex CCA, CCB;
  CCA = CCs_.find(vertices_[A].getCCIndex());
  CCB = CCs_.find(vertices_[B].getCCIndex());
  return (CCA == CCB);
}

void ompl::LTLVis::VRVPlanner::addVertex(const ompl::base::State *newState)
{
  ompl::LTLVis::tIndex newIndex = vertices_.size();
  ompl::LTLVis::VRVVertex dummy(si_, newState);
  vertices_.push_back(dummy);
  vertices_[newIndex].setIndex(newIndex);
  vertices_[newIndex].setCCIndex(CCs_.makeNewComponent());
  std::vector<ompl::LTLVis::tIndex> neighbors;
  getValidRNeighborhood(newIndex, 1e9, neighbors);
  //std::cout << "!!! " << vertices_.size() << ": " << neighbors.size() << std::endl;
  nearFinder_.add(newIndex);
  addEdges(newIndex, neighbors, true, true);
}


void ompl::LTLVis::VRVPlanner::sampleAndConnect(const ompl::base::PlannerTerminationCondition &ptc)
{
  bool extended = false;
  while((!extended) && (false == ptc()))
  {
    ompl::base::State* newState = si_->allocState();
ompl::tools::Profiler::Begin("sampling");
    while((!validSampler_->sample(newState)) && (false == ptc()))
    {
    }
ompl::tools::Profiler::End("sampling");
    if(false == ptc())
    {
      //Valid state found. Create a temp vertex for it
      ompl::LTLVis::tIndex newIndex = vertices_.size();
      ompl::LTLVis::VRVVertex dummy(si_, newState);
      vertices_.push_back(dummy);
      //Find neighbors and their CCs
      //TODO: add some variable radius function;
      double r = si_->getMaximumExtent()/std::pow(1.0 + vertices_.size(), 1.0/(1.0*si_->getStateDimension()));
      std::vector<ompl::LTLVis::tIndex> neighbors;
      getValidRNeighborhood(newIndex, r, neighbors);
      std::set<ompl::LTLVis::tIndex> neighborCCs;
      getNeighborCCs(neighbors, neighborCCs);
      if(0 == neighbors.size())
      {
ompl::tools::Profiler::Event("new CC");
        extended = true;
        //No connection to vertices in the roadmap: node improves coverage. Add to roadmap.
        vertices_[newIndex].setIndex(newIndex);
        nearFinder_.add(newIndex);
        vertices_[newIndex].setCCIndex(CCs_.makeNewComponent());
      }
      else if((1 == neighbors.size()) || (1 < neighborCCs.size()))
      {
ompl::tools::Profiler::Event("new Connection");
        extended = true;
        //Either extends in a hard to reach area or improves connectivity. Add to roadmap.
        nearFinder_.add(newIndex);
        vertices_[newIndex].setCCIndex(CCs_.makeNewComponent());
        vertices_[newIndex].setIndex(newIndex);
        addEdges(newIndex, neighbors, true, true);
      }
      //TODO: add useful loop/path shortening heuristic
      else
      {
        //Remove vertex from the roadmap, it does not appear useful.
ompl::tools::Profiler::Event("rejection");
        vertices_.pop_back();
      }
    }
    si_->freeState(newState);
  }
}

bool ompl::LTLVis::VRVPlanner::DijkstraSearch(const ompl::base::PlannerTerminationCondition &ptc, ompl::geometric::PathGeometric &solution)
{
  //update the value we use to check that a vertex has been initialized, or visited, in the current search
  ++crVisitation_;
  WorkingVertexVector pqueue;
  pqueue.clear();
  bool reachedGoal = false;
  bool goalDisconnected = false;

  ompl::LTLVis::tIndex goalIndexReached = 0;

  WorkingVertexVector pathCandidate;

  ompl::LTLVis::tIndex maxK = vertices_.size();
  for(ompl::LTLVis::tIndex k = 0; k < maxK; k++)
  {
    if(vertices_[k].getIsStart())
    {
      vertices_[k].setNegativeDistance(0);
      vertices_[k].setLastInitAlg(crVisitation_);
      pqueue.push_back(&vertices_[k]);
    }
  }
  make_heap(pqueue.begin(), pqueue.end(), VRVVertex::VRVVertexCompare);

  while(pqueue.size() && (ptc == false))
  {
    VRVVertex *cVertex(pqueue.front());
    pop_heap(pqueue.begin(), pqueue.end(), VRVVertex::VRVVertexCompare); pqueue.pop_back();

    if(cVertex->getLastVisitation() < crVisitation_)
    {
      cVertex->setLastVisitation(crVisitation_);
      if(cVertex->getIsGoal())
      {
        reachedGoal = true;
        goalIndexReached = cVertex->getIndex();
        break;
      }
      ompl::LTLVis::tIndex kMax = cVertex->getEdgeCount();
      for(ompl::LTLVis::tIndex k = 0; k < kMax; k++)
      {
        double weight;
        ompl::LTLVis::tIndex towards;
        cVertex->getEdge(k, weight, towards);
        VRVVertex *cTarget(&vertices_[towards]);
        if(cTarget->isInteresting())
        {
          //STL heap is a max heap, so we use negative edge weights to 'convert' it to a min-heap
          if(cTarget->getLastInitAlg() < crVisitation_)
          {
            cTarget->setLastInitAlg(crVisitation_);
            cTarget->setNegativeDistance(cVertex->getNegativeDistance() - weight - cTarget->getNodeCost());
            cTarget->setPreceding(cVertex->getIndex());
            cTarget->setPrecedingEdge(k);
            pqueue.push_back(cTarget); push_heap(pqueue.begin(), pqueue.end(), VRVVertex::VRVVertexCompare);
          }
          else
          {
            double negativeDist = cVertex->getNegativeDistance() - weight - cTarget->getNodeCost();
            if(negativeDist > cTarget->getNegativeDistance())
            {
              cTarget->setNegativeDistance(negativeDist);
              cTarget->setPreceding(cVertex->getIndex());
              cTarget->setPrecedingEdge(k);
              pqueue.push_back(cTarget); push_heap(pqueue.begin(), pqueue.end(), VRVVertex::VRVVertexCompare);
            }
          }
        }
      }
    }
  }

  if(reachedGoal)
  {
    pathCandidate.clear();
    ompl::LTLVis::tIndex cIndex = goalIndexReached;
    while(!vertices_[cIndex].getIsStart())
    {
      pathCandidate.push_back(&vertices_[cIndex]);
      cIndex = vertices_[cIndex].getPreceding();
    }
    pathCandidate.push_back(&vertices_[cIndex]);
    ompl::LTLVis::tIndex kMax = pathCandidate.size();
    for(ompl::LTLVis::tIndex k = 0, kPrev = 0; (k < kMax); k++)
    {
      if(0 < k)
      {
        if(0.000001 < si_->distance(pathCandidate[kMax - 1 - k]->getStateData(), pathCandidate[kMax - 1 - kPrev]->getStateData()))
        {
          solution.append(pathCandidate[kMax - 1 - k]->getStateData());
          kPrev = k;
        }
      }
      else
      {
        solution.append(pathCandidate[kMax - 1 - k]->getStateData());
      }
    }
    return true;
  }
  return false;
}
bool ompl::LTLVis::VRVPlanner::AStarSearch(const ompl::base::PlannerTerminationCondition &ptc, ompl::geometric::PathGeometric &solution)
{
  //TODO: implement
  return false;
}

void ompl::LTLVis::VRVPlanner::loadVertices(void)
{
  ompl::base::StateStorage storage(si_->getStateSpace());
  std::string fileName;
  fileName = ros::package::getPath("PR2_Card_Config");
  fileName = fileName + "/config/cardPuzzle.blg";
  storage.load(fileName.c_str());
  ompl::LTLVis::tIndex maxK = storage.size();
  vertices_.clear();
  CCs_.reset();
  nearFinder_.clear();
  for(ompl::LTLVis::tIndex k = 0; k < maxK; k++)
  {
    if(si_->isValid(storage.getState(k)))
      addVertex(storage.getState(k));
  }
  for(ompl::LTLVis::tIndex k = 0; k < vertices_.size(); k++)
  {
    vertices_[k].setCCIndex(CCs_.find(vertices_[k].getCCIndex()));
  }
}

void ompl::LTLVis::VRVPlanner::storeVertices(void) const
{
  ompl::base::StateStorage storage(si_->getStateSpace());
  std::string fileName;
  fileName = ros::package::getPath("PR2_Card_Config");
  fileName = fileName + "/config/cardPuzzle.blg";
  ompl::LTLVis::tIndex maxK = vertices_.size();
  for(ompl::LTLVis::tIndex k = 0; k < maxK; k++)
  {
    storage.addState(vertices_[k].getStateData());
  }
  storage.store(fileName.c_str());
}

void ompl::LTLVis::VRVPlanner::storeVerticesWithMetaData(void) const
{
  ompl::LTLVis::VertexQuickStorage quickStorage(si_->getStateSpace());
  ompl::LTLVis::tIndex maxK = vertices_.size();
  std::string fileNameRoadmap, fileNameCCs;
  fileNameRoadmap = ros::package::getPath("PR2_Card_Config");
  fileNameRoadmap = fileNameRoadmap + "/config/cardPuzzleWithMetaData_Roadmap.blg";
  fileNameCCs = ros::package::getPath("PR2_Card_Config");
  fileNameCCs = fileNameCCs + "/config/cardPuzzleWithMetaData_CCs.blg";
  for(ompl::LTLVis::tIndex k = 0; k < maxK; k++)
  {
    quickStorage.addState(vertices_[k].getStateData(), ompl::LTLVis::VRVVertexMetaData(vertices_[k]));
  }
  quickStorage.store(fileNameRoadmap.c_str());
  std::ofstream logFile(fileNameCCs.c_str(), std::ios::binary);
  boost::archive::binary_oarchive oa(logFile);
  oa << CCs_;
}
void ompl::LTLVis::VRVPlanner::loadVerticesWithMetaData(void)
{
  ompl::LTLVis::VertexQuickStorage quickStorage(si_->getStateSpace());
  std::string fileNameRoadmap, fileNameCCs;
  fileNameRoadmap = ros::package::getPath("PR2_Card_Config");
  fileNameRoadmap = fileNameRoadmap + "/config/cardPuzzleWithMetaData_Roadmap.blg";
  fileNameCCs = ros::package::getPath("PR2_Card_Config");
  fileNameCCs = fileNameCCs + "/config/cardPuzzleWithMetaData_CCs.blg";

std::cout << "Attempting to load from files: " << fileNameRoadmap << " and " << fileNameCCs << std::endl;
  quickStorage.load(fileNameRoadmap.c_str());
std::cout << "Attempting to load from files: " << fileNameRoadmap << " and " << fileNameCCs << std::endl;
  ompl::LTLVis::tIndex maxK = quickStorage.size();
  vertices_.clear();
  nearFinder_.clear();
  for(ompl::LTLVis::tIndex k = 0; k < maxK; k++)
  {
    ompl::LTLVis::VRVVertex dummy(si_, quickStorage.getState(k));
    dummy.readNonFlagsFromMetaData(quickStorage.getMetadata(k));
    vertices_.push_back(dummy);
    nearFinder_.add(k);
  }
  std::ifstream logFile(fileNameCCs.c_str(), std::ios::binary);
  boost::archive::binary_iarchive oa(logFile);
  CCs_.reset();
  oa >> CCs_;
  for(ompl::LTLVis::tIndex k = 0; k < vertices_.size(); k++)
  {
    vertices_[k].setCCIndex(CCs_.find(vertices_[k].getCCIndex()));
  }
std::cout << "Loading done!" << std::endl;
}

bool ompl::LTLVis::VRVPlanner::GraphSearch(const ompl::base::PlannerTerminationCondition &ptc, ompl::geometric::PathGeometric &solution, bool useAStar)
{
  return (useAStar)?(AStarSearch(ptc, solution)):(DijkstraSearch(ptc, solution));
}

bool ompl::LTLVis::VRVPlanner::verifyCC(tIndex vIndex) const
{
  std::vector<tIndex> toVisit;
  std::set<tIndex> visited;
  toVisit.clear();
  visited.clear();
  toVisit.push_back(vIndex);
  tIndex expCC = vertices_[vIndex].getCCIndex();
  bool retq = true;
  while(toVisit.size() && retq)
  {
    tIndex newIndex = toVisit[toVisit.size() - 1];
    tIndex numEdge = vertices_[newIndex].getEdgeCount();
    if(expCC != vertices_[newIndex].getCCIndex())
    {
      std::cout << "  Inconsistent set: exp " << expCC << " reach " << vertices_[newIndex].getCCIndex() << std::endl;
      retq = false;
    }
    else
    {
      visited.insert(newIndex);
      toVisit.pop_back();
      for(tIndex j = 0; j < numEdge; j++)
      {
        tIndex towards;
        double weight;
        vertices_[newIndex].getEdge(j, weight, towards);
        if(0 == visited.count(towards))
        {
          toVisit.push_back(towards);
        }
      }
    }
  }
  return retq;
}

ompl::base::PlannerStatus ompl::LTLVis::VRVPlanner::solve(const ompl::base::PlannerTerminationCondition &ptc)
{

  loadVerticesWithMetaData();
  std::cout << "Vertices loaded. " << std::endl;

  //prevent nasty overflows of the crVisitation_ index (unlikely though they are)
  if(ULONG_MAX - vertices_.size() < crVisitation_)
  {
    resetVisitation();
  }

  // make sure the planner is configured correctly; ompl::base::Planner::checkValidity
  // ensures that there is at least one input state and a ompl::base::Goal object specified
  checkValidity();
  std::cout << "Planner validity checked. " << std::endl;
  // get a handle to the Goal from the ompl::base::ProblemDefinition member, pdef_
  ompl::base::Goal *goal = pdef_->getGoal().get();
  std::cout << "Obtained goal. " << std::endl;
  ompl::geometric::PathGeometric solution(si_);
  bool pathFound = false;

ompl::tools::Profiler::Clear();
ompl::tools::Profiler::Start();

  std::cout << "VRV planner starting ... " << std::endl;
  std::cout << "Roadmap currently has " << vertices_.size() << " vertices." << std::endl;

  //Obtain one start and one goal state.
  //TODO: possibly make this more flexible to allow several start/goal states
  const ompl::base::State *startState = pis_.nextStart();
  const ompl::base::State *goalState = pis_.nextGoal(ptc);

  if(!startState)
  {
    return ompl::base::PlannerStatus::INVALID_START;
  }
  if(!goalState)
  {
    return ompl::base::PlannerStatus::INVALID_GOAL;
  }

  //Create temp vertices for the start and goal states
  ompl::LTLVis::tIndex startIndex = vertices_.size();
  ompl::LTLVis::tIndex goalIndex = startIndex + 1;

  vertices_.push_back(VRVVertex(si_, startState));
  vertices_.push_back(VRVVertex(si_, goalState));
  vertices_[startIndex].setIsStart();
  vertices_[startIndex].setIndex(startIndex);
  vertices_[goalIndex].setIsGoal();
  vertices_[goalIndex].setIndex(goalIndex);

  //Look for neighbors of the start/goal states in the roadmap
  //  do motion validation here. Keep only reachable neighbors. Lazy check not implemented.
  std::vector<ompl::LTLVis::tIndex> startNeighbors;
  std::vector<ompl::LTLVis::tIndex> goalNeighbors;
  getValidKNeighborhood(startIndex, 40, startNeighbors);
  getValidKNeighborhood(goalIndex, 40, goalNeighbors);


  //Check if the there is some path between a start and goal state: one pair of start/goal states
  //connects to the same connected component
  std::set<ompl::LTLVis::tIndex> startNeighborCCs;
  std::set<ompl::LTLVis::tIndex> goalNeighborCCs;
  std::set<ompl::LTLVis::tIndex> commonCCs;
  getNeighborCCs(startNeighbors, startNeighborCCs);
  getNeighborCCs(goalNeighbors, goalNeighborCCs);
  getCommonCCs(startNeighborCCs, goalNeighborCCs, commonCCs);

  if(commonCCs.size() && (1 == goalNeighborCCs.size()) && (1 == startNeighborCCs.size()))
  {
  std::cout << "Start and goal connectable to " << *(commonCCs.begin()) << std::endl;
    //If path between start and goal exists through a component C:
    //  add connectable start/goal to roadmap as temporary vertices
    //    {current implementation uses only one start and one goal state, and they were already added above}
    //  add temporary edges start->{neighbor vertices in C}
    addEdges(startIndex, startNeighbors, true, false);
    //  add temporary edges {neighbor vertices in C}->goal
    addEdges(goalIndex, goalNeighbors, false, true);
    //  check whether the problem is trivially solvable
    if(si_->checkMotion(vertices_[startIndex].getStateData(), vertices_[goalIndex].getStateData()))
    {
      std::vector<ompl::LTLVis::tIndex> dummy;
      std::cout << "Start and goal trivially connectable." << std::endl;
      dummy.clear(); dummy.push_back(goalIndex);
      addEdges(startIndex, dummy, true, false);
    }
    //  graph search, extract path
    pathFound = GraphSearch(ptc, solution, false);
    //  remove temporary edges and vertices
//  unsigned int kMax = vertices_.size() - 2;
//  for(unsigned int k = 0; k < kMax; k++)
//  {
//    std::cout << "CC " << vertices_[k].getCCIndex() << ": " << (verifyCC(k)?"consistent":"SET FAIL") << std::endl;
//  }
    ompl::LTLVis::tIndex maxK = goalNeighbors.size();
    for(ompl::LTLVis::tIndex k = 0; k < maxK; k++)
    {
      vertices_[goalNeighbors[k]].popEdge();
    }
    //if(pathFound)
    //  storeVerticesWithMetaData();
    vertices_.pop_back();
    vertices_.pop_back();
    //  storeVerticesWithMetaData();
    ///vertices_[goalIndex].setIsGoal(false);
    //  flag clearing: nothing much to do in this branch for now.
  }
  else
  {
  std::cout << "Start and goal in disconnected components." << std::endl;
    //If no path exists between start and goal through current roadmap:
    //  add the start/goal vertices to the roadmap as permanent vertices
    //    {current implementation uses only one start and one goal state; to fully add them to the roadmap:
    //       - add them to the nearFinder_ too
    //       - provide a CCindex
    //    }
    nearFinder_.add(startIndex);
    nearFinder_.add(goalIndex);
    vertices_[startIndex].setCCIndex(CCs_.makeNewComponent());
    vertices_[goalIndex].setCCIndex(CCs_.makeNewComponent());
    //  add edges to neighbors, merge connected components as appropriate
    addEdges(startIndex, startNeighbors, true, true);
    addEdges(goalIndex, goalNeighbors, true, true);
    //  check whether the problem is trivially solvable
    if(si_->checkMotion(vertices_[startIndex].getStateData(), vertices_[goalIndex].getStateData()))
    {
      std::vector<ompl::LTLVis::tIndex> dummy;
      dummy.clear(); dummy.push_back(goalIndex);
      std::cout << "Start and goal trivially connectable." << std::endl;
      addEdges(startIndex, dummy, true, true);
    }
    //  test whether start/goal get in same CC
  unsigned int kMax = vertices_.size();
  for(unsigned int k = 0; k < kMax; k++)
  {
    std::cout << "CC " << vertices_[k].getCCIndex() << ": " << (verifyCC(k)?"consistent":"SET FAIL") << std::endl;
  }
  //  pathFound = false;

    while((!areInSameCC(startIndex, goalIndex)) && (false == ptc()))
    {
      //  grow roadmap if not
ompl::tools::Profiler::Begin("sampleAndConnect");
      sampleAndConnect(ptc);
ompl::tools::Profiler::End("sampleAndConnect");
    }
    //  if start/goal in same CC
    if(areInSameCC(startIndex, goalIndex))
    {
      //  search for and extract path
ompl::tools::Profiler::Begin("graphSearch");
      pathFound = GraphSearch(ptc, solution, false);
ompl::tools::Profiler::End("graphSearch");
    }

    //  flag clearing
    vertices_[startIndex].setIsStart(false);
    vertices_[goalIndex].setIsGoal(false);
    //if(pathFound)
      storeVerticesWithMetaData();
  }
  // When a solution path is computed, save it here
  if(pathFound)
  {
    solution.interpolate();
    ompl::base::PathPtr solutionPtr(new ompl::geometric::PathGeometric(solution));
    pdef_->addSolutionPath(solutionPtr);
  }
  std::cout << "Roadmap currently has " << vertices_.size() << " vertices." << std::endl;
ompl::tools::Profiler::Stop();
ompl::tools::Profiler::Console();
ompl::tools::Profiler::Clear();
  // Return a value from the PlannerStatus enumeration.
  if(pathFound)
  {
    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  }
  else
  {
    return ompl::base::PlannerStatus::TIMEOUT;
  }
}  
void ompl::LTLVis::VRVPlanner::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
{
  ompl::base::Planner::setProblemDefinition(pdef);
}
void ompl::LTLVis::VRVPlanner::checkValidity(void)
{
  ompl::base::Planner::checkValidity();
}
void ompl::LTLVis::VRVPlanner::clear(void)
{
  ompl::base::Planner::clear();
  // clear the data structures here
  //CCs_.reset();
  //nearFinder_.clear();
  //vertices_.clear();
  //crVisitation_ = 0;
}
// optional, if additional setup/configuration is needed, the setup() method can be implemented
void ompl::LTLVis::VRVPlanner::setup(void)
{
  ompl::base::Planner::setup();
  // TODO: perhaps attempt some auto-configuration
}
void ompl::LTLVis::VRVPlanner::getPlannerData(ompl::base::PlannerData &data) const
{
  // fill data with the states and edges that were created
  // in the exploration data structure
  // perhaps also fill control::PlannerData
  // TODO: currently, only vertices are added
  data.clear();
std::cout << "VRVPlanner::getPlannerData : " << vertices_.size() << std::endl;
  std::vector<ompl::base::PlannerDataVertex> pdVertices;
  ompl::LTLVis::tIndex kMax = vertices_.size();
  for(ompl::LTLVis::tIndex k = 0; k < kMax; k++)
  {
    ompl::base::PlannerDataVertex dummy(vertices_[k].getStateData());
    pdVertices.push_back(dummy);
  }
  for(ompl::LTLVis::tIndex k = 0; k < kMax; k++)
  {
    data.addVertex(pdVertices[k]);
  }
  data.decoupleFromPlanner();
}

void ompl::LTLVis::VRVPlanner::printProperties(std::ostream &out) const
{
  //TODO
}
void ompl::LTLVis::VRVPlanner::printSettings(std::ostream &out) const
{
  //TODO
}

