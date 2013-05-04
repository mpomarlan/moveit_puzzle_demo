#include <moveit/ompl_interface/VRVVertex.h>
#include <moveit/ompl_interface/VRVEdge.h>

ompl::LTLVis::VRVVertexMetaData::VRVVertexMetaData():
  edges_(),
  nodeCost_(0.0),
  negativeDist_(0.0),
  CCindex_(0),
  preceding_(0),
  precedingEdge_(0),
  index_(0),
  lastInitAlg_(0),
  lastVisitation_(0),
  exactInvalid_(false),
  exactValid_(false),
  isStart_(false),
  isGoal_(false)
{
}

ompl::LTLVis::VRVVertexMetaData::VRVVertexMetaData(ompl::LTLVis::VRVVertex const& vertex):
  edges_(vertex.edges_),
  nodeCost_(vertex.nodeCost_),
  negativeDist_(vertex.negativeDist_),
  CCindex_(vertex.CCindex_),
  preceding_(vertex.preceding_),
  precedingEdge_(vertex.precedingEdge_),
  index_(vertex.index_),
  lastInitAlg_(vertex.lastInitAlg_),
  lastVisitation_(vertex.lastVisitation_),
  exactInvalid_(vertex.exactInvalid_),
  exactValid_(vertex.exactValid_),
  isStart_(vertex.isStart_),
  isGoal_(vertex.isGoal_)
{
}
ompl::LTLVis::VRVVertexMetaData::VRVVertexMetaData(ompl::LTLVis::VRVVertex const* vertex):
  edges_(vertex->edges_),
  nodeCost_(vertex->nodeCost_),
  negativeDist_(vertex->negativeDist_),
  CCindex_(vertex->CCindex_),
  preceding_(vertex->preceding_),
  precedingEdge_(vertex->precedingEdge_),
  index_(vertex->index_),
  lastInitAlg_(vertex->lastInitAlg_),
  lastVisitation_(vertex->lastVisitation_),
  exactInvalid_(vertex->exactInvalid_),
  exactValid_(vertex->exactValid_),
  isStart_(vertex->isStart_),
  isGoal_(vertex->isGoal_)
{
}

ompl::LTLVis::VRVVertexMetaData::VRVVertexMetaData(ompl::LTLVis::VRVVertexMetaData const& vertex):
  edges_(vertex.edges_),
  nodeCost_(vertex.nodeCost_),
  negativeDist_(vertex.negativeDist_),
  CCindex_(vertex.CCindex_),
  preceding_(vertex.preceding_),
  precedingEdge_(vertex.precedingEdge_),
  index_(vertex.index_),
  lastInitAlg_(vertex.lastInitAlg_),
  lastVisitation_(vertex.lastVisitation_),
  exactInvalid_(vertex.exactInvalid_),
  exactValid_(vertex.exactValid_),
  isStart_(vertex.isStart_),
  isGoal_(vertex.isGoal_)
{
}
ompl::LTLVis::VRVVertexMetaData::VRVVertexMetaData(ompl::LTLVis::VRVVertexMetaData const* vertex):
  edges_(vertex->edges_),
  nodeCost_(vertex->nodeCost_),
  negativeDist_(vertex->negativeDist_),
  CCindex_(vertex->CCindex_),
  preceding_(vertex->preceding_),
  precedingEdge_(vertex->precedingEdge_),
  index_(vertex->index_),
  lastInitAlg_(vertex->lastInitAlg_),
  lastVisitation_(vertex->lastVisitation_),
  exactInvalid_(vertex->exactInvalid_),
  exactValid_(vertex->exactValid_),
  isStart_(vertex->isStart_),
  isGoal_(vertex->isGoal_)
{
}

ompl::LTLVis::VRVVertexMetaData::~VRVVertexMetaData()
{
}

ompl::LTLVis::VRVVertex::VRVVertex():
  si_(),
  edges_(),
  stateData_(NULL),
  nodeCost_(0.0),
  negativeDist_(-1e9),
  CCindex_(0),
  preceding_(0),
  precedingEdge_(0),
  index_(0),
  lastInitAlg_(0),
  lastVisitation_(0),
  exactInvalid_(false),
  exactValid_(false),
  isStart_(false),
  isGoal_(false)
{
}
ompl::LTLVis::VRVVertex::VRVVertex(const ompl::base::SpaceInformationPtr &si):
  si_(new ompl::base::SpaceInformation(si->getStateSpace())),
  edges_(),
  stateData_(NULL),
  nodeCost_(0.0),
  negativeDist_(-1e9),
  CCindex_(0),
  preceding_(0),
  precedingEdge_(0),
  index_(0),
  lastInitAlg_(0),
  lastVisitation_(0),
  exactInvalid_(false),
  exactValid_(false),
  isStart_(false),
  isGoal_(false)
{
}
ompl::LTLVis::VRVVertex::VRVVertex(const ompl::base::SpaceInformationPtr &si, const ompl::base::State * state):
  si_(new ompl::base::SpaceInformation(si->getStateSpace())),
  edges_(),
  stateData_(NULL),
  nodeCost_(0.0),
  negativeDist_(-1e9),
  CCindex_(0),
  preceding_(0),
  precedingEdge_(0),
  index_(0),
  lastInitAlg_(0),
  lastVisitation_(0),
  exactInvalid_(false),
  exactValid_(false),
  isStart_(false),
  isGoal_(false)
{
  stateData_ = si_->cloneState(state);
}
ompl::LTLVis::VRVVertex::VRVVertex(ompl::LTLVis::VRVVertex const& orig):
  si_(new ompl::base::SpaceInformation(orig.si_->getStateSpace())),
  edges_(orig.edges_),
  stateData_(NULL),
  nodeCost_(orig.nodeCost_),
  negativeDist_(orig.negativeDist_),
  CCindex_(orig.CCindex_),
  preceding_(orig.preceding_),
  precedingEdge_(orig.precedingEdge_),
  index_(orig.index_),
  lastInitAlg_(orig.lastInitAlg_),
  lastVisitation_(orig.lastVisitation_),
  exactInvalid_(orig.exactInvalid_),
  exactValid_(orig.exactValid_),
  isStart_(orig.isStart_),
  isGoal_(orig.isGoal_)
{
  stateData_ = si_->cloneState(orig.stateData_);
}
ompl::LTLVis::VRVVertex::VRVVertex(ompl::LTLVis::VRVVertex const* orig):
  si_(new ompl::base::SpaceInformation(orig->si_->getStateSpace())),
  edges_(orig->edges_),
  stateData_(orig->stateData_),
  nodeCost_(orig->nodeCost_),
  negativeDist_(orig->negativeDist_),
  CCindex_(orig->CCindex_),
  preceding_(orig->preceding_),
  precedingEdge_(orig->precedingEdge_),
  index_(orig->index_),
  lastInitAlg_(orig->lastInitAlg_),
  lastVisitation_(orig->lastVisitation_),
  exactInvalid_(orig->exactInvalid_),
  exactValid_(orig->exactValid_),
  isStart_(orig->isStart_),
  isGoal_(orig->isGoal_)
{
  stateData_ = si_->cloneState(orig->stateData_);
}

ompl::LTLVis::VRVVertex::~VRVVertex()
{
  if(stateData_)
  {
    si_->freeState(stateData_);
    stateData_ = NULL;
  }
}

void ompl::LTLVis::VRVVertex::readNonFlagsFromMetaData(VRVVertexMetaData const& metadata)
{
  edges_.clear();
  edges_ = metadata.edges_;
  nodeCost_ = metadata.nodeCost_;
  CCindex_ = metadata.CCindex_;
  index_ = metadata.index_;
  negativeDist_ = -1e9;
  preceding_ = 0;
  precedingEdge_ = 0;
  lastInitAlg_ = 0;
  lastVisitation_ = 0;
  exactInvalid_ = false;
  exactValid_ = false;
  isStart_ = false;
  isGoal_ = false;
}
void ompl::LTLVis::VRVVertex::readAllFromMetaData(VRVVertexMetaData const& metadata)
{
  readNonFlagsFromMetaData(metadata);
  negativeDist_ = metadata.negativeDist_;
  preceding_ = metadata.preceding_;
  precedingEdge_ = metadata.precedingEdge_;
  lastInitAlg_ = metadata.lastInitAlg_;
  lastVisitation_ = metadata.lastVisitation_;
  exactInvalid_ = metadata.exactInvalid_;
  exactValid_ = metadata.exactValid_;
  isStart_ = metadata.isStart_;
  isGoal_ = metadata.isGoal_;
}
void ompl::LTLVis::VRVVertex::readNonFlagsFromMetaData(VRVVertexMetaData const* metadata)
{
  readNonFlagsFromMetaData(*metadata);
}
void ompl::LTLVis::VRVVertex::readAllFromMetaData(VRVVertexMetaData const* metadata)
{
  readAllFromMetaData(*metadata);
}

void ompl::LTLVis::VRVVertex::setIsStart(bool isStart)
{
  isStart_ = isStart;
}
void ompl::LTLVis::VRVVertex::setIsGoal(bool isGoal)
{
  isGoal_ = isGoal;
}
bool ompl::LTLVis::VRVVertex::getIsStart(void) const
{
  return isStart_;
}
bool ompl::LTLVis::VRVVertex::getIsGoal(void) const
{
  return isGoal_;
}

void ompl::LTLVis::VRVVertex::setIndex(ompl::LTLVis::tIndex index)
{
  index_ = index;
}
ompl::LTLVis::tIndex ompl::LTLVis::VRVVertex::getIndex(void) const
{
  return index_;
}

void ompl::LTLVis::VRVVertex::setCCIndex(ompl::LTLVis::tIndex index)
{
  CCindex_ = index;
}
ompl::LTLVis::tIndex ompl::LTLVis::VRVVertex::getCCIndex(void) const
{
  return CCindex_;
}

void ompl::LTLVis::VRVVertex::clearEdges(void)
{
  edges_.clear();
}
ompl::LTLVis::tIndex ompl::LTLVis::VRVVertex::getEdgeCount(void) const
{
  return edges_.size();
}
void ompl::LTLVis::VRVVertex::popEdge(void)
{
  edges_.pop_back();
}
void ompl::LTLVis::VRVVertex::getEdge(ompl::LTLVis::tIndex index, double &weight, ompl::LTLVis::tIndex &towards) const
{
  edges_[index].getEdgeData(weight, towards);
}
void ompl::LTLVis::VRVVertex::addEdge(double weight, ompl::LTLVis::tIndex towards)
{
  edges_.push_back(VRVEdge(weight, towards));
}

bool ompl::LTLVis::VRVVertex::isInteresting(void) const
{
  return (false == exactInvalid_);
}
void ompl::LTLVis::VRVVertex::setExactInvalid(bool isFlagged)
{
  exactInvalid_ = isFlagged;
  if(isFlagged)
  {
    setExactValid(false);
  }
}
void ompl::LTLVis::VRVVertex::clearExactInvalid()
{
  exactInvalid_ = false;
}
void ompl::LTLVis::VRVVertex::setExactValid(bool isValid)
{
  exactValid_ = isValid;
  if(isValid)
  {
    setExactInvalid(false);
  }
}
void ompl::LTLVis::VRVVertex::clearExactValid()
{
  exactValid_ = false;
}
bool ompl::LTLVis::VRVVertex::isExactValid() const
{
  return exactValid_;
}

void ompl::LTLVis::VRVVertex::incVisitation(void)
{
  ++lastVisitation_;
}
void ompl::LTLVis::VRVVertex::clearVisitation(void)
{
  lastVisitation_ = 0;
}
void ompl::LTLVis::VRVVertex::setVisitation(ompl::LTLVis::tIndex newVal)
{
  lastVisitation_ = newVal;
}
ompl::LTLVis::tIndex ompl::LTLVis::VRVVertex::getVisitation(void) const
{
  return lastVisitation_;
}
void ompl::LTLVis::VRVVertex::incInitAlg(void)
{
  ++lastInitAlg_;
}
void ompl::LTLVis::VRVVertex::clearInitAlg(void)
{
  lastInitAlg_ = 0;
}
void ompl::LTLVis::VRVVertex::setInitAlg(ompl::LTLVis::tIndex newVal)
{
  lastInitAlg_ = newVal;
}
ompl::LTLVis::tIndex ompl::LTLVis::VRVVertex::getInitAlg(void) const
{
  return lastInitAlg_;
}

ompl::base::State const* ompl::LTLVis::VRVVertex::getStateData(void) const
{
  return stateData_;
}
void ompl::LTLVis::VRVVertex::setStateData(ompl::base::State const* stateData)
{
  if(NULL == stateData_)
  {
    stateData_ = si_->cloneState(stateData);
  }
  else
  {
    si_->copyState(stateData_, stateData);
  }
}

bool ompl::LTLVis::VRVVertex::VRVVertexCompare(ompl::LTLVis::VRVVertex const* a, ompl::LTLVis::VRVVertex const* b)
{
  if((a->lastInitAlg_ < b->lastInitAlg_) || ((a->lastInitAlg_ == b->lastInitAlg_) && (a->negativeDist_ < b->negativeDist_)))
  {
    return true;
  }
  return false;
}

void ompl::LTLVis::VRVVertex::setNegativeDistance(double negativeDistance)
{
  negativeDist_ = negativeDistance;
}
double ompl::LTLVis::VRVVertex::getNegativeDistance(void) const
{
  return negativeDist_;
}
void ompl::LTLVis::VRVVertex::setLastInitAlg(ompl::LTLVis::tIndex lastInitAlg)
{
  lastInitAlg_ = lastInitAlg;
}
ompl::LTLVis::tIndex ompl::LTLVis::VRVVertex::getLastInitAlg(void) const
{
  return lastInitAlg_;
}
void ompl::LTLVis::VRVVertex::setLastVisitation(ompl::LTLVis::tIndex lastVisitation)
{
  lastVisitation_ = lastVisitation;
}
ompl::LTLVis::tIndex ompl::LTLVis::VRVVertex::getLastVisitation(void) const
{
  return lastVisitation_;
}
void ompl::LTLVis::VRVVertex::setNodeCost(double nodeCost)
{
  nodeCost_ = nodeCost;
}
double ompl::LTLVis::VRVVertex::getNodeCost(void) const
{
  return nodeCost_;
}
void ompl::LTLVis::VRVVertex::setPreceding(ompl::LTLVis::tIndex preceding)
{
  preceding_ = preceding;
}
ompl::LTLVis::tIndex ompl::LTLVis::VRVVertex::getPreceding(void) const
{
  return preceding_;
}
void ompl::LTLVis::VRVVertex::setPrecedingEdge(ompl::LTLVis::tIndex precedingEdge)
{
  precedingEdge_ = precedingEdge;
}
ompl::LTLVis::tIndex ompl::LTLVis::VRVVertex::getPrecedingEdge(void) const
{
  return precedingEdge_;
}


