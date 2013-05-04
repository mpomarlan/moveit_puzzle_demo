#include <moveit/ompl_interface/VRVEdge.h>

ompl::LTLVis::VRVEdge::VRVEdge():
  towards_(0),
  weight_(0.0),
  exactInvalid_(false),
  exactValid_(false)
{
}
ompl::LTLVis::VRVEdge::VRVEdge(double weight, ompl::LTLVis::tIndex towards):
  towards_(towards),
  weight_(weight),
  exactInvalid_(false),
  exactValid_(false)
{
}
ompl::LTLVis::VRVEdge::VRVEdge(ompl::LTLVis::VRVEdge const &orig):
  towards_(orig.towards_),
  weight_(orig.weight_),
  exactInvalid_(orig.exactInvalid_),
  exactValid_(orig.exactValid_)
{
}
ompl::LTLVis::VRVEdge::VRVEdge(ompl::LTLVis::VRVEdge const *orig):
  towards_(orig->towards_),
  weight_(orig->weight_),
  exactInvalid_(orig->exactInvalid_),
  exactValid_(orig->exactValid_)
{
}
ompl::LTLVis::VRVEdge::~VRVEdge()
{
}

void ompl::LTLVis::VRVEdge::initialize(double weight, ompl::LTLVis::tIndex towards)
{
  weight_ = weight;
  towards_ = towards;
}
void ompl::LTLVis::VRVEdge::getEdgeData(double &weight, ompl::LTLVis::tIndex &towards) const
{
  weight = weight_;
  towards = towards_;
}

bool ompl::LTLVis::VRVEdge::isInteresting(void) const
{
  return (false == exactInvalid_);
}
void ompl::LTLVis::VRVEdge::setExactInvalid(bool isFlagged)
{
  exactInvalid_ = isFlagged;
  if(exactInvalid_)
  {
    exactValid_ = false;
  }
}
void ompl::LTLVis::VRVEdge::clearExactInvalid()
{
  exactInvalid_ = false;
}
void ompl::LTLVis::VRVEdge::setExactValid(bool isValid)
{
  exactValid_ = isValid;
  if(exactValid_)
  {
    exactInvalid_ = false;
  }
}
void ompl::LTLVis::VRVEdge::clearExactValid()
{
  exactValid_ = false;
}
bool ompl::LTLVis::VRVEdge::isExactValid() const
{
  return exactValid_;
}

