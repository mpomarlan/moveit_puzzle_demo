#include <stdio.h>
#include <vector>

#include <moveit/ompl_interface/SetUnion.h>

ompl::LTLVis::ConnectedComponent::ConnectedComponent(): index_(0), parent_(0), rank_(0)
{
}
ompl::LTLVis::ConnectedComponent::ConnectedComponent(ompl::LTLVis::tIndex indexP): index_(indexP), parent_(indexP), rank_(0)
{
}


ompl::LTLVis::ComponentMap::ComponentMap(): components_(1)
{
}

ompl::LTLVis::tIndex ompl::LTLVis::ComponentMap::makeNewComponent(void)
{
  ompl::LTLVis::tIndex retq = components_.size();
  components_.push_back(ConnectedComponent(components_.size()));
  return(retq);
}
		
ompl::LTLVis::tIndex ompl::LTLVis::ComponentMap::find(ompl::LTLVis::tIndex index)
{
  if(components_[index].parent_ != index)
  {
    components_[index].parent_ = find(components_[index].parent_);
  }
  return components_[index].parent_;
}

void ompl::LTLVis::ComponentMap::setUnion(ompl::LTLVis::tIndex indexA, ompl::LTLVis::tIndex indexB)
{
  if((components_.size() <= indexA) || (components_.size() <= indexB))
  {
    return;
  }
  ompl::LTLVis::tIndex compA, compB;
  compA = find(indexA);
  compB = find(indexB);
  if(compA == compB)
  {
    return;
  }
  if(components_[compA].rank_ < components_[compB].rank_)
  {
    components_[compA].parent_ = compB;
    components_[indexA].parent_ = compB;
  }
  else if(components_[compA].rank_ > components_[compB].rank_)
  {
    components_[compB].parent_ = compA;
    components_[indexB].parent_ = compA;
  }
  else
  {
    components_[compB].parent_ = compA;
    components_[indexB].parent_ = compA;
    components_[compA].rank_ = components_[compA].rank_ + 1;
  }
}

void ompl::LTLVis::ComponentMap::reset(void)
{
  components_.clear();
  components_.push_back(ConnectedComponent(components_.size()));
}
