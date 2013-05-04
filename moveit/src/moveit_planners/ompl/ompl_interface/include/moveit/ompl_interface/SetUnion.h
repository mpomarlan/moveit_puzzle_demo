#ifndef __SETUNION_H__

#define __SETUNION_H__

#include <vector>

#include <moveit/ompl_interface/VRVTypes.h>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

namespace ompl
{
namespace LTLVis
{

//Two classes used to implement Tarjan's disjoint set data structure. 
//Amortized queries (find set, merge sets) are inverse Ackermann complexity, so constant for practical purposes.

//Index 0 is reserved for an empty set.

    class ConnectedComponent
    {
      friend class boost::serialization::access;
      public:
        ConnectedComponent();
        ConnectedComponent(tIndex indexP);
        template<class Archive> void serialize(Archive & ar, const unsigned int version)
        {
          ar & this->index_;
          ar & this->parent_;
          ar & this->rank_;
        }
        tIndex index_;
        tIndex parent_;
        tIndex rank_;
    };

    class ComponentMap
    {
      friend class boost::serialization::access;
      public:
        ComponentMap();

        tIndex makeNewComponent(void);
        tIndex find(tIndex index);
        void setUnion(tIndex indexA, tIndex indexB);
        void reset(void);

        template<class Archive> void serialize(Archive & ar, const unsigned int version)
        {
          ar & this->components_;
        }

      private:
        std::vector<ConnectedComponent> components_;
};

}
}
#endif //ndef __SETUNION_H__
