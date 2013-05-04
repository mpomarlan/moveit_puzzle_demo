#ifndef __VRVEDGE_H__

#define __VRVEDGE_H__

#include <vector>

#include <moveit/ompl_interface/VRVTypes.h>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>


namespace ompl
{
namespace LTLVis
{    
    class VRVEdge
    {
      friend class boost::serialization::access;

      public:
        VRVEdge();
        VRVEdge(double weight, tIndex towards);
        VRVEdge(VRVEdge const &orig);
        VRVEdge(VRVEdge const *orig);

        virtual ~VRVEdge();

        void initialize(double weight, tIndex towards);
        void getEdgeData(double &weight, tIndex &towards) const;

        bool isInteresting(void) const;
        void setExactInvalid(bool isFlagged = true);
        void clearExactInvalid();
        void setExactValid(bool isValid = true);
        void clearExactValid();
        bool isExactValid() const;

        template<class Archive> void serialize(Archive & ar, const unsigned int version)
        {
          ar & this->weight_;
          ar & this->towards_;
          ar & this->exactInvalid_;
          ar & this->exactValid_;
        }

      private:
        tIndex towards_;
        double weight_;
        bool exactInvalid_;
        bool exactValid_;
    };
}
}
#endif //ndef __VRVEDGE_H__
