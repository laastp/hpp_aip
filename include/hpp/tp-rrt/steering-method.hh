//
// Copyright (c) 2015 CNRS
// Authors: Florent Lamiraux
//
// This file is part of tp_rrt
// tp_rrt is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// tp_rrt is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// tp_rrt  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_TP_HPP_STEERING_METHOD_STRAIGHT_HH
# define HPP_TP_HPP_STEERING_METHOD_STRAIGHT_HH

# include <hpp/model/joint.hh>
# include <hpp/core/steering-method.hh>
# include <hpp/tp-rrt/config.hh>
# include <hpp/tp-rrt/flat-path.hh>
# include <hpp/util/debug.hh>
# include <hpp/util/pointer.hh>

namespace hpp {
  namespace tp_rrt {
    /// \addtogroup steering_method
    /// \{

    /// Steering method that creates FlatPath instances
    ///
    class TP_RRT_DLLAPI SteeringMethod : public core::SteeringMethod
    {
    public:
      /// Create instance and return shared pointer
      static SteeringMethodPtr_t create (const DevicePtr_t& device)
      {
	SteeringMethod* ptr = new SteeringMethod (device);
	SteeringMethodPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Copy instance and return shared pointer
      static SteeringMethodPtr_t createCopy
	(const SteeringMethodPtr_t& other)
      {
	SteeringMethod* ptr = new SteeringMethod (*other);
	SteeringMethodPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Copy instance and return shared pointer
      virtual core::SteeringMethodPtr_t copy () const
      {
	return createCopy (weak_.lock ());
      }

      /// create a path between two configurations
      virtual PathPtr_t impl_compute (ConfigurationIn_t q1,
				      ConfigurationIn_t q2) const
      {
	PathPtr_t path = FlatPath::create (device_.lock (), q1, q2,
					   distanceBetweenAxes_,
					   constraints ());
	return path;
      }
    protected:
      /// Constructor with robot
      /// Weighed distance is created from robot
      SteeringMethod (const DevicePtr_t& device) :
	core::SteeringMethod (), device_ (device), weak_ ()
	{
	  computeDistanceBetweenAxes ();
	}
      /// Copy constructor
      SteeringMethod (const SteeringMethod& other) :
	core::SteeringMethod (other), device_ (other.device_),
	distanceBetweenAxes_ (other.distanceBetweenAxes_), weak_ ()
	{
	}

      /// Store weak pointer to itself
      void init (SteeringMethodWkPtr_t weak)
      {
        core::SteeringMethod::init (weak);
	weak_ = weak;
      }
    private:
      void computeDistanceBetweenAxes ()
      {        
      }

      DeviceWkPtr_t device_;
      // distance between front and rear wheel axes.
      value_type distanceBetweenAxes_;
      SteeringMethodWkPtr_t weak_;
    }; // SteeringMethod
    /// \}
  } // namespace tp_rrt
} // namespace hpp
#endif // HPP_TP_RRT_STEERING_METHOD_STRAIGHT_HH
