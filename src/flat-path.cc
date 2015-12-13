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

#include <hpp/util/debug.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/projection-error.hh>
#include <hpp/tp-rrt/flat-path.hh>

namespace hpp {
  namespace tp_rrt {

    void FlatPath::computeCoefficients ()
    {

    }

    FlatPath::FlatPath (const DevicePtr_t& device,
			ConfigurationIn_t init,
			ConfigurationIn_t end,
			value_type distanceBetweenAxes) :
      parent_t (interval_t (0, 1.), device->configSize (),
		device->numberDof ()),
      device_ (device), initial_ (init), end_ (end),
      distanceBetweenAxes_ (distanceBetweenAxes), P_ (6)
    {

    }

    FlatPath::FlatPath (const DevicePtr_t& device,
			ConfigurationIn_t init,
			ConfigurationIn_t end,
			value_type distanceBetweenAxes,
			ConstraintSetPtr_t constraints) :
      parent_t (interval_t (0, 1.), device->configSize (),
		device->numberDof (), constraints),
      device_ (device), initial_ (init), end_ (end),
      distanceBetweenAxes_ (distanceBetweenAxes), P_ (6)
    {

    }

    FlatPath::FlatPath (const FlatPath& path) :
      parent_t (path), device_ (path.device_), initial_ (path.initial_),
      end_ (path.end_), distanceBetweenAxes_ (path.distanceBetweenAxes_),
      P_ (path.P_)
    {
    }

    FlatPath::FlatPath (const FlatPath& path,
			const ConstraintSetPtr_t& constraints) :
      parent_t (path, constraints), device_ (path.device_),
      initial_ (path.initial_), end_ (path.end_),
      distanceBetweenAxes_ (path.distanceBetweenAxes_), P_ (path.P_)
    {

    }

    bool FlatPath::impl_compute (ConfigurationOut_t /*result*/,
                 value_type /*param*/) const
    {
      return true;
    }

    DevicePtr_t FlatPath::device () const
    {
      return device_;
    }
  } //   namespace tp_rrt
} // namespace hpp
