// Copyright 2023, Micro-G Dev Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "TargetSpawnerPlugin.hh"

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>

namespace micro_g {

class TargetSpawnerPluginPrivate {
public:
  // Plugin parameter: linear velocity (m/s)
  gz::math::Vector3d linearVelocity;

  // Plugin parameter: angular velocity (m/s)
  gz::math::Vector3d angularVelocity;

  gz::sim::Entity linkEntity;
  gz::transport::Node node;
  std::mutex mtx;
};

double sdfParamDouble(const std::shared_ptr<const sdf::Element> & _sdf,
                      const std::string & _field,
                      double _default) {
  if (!_sdf->HasElement(_field)) {
    return _default;
  }
  return _sdf->Get<double>(_field);
}

TargetSpawnerPlugin::TargetSpawnerPlugin()
: dataPtr(std::make_unique<TargetSpawnerPluginPrivate>()) {}

void TargetSpawnerPlugin::Configure(const gz::sim::Entity & _entity,
                                    const std::shared_ptr<const sdf::Element> & _sdf,
                                    gz::sim::EntityComponentManager & _ecm,
                                    gz::sim::EventManager & /*_eventMgr*/) {
  // Load the desired initial velocity
  this->dataPtr->linearVelocity = gz::math::Vector3d(sdfParamDouble(_sdf, "initVx", 0.0),
                                                     sdfParamDouble(_sdf, "initVy", 0.0),
                                                     sdfParamDouble(_sdf, "initVz", 0.0));

  this->dataPtr->angularVelocity = gz::math::Vector3d(sdfParamDouble(_sdf, "initRx", 0.0),
                                                      sdfParamDouble(_sdf, "initRy", 0.0),
                                                      sdfParamDouble(_sdf, "initRz", 0.0));

  // Create a model object for us to use for setting the target velocity
  auto model = gz::sim::Model(_entity);
  auto linkName = _sdf->Get<std::string>("link_name");
  this->dataPtr->linkEntity = model.LinkByName(_ecm, linkName);

  // if (gz::sim::kNullEntity == this->dataPtr->linkEntity) {
  //   gzerr << "Failed to find a link named [" << linkName << "] in model [" << model.Name(_ecm)
  //         << "]. Plugin failed to initialize." << std::endl;
  //   return;
  // }

  // Add velocity components
  if (!_ecm.Component<gz::sim::components::LinearVelocity>(_entity)) {
    _ecm.CreateComponent(_entity, gz::sim::components::LinearVelocity());
  }
  if (!_ecm.Component<gz::sim::components::AngularVelocity>(_entity)) {
    _ecm.CreateComponent(_entity, gz::sim::components::AngularVelocity());
  }
}

void TargetSpawnerPlugin::PreUpdate(const gz::sim::UpdateInfo & /*_info*/,
                                    gz::sim::EntityComponentManager & _ecm) {
  std::lock_guard<std::mutex> lock(this->dataPtr->mtx);

  // Set the desired velocities
  auto linearVelComp =
    _ecm.Component<gz::sim::components::LinearVelocity>(this->dataPtr->linkEntity);
  *linearVelComp = gz::sim::components::LinearVelocity(this->dataPtr->linearVelocity);

  auto angularVelComp =
    _ecm.Component<gz::sim::components::AngularVelocity>(this->dataPtr->linkEntity);
  *angularVelComp = gz::sim::components::AngularVelocity(this->dataPtr->angularVelocity);
}

}  // namespace micro_g

IGNITION_ADD_PLUGIN(micro_g::TargetSpawnerPlugin,
                    gz::sim::System,
                    micro_g::TargetSpawnerPlugin::ISystemConfigure,
                    micro_g::TargetSpawnerPlugin::ISystemPreUpdate)
