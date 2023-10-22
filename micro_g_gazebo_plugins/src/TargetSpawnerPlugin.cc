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

#include <gz/msgs.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>

namespace micro_g
{

class TargetSpawnerPrivateData
{
public:
  double paramInitWrench;
  gz::sim::Entity linkEntity;
  gz::transport::Node node;
};

void AddWorldLinearVelocity(const gz::sim::Entity & _entity, gz::sim::EntityComponentManager & _ecm)
{
  if (!_ecm.Component<gz::sim::components::WorldLinearVelocity>(_entity)) {
    _ecm.CreateComponent(_entity, gz::sim::components::WorldLinearVelocity());
  }
}

void AddAngularVelocityComponent(
  const gz::sim::Entity & _entity, gz::sim::EntityComponentManager & _ecm)
{
  if (!_ecm.Component<gz::sim::components::AngularVelocity>(_entity)) {
    _ecm.CreateComponent(_entity, gz::sim::components::AngularVelocity());
  }
  if (!_ecm.Component<gz::sim::components::WorldAngularVelocity>(_entity)) {
    _ecm.CreateComponent(_entity, gz::sim::components::WorldAngularVelocity());
  }
}

void AddWorldPose(const gz::sim::Entity & _entity, gz::sim::EntityComponentManager & _ecm)
{
  if (!_ecm.Component<gz::sim::components::WorldPose>(_entity)) {
    _ecm.CreateComponent(_entity, gz::sim::components::WorldPose());
  }
}

double SdfParamDouble(
  const std::shared_ptr<const sdf::Element> & _sdf, const std::string & _field, double _default)
{
  if (!_sdf->HasElement(_field)) {
    return _default;
  }
  return _sdf->Get<double>(_field);
};

TargetSpawnerPlugin::TargetSpawnerPlugin()
: dataPtr(std::make_unique<TargetSpawnerPrivateData>()){};

void TargetSpawnerPlugin::Configure(
  const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & /*_eventMgr*/)
{
  this->dataPtr->paramInitWrench = SdfParamDouble(_sdf, "initWrench", 0.0);

  AddWorldPose(this->dataPtr->linkEntity, _ecm);
  AddWorldLinearVelocity(this->dataPtr->linkEntity, _ecm);
  AddAngularVelocityComponent(this->dataPtr->linkEntity, _ecm);
};

}  // namespace micro_g
