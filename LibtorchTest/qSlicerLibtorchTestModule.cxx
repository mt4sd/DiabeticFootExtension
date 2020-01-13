/*==============================================================================

  Program: 3D Slicer

  Portions (c) Copyright Brigham and Women's Hospital (BWH) All Rights Reserved.

  See COPYRIGHT.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

==============================================================================*/

// LibtorchTest Logic includes
#include <vtkSlicerLibtorchTestLogic.h>

// LibtorchTest includes
#include "qSlicerLibtorchTestModule.h"
#include "qSlicerLibtorchTestModuleWidget.h"

//-----------------------------------------------------------------------------
#if (QT_VERSION < QT_VERSION_CHECK(5, 0, 0))
#include <QtPlugin>
Q_EXPORT_PLUGIN2(qSlicerLibtorchTestModule, qSlicerLibtorchTestModule);
#endif

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ExtensionTemplate
class qSlicerLibtorchTestModulePrivate
{
public:
  qSlicerLibtorchTestModulePrivate();
};

//-----------------------------------------------------------------------------
// qSlicerLibtorchTestModulePrivate methods

//-----------------------------------------------------------------------------
qSlicerLibtorchTestModulePrivate::qSlicerLibtorchTestModulePrivate()
{
}

//-----------------------------------------------------------------------------
// qSlicerLibtorchTestModule methods

//-----------------------------------------------------------------------------
qSlicerLibtorchTestModule::qSlicerLibtorchTestModule(QObject* _parent)
  : Superclass(_parent)
  , d_ptr(new qSlicerLibtorchTestModulePrivate)
{
}

//-----------------------------------------------------------------------------
qSlicerLibtorchTestModule::~qSlicerLibtorchTestModule()
{
}

//-----------------------------------------------------------------------------
QString qSlicerLibtorchTestModule::helpText() const
{
  return "This is a loadable module that can be bundled in an extension";
}

//-----------------------------------------------------------------------------
QString qSlicerLibtorchTestModule::acknowledgementText() const
{
  return "This work was partially funded by NIH grant NXNNXXNNNNNN-NNXN";
}

//-----------------------------------------------------------------------------
QStringList qSlicerLibtorchTestModule::contributors() const
{
  QStringList moduleContributors;
  moduleContributors << QString("John Doe (AnyWare Corp.)");
  return moduleContributors;
}

//-----------------------------------------------------------------------------
QIcon qSlicerLibtorchTestModule::icon() const
{
  return QIcon(":/Icons/LibtorchTest.png");
}

//-----------------------------------------------------------------------------
QStringList qSlicerLibtorchTestModule::categories() const
{
  return QStringList() << "Examples";
}

//-----------------------------------------------------------------------------
QStringList qSlicerLibtorchTestModule::dependencies() const
{
  return QStringList();
}

//-----------------------------------------------------------------------------
void qSlicerLibtorchTestModule::setup()
{
  this->Superclass::setup();
}

//-----------------------------------------------------------------------------
qSlicerAbstractModuleRepresentation* qSlicerLibtorchTestModule
::createWidgetRepresentation()
{
  return new qSlicerLibtorchTestModuleWidget;
}

//-----------------------------------------------------------------------------
vtkMRMLAbstractLogic* qSlicerLibtorchTestModule::createLogic()
{
  return vtkSlicerLibtorchTestLogic::New();
}
