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

// PCLTest Logic includes
#include <vtkSlicerPCLTestLogic.h>

// PCLTest includes
#include "qSlicerPCLTestModule.h"
#include "qSlicerPCLTestModuleWidget.h"

//-----------------------------------------------------------------------------
#if (QT_VERSION < QT_VERSION_CHECK(5, 0, 0))
#include <QtPlugin>
Q_EXPORT_PLUGIN2(qSlicerPCLTestModule, qSlicerPCLTestModule);
#endif

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ExtensionTemplate
class qSlicerPCLTestModulePrivate
{
public:
  qSlicerPCLTestModulePrivate();
};

//-----------------------------------------------------------------------------
// qSlicerPCLTestModulePrivate methods

//-----------------------------------------------------------------------------
qSlicerPCLTestModulePrivate::qSlicerPCLTestModulePrivate()
{
}

//-----------------------------------------------------------------------------
// qSlicerPCLTestModule methods

//-----------------------------------------------------------------------------
qSlicerPCLTestModule::qSlicerPCLTestModule(QObject* _parent)
  : Superclass(_parent)
  , d_ptr(new qSlicerPCLTestModulePrivate)
{
}

//-----------------------------------------------------------------------------
qSlicerPCLTestModule::~qSlicerPCLTestModule()
{
}

//-----------------------------------------------------------------------------
QString qSlicerPCLTestModule::helpText() const
{
  return "This is a loadable module that can be bundled in an extension";
}

//-----------------------------------------------------------------------------
QString qSlicerPCLTestModule::acknowledgementText() const
{
  return "This work was partially funded by NIH grant NXNNXXNNNNNN-NNXN";
}

//-----------------------------------------------------------------------------
QStringList qSlicerPCLTestModule::contributors() const
{
  QStringList moduleContributors;
  moduleContributors << QString("John Doe (AnyWare Corp.)");
  return moduleContributors;
}

//-----------------------------------------------------------------------------
QIcon qSlicerPCLTestModule::icon() const
{
  return QIcon(":/Icons/PCLTest.png");
}

//-----------------------------------------------------------------------------
QStringList qSlicerPCLTestModule::categories() const
{
  return QStringList() << "Examples";
}

//-----------------------------------------------------------------------------
QStringList qSlicerPCLTestModule::dependencies() const
{
  return QStringList();
}

//-----------------------------------------------------------------------------
void qSlicerPCLTestModule::setup()
{
  this->Superclass::setup();
}

//-----------------------------------------------------------------------------
qSlicerAbstractModuleRepresentation* qSlicerPCLTestModule
::createWidgetRepresentation()
{
  return new qSlicerPCLTestModuleWidget;
}

//-----------------------------------------------------------------------------
vtkMRMLAbstractLogic* qSlicerPCLTestModule::createLogic()
{
  return vtkSlicerPCLTestLogic::New();
}
