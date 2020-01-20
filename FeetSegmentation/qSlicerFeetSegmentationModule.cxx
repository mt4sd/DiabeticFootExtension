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

// FeetSegmentation Logic includes
#include <vtkSlicerFeetSegmentationLogic.h>

// FeetSegmentation includes
#include "qSlicerFeetSegmentationModule.h"
#include "qSlicerFeetSegmentationModuleWidget.h"

//To remove
#include <QDebug>

//-----------------------------------------------------------------------------
#if (QT_VERSION < QT_VERSION_CHECK(5, 0, 0))
#include <QtPlugin>
Q_EXPORT_PLUGIN2(qSlicerFeetSegmentationModule, qSlicerFeetSegmentationModule);
#endif

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ExtensionTemplate
class qSlicerFeetSegmentationModulePrivate
{
public:
  qSlicerFeetSegmentationModulePrivate();
};

//-----------------------------------------------------------------------------
// qSlicerFeetSegmentationModulePrivate methods

//-----------------------------------------------------------------------------
qSlicerFeetSegmentationModulePrivate::qSlicerFeetSegmentationModulePrivate()
{
}

//-----------------------------------------------------------------------------
// qSlicerFeetSegmentationModule methods

//-----------------------------------------------------------------------------
qSlicerFeetSegmentationModule::qSlicerFeetSegmentationModule(QObject* _parent)
  : Superclass(_parent)
  , d_ptr(new qSlicerFeetSegmentationModulePrivate)
  , rgbInputVolumeNode(nullptr), depthInputVolumeNode(nullptr)
{
}

//-----------------------------------------------------------------------------
qSlicerFeetSegmentationModule::~qSlicerFeetSegmentationModule()
{
}

//-----------------------------------------------------------------------------
QString qSlicerFeetSegmentationModule::helpText() const
{
  return "This is a loadable module that can be bundled in an extension";
}

//-----------------------------------------------------------------------------
QString qSlicerFeetSegmentationModule::acknowledgementText() const
{
  return "This work was partially funded by NIH grant NXNNXXNNNNNN-NNXN";
}

//-----------------------------------------------------------------------------
QStringList qSlicerFeetSegmentationModule::contributors() const
{
  QStringList moduleContributors;
  moduleContributors << QString("John Doe (AnyWare Corp.)");
  return moduleContributors;
}

//-----------------------------------------------------------------------------
QIcon qSlicerFeetSegmentationModule::icon() const
{
  return QIcon(":/Icons/FeetSegmentation.png");
}

//-----------------------------------------------------------------------------
QStringList qSlicerFeetSegmentationModule::categories() const
{
  return QStringList() << "Examples";
}

//-----------------------------------------------------------------------------
QStringList qSlicerFeetSegmentationModule::dependencies() const
{
  return QStringList();
}

//-----------------------------------------------------------------------------
void qSlicerFeetSegmentationModule::setup()
{
  this->Superclass::setup();
}

//-----------------------------------------------------------------------------
qSlicerAbstractModuleRepresentation* qSlicerFeetSegmentationModule
::createWidgetRepresentation()
{
  return new qSlicerFeetSegmentationModuleWidget;
}

//-----------------------------------------------------------------------------
vtkMRMLAbstractLogic* qSlicerFeetSegmentationModule::createLogic()
{
  return vtkSlicerFeetSegmentationLogic::New();
}

void qSlicerFeetSegmentationModule::updateInputParameters()
{
  qSlicerFeetSegmentationModuleInputs inputs = static_cast<qSlicerFeetSegmentationModuleWidget *>(this->widgetRepresentation())->getInputs();
  if (inputs.rgbInputVolumeNode)
    qDebug() << inputs.rgbInputVolumeNode->GetID();

  if (inputs.depthInputVolumeNode)
    qDebug() << inputs.depthInputVolumeNode->GetID();

//  static_cast<qSlicerFeetSegmentationModuleWidget *>(widgetRepresentation())->get
}
