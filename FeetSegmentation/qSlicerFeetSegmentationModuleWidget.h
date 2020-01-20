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

#ifndef __qSlicerFeetSegmentationModuleWidget_h
#define __qSlicerFeetSegmentationModuleWidget_h

// Slicer includes
#include <vtkMRMLScalarVolumeNode.h>
#include <vtkMRMLVectorVolumeNode.h>

// SlicerQt includes
#include "qSlicerAbstractModuleWidget.h"

#include "qSlicerFeetSegmentationModuleExport.h"

class qSlicerFeetSegmentationModuleWidgetPrivate;
class vtkMRMLNode;

struct qSlicerFeetSegmentationModuleInputs
{
  vtkMRMLVectorVolumeNode * rgbInputVolumeNode;
  vtkMRMLScalarVolumeNode * depthInputVolumeNode;
};

/// \ingroup Slicer_QtModules_ExtensionTemplate
class Q_SLICER_QTMODULES_FEETSEGMENTATION_EXPORT qSlicerFeetSegmentationModuleWidget :
  public qSlicerAbstractModuleWidget
{
  Q_OBJECT

public:

  typedef qSlicerAbstractModuleWidget Superclass;
  qSlicerFeetSegmentationModuleWidget(QWidget *parent=0);
  virtual ~qSlicerFeetSegmentationModuleWidget();

  qSlicerFeetSegmentationModuleInputs getInputs();

public slots:
  void elTest();

signals:
  void currentInputChanged();

protected:
  QScopedPointer<qSlicerFeetSegmentationModuleWidgetPrivate> d_ptr;

  virtual void setup();

private:
  Q_DECLARE_PRIVATE(qSlicerFeetSegmentationModuleWidget);
  Q_DISABLE_COPY(qSlicerFeetSegmentationModuleWidget);
};

#endif
