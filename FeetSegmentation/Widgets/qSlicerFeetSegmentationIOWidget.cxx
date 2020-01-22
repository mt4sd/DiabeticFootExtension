/*==============================================================================

  Program: 3D Slicer

  Copyright (c) Kitware Inc.

  See COPYRIGHT.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

  This file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc.
  and was partially funded by NIH grant 3P41RR013218-12S1

==============================================================================*/

// IO Widgets includes
#include "qSlicerFeetSegmentationIOWidget.h"
#include "ui_qSlicerFeetSegmentationIOWidget.h"

#include <QDebug>

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_FeetSegmentation
class qSlicerFeetSegmentationIOWidgetPrivate
  : public Ui_qSlicerFeetSegmentationIOWidget
{
  Q_DECLARE_PUBLIC(qSlicerFeetSegmentationIOWidget);
protected:
  qSlicerFeetSegmentationIOWidget* const q_ptr;

public:
  qSlicerFeetSegmentationIOWidgetPrivate(
    qSlicerFeetSegmentationIOWidget& object);
  virtual void setupUi(qSlicerFeetSegmentationIOWidget*);
};

// --------------------------------------------------------------------------
qSlicerFeetSegmentationIOWidgetPrivate
::qSlicerFeetSegmentationIOWidgetPrivate(
  qSlicerFeetSegmentationIOWidget& object)
  : q_ptr(&object)
{
}

// --------------------------------------------------------------------------
void qSlicerFeetSegmentationIOWidgetPrivate
::setupUi(qSlicerFeetSegmentationIOWidget* widget)
{
  this->Ui_qSlicerFeetSegmentationIOWidget::setupUi(widget);
  rgbInputNodeSelector->setAddEnabled(false);
  rgbInputNodeSelector->setNoneEnabled(true);
  rgbInputNodeSelector->setNoneDisplay("Select the RGB image");
  rgbInputNodeSelector->setNodeTypes(QStringList("vtkMRMLVectorVolumeNode"));

  depthInputNodeSelector->setAddEnabled(false);
  depthInputNodeSelector->setNoneEnabled(true);
  depthInputNodeSelector->setShowChildNodeTypes(false);
  depthInputNodeSelector->setNoneDisplay("Select the Depth image");
  depthInputNodeSelector->setNodeTypes(QStringList("vtkMRMLScalarVolumeNode"));

  resultOutputNodeSelector->setShowChildNodeTypes(false);
  resultOutputNodeSelector->setNodeTypes(QStringList("vtkMRMLScalarVolumeNode"));
}

//-----------------------------------------------------------------------------
// qSlicerFeetSegmentationIOWidget methods

//-----------------------------------------------------------------------------
qSlicerFeetSegmentationIOWidget
::qSlicerFeetSegmentationIOWidget(QWidget* parentWidget)
  : Superclass( parentWidget )
  , d_ptr( new qSlicerFeetSegmentationIOWidgetPrivate(*this) )
{
  Q_D(qSlicerFeetSegmentationIOWidget);
  d->setupUi(this);

  QObject::connect(
        d->rgbInputNodeSelector, SIGNAL(currentNodeChanged(vtkMRMLNode *)),
        this, SLOT(inputChanged())
        );

  QObject::connect(
        d->depthInputNodeSelector, SIGNAL(currentNodeChanged(vtkMRMLNode *)),
        this, SLOT(inputChanged())
        );
}

//-----------------------------------------------------------------------------
qSlicerFeetSegmentationIOWidget
::~qSlicerFeetSegmentationIOWidget()
{
}

void qSlicerFeetSegmentationIOWidget::updateMRMLScene(vtkMRMLScene *mrmlScene)
{
  d_ptr->rgbInputNodeSelector->setMRMLScene(mrmlScene);
  d_ptr->depthInputNodeSelector->setMRMLScene(mrmlScene);
  d_ptr->resultOutputNodeSelector->setMRMLScene(mrmlScene);
}

void qSlicerFeetSegmentationIOWidget::inputChanged()
{
  qDebug() << "||| qSlicerFeetSegmentationIOWidget::inputChanged() ||||";
  emit currentInputChanged();
};

vtkMRMLVectorVolumeNode * qSlicerFeetSegmentationIOWidget::getRGBInputNode()
{
  vtkMRMLVectorVolumeNode * output = static_cast<vtkMRMLVectorVolumeNode *>(d_ptr->rgbInputNodeSelector->currentNode());
  std::cout << output;
  if (output == NULL)
    return nullptr;

  return output;
}

vtkMRMLScalarVolumeNode * qSlicerFeetSegmentationIOWidget::getDepthInputNode()
{
  vtkMRMLScalarVolumeNode * output = static_cast<vtkMRMLScalarVolumeNode *>(d_ptr->rgbInputNodeSelector->currentNode());
  if (output == NULL)
    return nullptr;

  return output;
}

