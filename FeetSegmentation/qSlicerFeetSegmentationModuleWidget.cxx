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

// Qt includes
#include <QDebug>

//Module includes
#import "vtkSlicerFeetSegmentationLogic.h"

// SlicerQt includes
#include "qSlicerFeetSegmentationModuleWidget.h"
#include "ui_qSlicerFeetSegmentationModuleWidget.h"

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ExtensionTemplate
class qSlicerFeetSegmentationModuleWidgetPrivate: public Ui_qSlicerFeetSegmentationModuleWidget
{
  Q_DECLARE_PUBLIC(qSlicerFeetSegmentationModuleWidget);
protected:
  qSlicerFeetSegmentationModuleWidget* const q_ptr;
public:
  qSlicerFeetSegmentationModuleWidgetPrivate(qSlicerFeetSegmentationModuleWidget &obj);
  vtkSlicerFeetSegmentationLogic* logic() const;

public slots:
  void test();

};

//-----------------------------------------------------------------------------
// qSlicerFeetSegmentationModuleWidgetPrivate methods

//-----------------------------------------------------------------------------
qSlicerFeetSegmentationModuleWidgetPrivate::qSlicerFeetSegmentationModuleWidgetPrivate(qSlicerFeetSegmentationModuleWidget &obj)
  : q_ptr(&obj)
{
}

vtkSlicerFeetSegmentationLogic* qSlicerFeetSegmentationModuleWidgetPrivate::logic() const
{
  Q_Q(const qSlicerFeetSegmentationModuleWidget);
  return vtkSlicerFeetSegmentationLogic::SafeDownCast(q->logic());
}

void qSlicerFeetSegmentationModuleWidgetPrivate::test()
{
  logic()->test();
}

//-----------------------------------------------------------------------------
// qSlicerFeetSegmentationModuleWidget methods

//-----------------------------------------------------------------------------
qSlicerFeetSegmentationModuleWidget::qSlicerFeetSegmentationModuleWidget(QWidget* _parent)
  : Superclass( _parent )
  , d_ptr( new qSlicerFeetSegmentationModuleWidgetPrivate(*this) )
{

}

//-----------------------------------------------------------------------------
qSlicerFeetSegmentationModuleWidget::~qSlicerFeetSegmentationModuleWidget()
{
}

//-----------------------------------------------------------------------------
void qSlicerFeetSegmentationModuleWidget::setup()
{
  Q_D(qSlicerFeetSegmentationModuleWidget);
  d->setupUi(this);
  this->Superclass::setup();

  QObject::connect(this, &qSlicerFeetSegmentationModuleWidget::mrmlSceneChanged,
                   d->IOWidget, &qSlicerFeetSegmentationIOWidget::updateMRMLScene);

  QObject::connect(
      d->IOWidget, &qSlicerFeetSegmentationIOWidget::currentInputChanged,
      [=]() { qDebug() << "La prueba!"; emit currentInputChanged(); }
  );

  QObject::connect(
    d->applyButton, SIGNAL(clicked()), this, SLOT(elTest())
  );
}

qSlicerFeetSegmentationModuleInputs qSlicerFeetSegmentationModuleWidget::getInputs()
{
  Q_D(qSlicerFeetSegmentationModuleWidget);
  qSlicerFeetSegmentationModuleInputs inputs;
  inputs.rgbInputVolumeNode = d->IOWidget->getRGBInputNode();
  inputs.depthInputVolumeNode = d->IOWidget->getDepthInputNode();

  return inputs;
}

void qSlicerFeetSegmentationModuleWidget::elTest()
{
  Q_D(qSlicerFeetSegmentationModuleWidget);
  d->logic()->test();
}

