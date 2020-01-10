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

// FooBar Widgets includes
#include "qSlicerFeetSegmentationFooBarWidget.h"
#include "ui_qSlicerFeetSegmentationFooBarWidget.h"

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_FeetSegmentation
class qSlicerFeetSegmentationFooBarWidgetPrivate
  : public Ui_qSlicerFeetSegmentationFooBarWidget
{
  Q_DECLARE_PUBLIC(qSlicerFeetSegmentationFooBarWidget);
protected:
  qSlicerFeetSegmentationFooBarWidget* const q_ptr;

public:
  qSlicerFeetSegmentationFooBarWidgetPrivate(
    qSlicerFeetSegmentationFooBarWidget& object);
  virtual void setupUi(qSlicerFeetSegmentationFooBarWidget*);
};

// --------------------------------------------------------------------------
qSlicerFeetSegmentationFooBarWidgetPrivate
::qSlicerFeetSegmentationFooBarWidgetPrivate(
  qSlicerFeetSegmentationFooBarWidget& object)
  : q_ptr(&object)
{
}

// --------------------------------------------------------------------------
void qSlicerFeetSegmentationFooBarWidgetPrivate
::setupUi(qSlicerFeetSegmentationFooBarWidget* widget)
{
  this->Ui_qSlicerFeetSegmentationFooBarWidget::setupUi(widget);
}

//-----------------------------------------------------------------------------
// qSlicerFeetSegmentationFooBarWidget methods

//-----------------------------------------------------------------------------
qSlicerFeetSegmentationFooBarWidget
::qSlicerFeetSegmentationFooBarWidget(QWidget* parentWidget)
  : Superclass( parentWidget )
  , d_ptr( new qSlicerFeetSegmentationFooBarWidgetPrivate(*this) )
{
  Q_D(qSlicerFeetSegmentationFooBarWidget);
  d->setupUi(this);
}

//-----------------------------------------------------------------------------
qSlicerFeetSegmentationFooBarWidget
::~qSlicerFeetSegmentationFooBarWidget()
{
}
