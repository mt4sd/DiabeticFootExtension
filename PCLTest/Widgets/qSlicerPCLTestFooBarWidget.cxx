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
#include "qSlicerPCLTestFooBarWidget.h"
#include "ui_qSlicerPCLTestFooBarWidget.h"

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_PCLTest
class qSlicerPCLTestFooBarWidgetPrivate
  : public Ui_qSlicerPCLTestFooBarWidget
{
  Q_DECLARE_PUBLIC(qSlicerPCLTestFooBarWidget);
protected:
  qSlicerPCLTestFooBarWidget* const q_ptr;

public:
  qSlicerPCLTestFooBarWidgetPrivate(
    qSlicerPCLTestFooBarWidget& object);
  virtual void setupUi(qSlicerPCLTestFooBarWidget*);
};

// --------------------------------------------------------------------------
qSlicerPCLTestFooBarWidgetPrivate
::qSlicerPCLTestFooBarWidgetPrivate(
  qSlicerPCLTestFooBarWidget& object)
  : q_ptr(&object)
{
}

// --------------------------------------------------------------------------
void qSlicerPCLTestFooBarWidgetPrivate
::setupUi(qSlicerPCLTestFooBarWidget* widget)
{
  this->Ui_qSlicerPCLTestFooBarWidget::setupUi(widget);
}

//-----------------------------------------------------------------------------
// qSlicerPCLTestFooBarWidget methods

//-----------------------------------------------------------------------------
qSlicerPCLTestFooBarWidget
::qSlicerPCLTestFooBarWidget(QWidget* parentWidget)
  : Superclass( parentWidget )
  , d_ptr( new qSlicerPCLTestFooBarWidgetPrivate(*this) )
{
  Q_D(qSlicerPCLTestFooBarWidget);
  d->setupUi(this);
}

//-----------------------------------------------------------------------------
qSlicerPCLTestFooBarWidget
::~qSlicerPCLTestFooBarWidget()
{
}
