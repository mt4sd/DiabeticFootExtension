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
#include "qSlicerLibtorchTestFooBarWidget.h"
#include "ui_qSlicerLibtorchTestFooBarWidget.h"

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_LibtorchTest
class qSlicerLibtorchTestFooBarWidgetPrivate
  : public Ui_qSlicerLibtorchTestFooBarWidget
{
  Q_DECLARE_PUBLIC(qSlicerLibtorchTestFooBarWidget);
protected:
  qSlicerLibtorchTestFooBarWidget* const q_ptr;

public:
  qSlicerLibtorchTestFooBarWidgetPrivate(
    qSlicerLibtorchTestFooBarWidget& object);
  virtual void setupUi(qSlicerLibtorchTestFooBarWidget*);
};

// --------------------------------------------------------------------------
qSlicerLibtorchTestFooBarWidgetPrivate
::qSlicerLibtorchTestFooBarWidgetPrivate(
  qSlicerLibtorchTestFooBarWidget& object)
  : q_ptr(&object)
{
}

// --------------------------------------------------------------------------
void qSlicerLibtorchTestFooBarWidgetPrivate
::setupUi(qSlicerLibtorchTestFooBarWidget* widget)
{
  this->Ui_qSlicerLibtorchTestFooBarWidget::setupUi(widget);
}

//-----------------------------------------------------------------------------
// qSlicerLibtorchTestFooBarWidget methods

//-----------------------------------------------------------------------------
qSlicerLibtorchTestFooBarWidget
::qSlicerLibtorchTestFooBarWidget(QWidget* parentWidget)
  : Superclass( parentWidget )
  , d_ptr( new qSlicerLibtorchTestFooBarWidgetPrivate(*this) )
{
  Q_D(qSlicerLibtorchTestFooBarWidget);
  d->setupUi(this);
}

//-----------------------------------------------------------------------------
qSlicerLibtorchTestFooBarWidget
::~qSlicerLibtorchTestFooBarWidget()
{
}
