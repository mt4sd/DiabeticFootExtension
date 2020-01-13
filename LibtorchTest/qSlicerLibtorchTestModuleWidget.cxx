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

// SlicerQt includes
#include "qSlicerLibtorchTestModuleWidget.h"
#include "ui_qSlicerLibtorchTestModuleWidget.h"

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ExtensionTemplate
class qSlicerLibtorchTestModuleWidgetPrivate: public Ui_qSlicerLibtorchTestModuleWidget
{
public:
  qSlicerLibtorchTestModuleWidgetPrivate();
};

//-----------------------------------------------------------------------------
// qSlicerLibtorchTestModuleWidgetPrivate methods

//-----------------------------------------------------------------------------
qSlicerLibtorchTestModuleWidgetPrivate::qSlicerLibtorchTestModuleWidgetPrivate()
{
}

//-----------------------------------------------------------------------------
// qSlicerLibtorchTestModuleWidget methods

//-----------------------------------------------------------------------------
qSlicerLibtorchTestModuleWidget::qSlicerLibtorchTestModuleWidget(QWidget* _parent)
  : Superclass( _parent )
  , d_ptr( new qSlicerLibtorchTestModuleWidgetPrivate )
{
}

//-----------------------------------------------------------------------------
qSlicerLibtorchTestModuleWidget::~qSlicerLibtorchTestModuleWidget()
{
}

//-----------------------------------------------------------------------------
void qSlicerLibtorchTestModuleWidget::setup()
{
  Q_D(qSlicerLibtorchTestModuleWidget);
  d->setupUi(this);
  this->Superclass::setup();
}
