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

#ifndef __qSlicerPCLTestFooBarWidget_h
#define __qSlicerPCLTestFooBarWidget_h

// Qt includes
#include <QWidget>

// FooBar Widgets includes
#include "qSlicerPCLTestModuleWidgetsExport.h"

class qSlicerPCLTestFooBarWidgetPrivate;

/// \ingroup Slicer_QtModules_PCLTest
class Q_SLICER_MODULE_PCLTEST_WIDGETS_EXPORT qSlicerPCLTestFooBarWidget
  : public QWidget
{
  Q_OBJECT
public:
  typedef QWidget Superclass;
  qSlicerPCLTestFooBarWidget(QWidget *parent=0);
  virtual ~qSlicerPCLTestFooBarWidget();

protected slots:

protected:
  QScopedPointer<qSlicerPCLTestFooBarWidgetPrivate> d_ptr;

private:
  Q_DECLARE_PRIVATE(qSlicerPCLTestFooBarWidget);
  Q_DISABLE_COPY(qSlicerPCLTestFooBarWidget);
};

#endif
