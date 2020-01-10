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

#ifndef __qSlicerFeetSegmentationFooBarWidget_h
#define __qSlicerFeetSegmentationFooBarWidget_h

// Qt includes
#include <QWidget>

// FooBar Widgets includes
#include "qSlicerFeetSegmentationModuleWidgetsExport.h"

class qSlicerFeetSegmentationFooBarWidgetPrivate;

/// \ingroup Slicer_QtModules_FeetSegmentation
class Q_SLICER_MODULE_FEETSEGMENTATION_WIDGETS_EXPORT qSlicerFeetSegmentationFooBarWidget
  : public QWidget
{
  Q_OBJECT
public:
  typedef QWidget Superclass;
  qSlicerFeetSegmentationFooBarWidget(QWidget *parent=0);
  virtual ~qSlicerFeetSegmentationFooBarWidget();

protected slots:

protected:
  QScopedPointer<qSlicerFeetSegmentationFooBarWidgetPrivate> d_ptr;

private:
  Q_DECLARE_PRIVATE(qSlicerFeetSegmentationFooBarWidget);
  Q_DISABLE_COPY(qSlicerFeetSegmentationFooBarWidget);
};

#endif
