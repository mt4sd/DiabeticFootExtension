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

#ifndef __qSlicerFeetSegmentationIOWidget_h
#define __qSlicerFeetSegmentationIOWidget_h

// Qt includes
#include <QWidget>

// Slicer includes
#include <vtkMRMLScene.h>
#include <vtkMRMLVectorVolumeNode.h>

// FooBar Widgets includes
#include "qSlicerFeetSegmentationModuleWidgetsExport.h"

class qSlicerFeetSegmentationIOWidgetPrivate;

/// \ingroup Slicer_QtModules_FeetSegmentation
class Q_SLICER_MODULE_FEETSEGMENTATION_WIDGETS_EXPORT qSlicerFeetSegmentationIOWidget
  : public QWidget
{
  Q_OBJECT
public:
  typedef QWidget Superclass;
  qSlicerFeetSegmentationIOWidget(QWidget *parent=0);
  virtual ~qSlicerFeetSegmentationIOWidget();

  vtkMRMLVectorVolumeNode * getRGBInputNode();
  vtkMRMLScalarVolumeNode * getDepthInputNode();

signals:
  void currentInputChanged();

public slots:
  void updateMRMLScene(vtkMRMLScene *);
  void inputChanged();


protected slots:

protected:
  QScopedPointer<qSlicerFeetSegmentationIOWidgetPrivate> d_ptr;

private:
  Q_DECLARE_PRIVATE(qSlicerFeetSegmentationIOWidget);
  Q_DISABLE_COPY(qSlicerFeetSegmentationIOWidget);
};

#endif
