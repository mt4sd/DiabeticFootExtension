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

// .NAME vtkSlicerFeetSegmentationLogic - slicer logic class for volumes manipulation
// .SECTION Description
// This class manages the logic associated with reading, saving,
// and changing propertied of the volumes


#ifndef __vtkSlicerFeetSegmentationLogic_h
#define __vtkSlicerFeetSegmentationLogic_h

// To remove
#include <QDebug>
#include <QImage>

// Slicer includes
#include "vtkSlicerModuleLogic.h"

// MRML includes
#include "vtkMRMLVectorVolumeNode.h"

// STD includes
#include <cstdlib>

#include <Torch>
#include "vtkSlicerFeetSegmentationModuleLogicExport.h"
#include "FeetSegmentation.h"


/// \ingroup Slicer_QtModules_ExtensionTemplate
class VTK_SLICER_FEETSEGMENTATION_MODULE_LOGIC_EXPORT vtkSlicerFeetSegmentationLogic :
  public vtkSlicerModuleLogic
{
public:

  static vtkSlicerFeetSegmentationLogic *New();
  vtkTypeMacro(vtkSlicerFeetSegmentationLogic, vtkSlicerModuleLogic);
  void PrintSelf(ostream& os, vtkIndent indent);

  /**
   * @brief torchSegmentation
   * @param input
   */
  //void torchSegmentation(vtkMRMLVectorVolumeNode *input);
  torch::Tensor torchSegmentation(vtkMRMLVectorVolumeNode *input);

  /**
   * @brief tensorBinarize, binarize a tensor input by thresholding
   * @param tensor
   * @param threshold
   * @return mask tensor
   */
  torch::Tensor tensorBinarize(torch::Tensor tensor, double threshold);

  // Tmp
  torch::Tensor qImageToTensor(QImage &img);
  void test();

  void torchVTKTest(vtkMRMLVectorVolumeNode *node, vtkMRMLScalarVolumeNode *outputNode);

  void pointCloudTest(vtkMRMLScalarVolumeNode *depthNode);

protected:
  vtkSlicerFeetSegmentationLogic();
  virtual ~vtkSlicerFeetSegmentationLogic();

  virtual void SetMRMLSceneInternal(vtkMRMLScene* newScene);
  /// Register MRML Node classes to Scene. Gets called automatically when the MRMLScene is attached to this logic class.
  virtual void RegisterNodes();
  virtual void UpdateFromMRMLScene();
  virtual void OnMRMLSceneNodeAdded(vtkMRMLNode* node);
  virtual void OnMRMLSceneNodeRemoved(vtkMRMLNode* node);
private:
  /**
   * @brief vtkImageToTensor
   * @param data
   */
  torch::Tensor vtkImageToTensor(vtkImageData *data);

  vtkSlicerFeetSegmentationLogic(const vtkSlicerFeetSegmentationLogic&); // Not implemented
  void operator=(const vtkSlicerFeetSegmentationLogic&); // Not implemented
};

#endif
