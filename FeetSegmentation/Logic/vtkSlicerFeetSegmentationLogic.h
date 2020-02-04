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

// Slicer includes
#include "vtkSlicerModuleLogic.h"

// MRML includes
#include "vtkMRMLVectorVolumeNode.h"

// Qt Includes
#include <QDebug>
#include <QImage> // Temporal!

// STD includes
#include <cstdlib>

// PCL Includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <Torch>
#include "vtkSlicerFeetSegmentationModuleLogicExport.h"
#include "FeetSegmentation.h"
#include "vtkFeetSegmentationDepthDataset.h"


/// \ingroup Slicer_QtModules_ExtensionTemplate
class VTK_SLICER_FEETSEGMENTATION_MODULE_LOGIC_EXPORT vtkSlicerFeetSegmentationLogic :
  public vtkSlicerModuleLogic
{
  typedef pcl::PointXYZ Point;
  typedef pcl::PointCloud<Point> PointCloud;
public:

  static vtkSlicerFeetSegmentationLogic *New();
  vtkTypeMacro(vtkSlicerFeetSegmentationLogic, vtkSlicerModuleLogic);
  void PrintSelf(ostream& os, vtkIndent indent);

  /**
   * @brief feetSegmentation, automatic feet segmentation using the algorithm...
   * @param rgbInputNode
   * @param depthInputNode
   * @param outputNode
   */
  void feetSegmentation(vtkMRMLVectorVolumeNode *rgbInputNode, vtkMRMLScalarVolumeNode *depthInputNode,
      vtkMRMLScalarVolumeNode *outputNode);

  /**
   * @brief torchSegmentation
   * @param input
   */
  std::vector<vtkImageData *> torchSegmentation(vtkMRMLVectorVolumeNode *input);

  /**
   * @brief pointCloudFilter
   * @param pointCloud
   */
  PointCloud::Ptr pointCloudStatisticalFilter(vtkFeetSegmentationDepthDataset *pointCloud);

  /**
   * @brief pointCloudFilter
   * @param pointCloud
   */
  std::vector<int> pointCloudStatisticalFilter2(vtkFeetSegmentationDepthDataset *pointCloud);

  /**
  * @brief planeModelSegmentation
  * @param pointCloud
  * @return
  */
 PointCloud::Ptr planeModelSegmentation(PointCloud::Ptr pointCloud);

 /**
  * @brief planeModelSegmentation
  * @param pointCloud
  * @return
  */
 std::vector<int> planeModelSegmentation(vtkFeetSegmentationDepthDataset *pointCloud);

  /**
   * @brief tensorBinarize, binarize a tensor input by thresholding
   * @param tensor
   * @param threshold
   * @return mask tensor
   */
  torch::Tensor tensorBinarize(torch::Tensor tensor, double threshold);

  /**
   * @brief vtkImageToTensor
   * @param data
   * @return
   */
  torch::Tensor vtkImageToTensor(vtkImageData *data);

  /**
   * @brief qImageToTensor
   * @param img
   * @return
   */
  torch::Tensor qImageToTensor(QImage &img);

  // To remove
  void test();
  //To remove
  void testNewVtkImageToPointCloud(vtkMRMLScalarVolumeNode * depthNode, vtkMRMLScalarVolumeNode * outputNode);
  //To remove
  void torchVTKTest(vtkMRMLVectorVolumeNode *node, vtkMRMLScalarVolumeNode *outputNode);
  //To remove
  void pointCloudTest(vtkMRMLScalarVolumeNode *maskNode, vtkMRMLScalarVolumeNode * depthNode);

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

  vtkSlicerFeetSegmentationLogic(const vtkSlicerFeetSegmentationLogic&); // Not implemented
  void operator=(const vtkSlicerFeetSegmentationLogic&); // Not implemented
};

#endif
