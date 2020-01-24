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

// FeetSegmentation Logic includes
#include "vtkSlicerFeetSegmentationLogic.h"
#include "vtkFeetSegmentationDepthDataset.h"
#include "FeetSegmentation.h"
#include "Utils.h"

// MRML includes
#include <vtkMRMLScene.h>

// VTK includes
#include <vtkIntArray.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkImageData.h>

// STD includes
#include <cassert>

// PCL Includes
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <QDebug>

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSlicerFeetSegmentationLogic);

//----------------------------------------------------------------------------
vtkSlicerFeetSegmentationLogic::vtkSlicerFeetSegmentationLogic()
{
}

//----------------------------------------------------------------------------
vtkSlicerFeetSegmentationLogic::~vtkSlicerFeetSegmentationLogic()
{
}

//----------------------------------------------------------------------------
void vtkSlicerFeetSegmentationLogic::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//---------------------------------------------------------------------------
void vtkSlicerFeetSegmentationLogic::SetMRMLSceneInternal(vtkMRMLScene * newScene)
{
  vtkNew<vtkIntArray> events;
  events->InsertNextValue(vtkMRMLScene::NodeAddedEvent);
  events->InsertNextValue(vtkMRMLScene::NodeRemovedEvent);
  events->InsertNextValue(vtkMRMLScene::EndBatchProcessEvent);
  this->SetAndObserveMRMLSceneEventsInternal(newScene, events.GetPointer());
}

//-----------------------------------------------------------------------------
void vtkSlicerFeetSegmentationLogic::RegisterNodes()
{
  assert(this->GetMRMLScene() != 0);
}

//---------------------------------------------------------------------------
void vtkSlicerFeetSegmentationLogic::UpdateFromMRMLScene()
{
  assert(this->GetMRMLScene() != 0);
}

//---------------------------------------------------------------------------
void vtkSlicerFeetSegmentationLogic
::OnMRMLSceneNodeAdded(vtkMRMLNode* vtkNotUsed(node))
{
}

//---------------------------------------------------------------------------
void vtkSlicerFeetSegmentationLogic
::OnMRMLSceneNodeRemoved(vtkMRMLNode* vtkNotUsed(node))
{
}

void vtkSlicerFeetSegmentationLogic::feetSegmentation(
    vtkMRMLVectorVolumeNode *rgbInputNode, vtkMRMLScalarVolumeNode *depthInputNode,
    vtkMRMLScalarVolumeNode *outputNode)
{
  std::vector<vtkImageData *> results = torchSegmentation(rgbInputNode);

  vtkFeetSegmentationDepthDataset pointCloud(depthInputNode->GetImageData(), QSize(512,512));
  pointCloud.applyMask(results[0]);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered = pointCloudFilter(&pointCloud);
  pcl::PointIndices::Ptr indices = planeModelSegmentation(cloudFiltered);
  pointCloud.setInliers(indices);
  vtkImageData * result = pointCloud.getMaskResult();

  outputNode->SetAndObserveImageData(result);
  outputNode->SetIJKToRASDirections(-1,0,0,0,-1,0,0,0,1);
  outputNode->SetOrigin(rgbInputNode->GetOrigin());
  outputNode->SetSpacing(rgbInputNode->GetSpacing());
}

std::vector<vtkImageData *> vtkSlicerFeetSegmentationLogic::torchSegmentation(vtkMRMLVectorVolumeNode *input)
{
  FeetSegmentation torchModel = FeetSegmentation();
  if (input == nullptr)
    return std::vector<vtkImageData *>();

  return torchModel.predict(input);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr vtkSlicerFeetSegmentationLogic::pointCloudFilter(vtkFeetSegmentationDepthDataset *pointCloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (pointCloud->getPointCloud());
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloudFiltered);

  // Just testing
  pcl::io::savePCDFile("maskedPCDOutlierRemoved.pcd", *cloudFiltered);

  return cloudFiltered;
}

pcl::PointIndices::Ptr vtkSlicerFeetSegmentationLogic::planeModelSegmentation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
{
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (100);
  seg.setMaxIterations (1000);

  seg.setInputCloud (pointCloud);
  seg.segment (*inliers, *coefficients);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Extract the inliers
  extract.setInputCloud (pointCloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_p);

  pcl::io::savePCDFile("resutPCD.pcd", *cloud_p);

  return inliers;
}

torch::Tensor vtkSlicerFeetSegmentationLogic::tensorBinarize(torch::Tensor tensor, double threshold)
{
  if (threshold < 0 || threshold > 1)
          throw std::invalid_argument("threshold values must be in the range [0, 1]");

  return torch::where(tensor >= threshold, torch::ones(1), torch::zeros(1));
}

torch::Tensor vtkSlicerFeetSegmentationLogic::vtkImageToTensor(vtkImageData *data)
{
  int *imgSize = data->GetDimensions();

  int nPixels = imgSize[0]*imgSize[1]*data->GetNumberOfScalarComponents();
  uint8_t *bits = reinterpret_cast<uint8_t *>(data->GetScalarPointer());

  torch::Tensor tmpTensor = torch::from_blob(bits, {nPixels},
                                              torch::kByte).clone();

  //img.size().rheight() = imgSize[1]
  //img.size().rwidth() = imgSize[0]
  torch::Tensor imgTensor = torch::zeros({3, imgSize[1], imgSize[0]});
  imgTensor.slice(0, 0, 1) = tmpTensor.slice(0, 0, nPixels, 3).reshape({1, imgSize[1], imgSize[0]});
  imgTensor.slice(0, 1, 2) = tmpTensor.slice(0, 1, nPixels, 3).reshape({1, imgSize[1], imgSize[0]});
  imgTensor.slice(0, 2, 3) = tmpTensor.slice(0, 2, nPixels, 3).reshape({1, imgSize[1], imgSize[0]});

//  torch::Tensor imgTensor = torch::zeros({3, img.size().rheight(), img.size().rwidth()});
//  imgTensor.slice(0, 0, 1) = tmpTensor.slice(0, 0, img.size().rheight()*img.size().rwidth()*3, 3).reshape({1, img.size().rheight(), img.size().rwidth()});
//  imgTensor.slice(0, 1, 2) = tmpTensor.slice(0, 1, img.size().rheight()*img.size().rwidth()*3, 3).reshape({1, img.size().rheight(), img.size().rwidth()});
//  imgTensor.slice(0, 2, 3) = tmpTensor.slice(0, 2, img.size().rheight()*img.size().rwidth()*3, 3).reshape({1, img.size().rheight(), img.size().rwidth()});

//  std::cout << imgTensor.slice(0, 0, 1) << std::endl;

//  imgTensor = imgTensor.to(torch::kFloat32);
//  imgTensor /= 255;

  return imgTensor;
}

torch::Tensor vtkSlicerFeetSegmentationLogic::qImageToTensor(QImage &img)
{
  return Utils::qImageToTensor(img);
}

void vtkSlicerFeetSegmentationLogic::test()
{
  FeetSegmentation torchModel = FeetSegmentation();
  std::vector<QImage> results = torchModel.predict("/home/abian/Data/Tools/Slicer/Modules/DiabeticFootExtension/FeetSegmentation/Testing/Dataset/visible/");

  results[0].save("itsTheFinalTest.png");
}

void vtkSlicerFeetSegmentationLogic::torchVTKTest(vtkMRMLVectorVolumeNode *inputNode, vtkMRMLScalarVolumeNode *outputNode)
{
  FeetSegmentation torchModel = FeetSegmentation();
  std::vector<vtkImageData *> results = torchModel.predict(inputNode);
  outputNode->SetAndObserveImageData(results[0]);

  //Resize.... meh!
//  vtkSmartPointer<vtkImageResize> resize =
//      vtkSmartPointer<vtkImageResize>::New();

//  resize->SetInputData(results[0]);
//  resize->SetOutputDimensions(inputNode->GetImageData()->GetDimensions());
//  resize->Update();
//  vtkImageData *resizedImg = resize->GetOutput();

//  int* data = inputNode->GetImageData()->GetDimensions();
//  qDebug() << "Dimension[0]: " << data[0];
//  qDebug() << "Dimension[1]: "<< data[1];


//  outputNode->SetAndObserveImageData(resizedImg);

  outputNode->SetIJKToRASDirections(-1,0,0,0,-1,0,0,0,1);
  outputNode->SetOrigin(inputNode->GetOrigin());
  outputNode->SetSpacing(inputNode->GetSpacing());
}

void vtkSlicerFeetSegmentationLogic::pointCloudTest(vtkMRMLScalarVolumeNode *maskNode, vtkMRMLScalarVolumeNode * depthNode)
{
  vtkFeetSegmentationDepthDataset pointCloud(depthNode->GetImageData(), QSize(512,512));
  pointCloud.applyMask(maskNode->GetImageData());
//  Utils::vtkImageToPointCloud(depthNode->GetImageData());
}
