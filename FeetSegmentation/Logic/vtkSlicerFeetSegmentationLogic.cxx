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
#include "FeetSegmentation.h"

// MRML includes
#include <vtkMRMLScene.h>

// VTK includes
#include <vtkIntArray.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkImageData.h>

// STD includes
#include <cassert>

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

torch::Tensor vtkSlicerFeetSegmentationLogic::torchSegmentation(vtkMRMLVectorVolumeNode *input)
{
  if (input == nullptr)
    return torch::zeros({1});

  vtkImageData *rgbImg = input->GetImageData();
  torch::Tensor tensor = vtkImageToTensor(rgbImg);

  // Meh
  return tensor;
  //TODO
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

  //Los datos se ve tal cual la imagen, fila x columna.
//  qDebug() << bits[0];
//  qDebug() << bits[1];
//  qDebug() << bits[2];
//  qDebug() << "------------------------";
//  qDebug() << bits[nPixels-3];
//  qDebug() << bits[nPixels-2];
//  qDebug() << bits[nPixels-1];
  return imgTensor;
}

#include "Utils.h"
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

void vtkSlicerFeetSegmentationLogic::torchVTKTest(vtkMRMLVectorVolumeNode *node)
{
  FeetSegmentation torchModel = FeetSegmentation();
  std::vector<QImage> results = torchModel.predict(node);

  qDebug() << results.size();
  results[0].save("itsTheFinalVTKTest.png");
}

