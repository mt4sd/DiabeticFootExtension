#include "FeetSegmentation.h"
#include "FeetSegmentationDataset.h"
#include "vtkFeetSegmentationDataset.h"
#include "Utils.h"
#include <torch/script.h>

//Tmp
#include <QDebug>

FeetSegmentation::FeetSegmentation() :
  device(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU),
  normalize(torch::data::transforms::Normalize<>({0.485, 0.456, 0.406}, {0.229, 0.224, 0.225}))
{
  loadModel("FeetSegmentation/Resources/TorchScript/ternausnet.pt");
}

std::vector<QImage> FeetSegmentation::predict(QString datasetDir, size_t batchSize)
{
    std::vector<torch::Tensor> batchResults;
    std::vector<QImage> results;

//    InvNormalize<> invNorm({0.485, 0.456, 0.406}, {0.229, 0.224, 0.225});

//    if (model.name().name() != "TernausNet")
//        return results;

    auto dataset = FeetSegmentationDataset(datasetDir)
            .map(normalize)
            .map(torch::data::transforms::Stack<>());

    auto data_loader = torch::data::make_data_loader<torch::data::samplers::SequentialSampler>(
        std::move(dataset),
        torch::data::DataLoaderOptions().batch_size(batchSize).workers(2));


    for (auto & batch : *data_loader)
    {
        auto imgs = batch.data.to(device);
        //auto labels = batch.target.squeeze();

        std::cout << imgs.sizes() << std::endl;

        std::vector<torch::jit::IValue> input;
        input.push_back(imgs);

        torch::Tensor outputTensor = model.forward(input).toTensor();
        outputTensor = torch::sigmoid(outputTensor.detach().to(torch::kCPU));

        batchResults.push_back(outputTensor.detach().to(torch::kCPU));
    }


    for (std::vector<torch::Tensor>::iterator batchIt = batchResults.begin();
         batchIt != batchResults.end();
         ++batchIt)
    {
        for (size_t imgIdx = 0; imgIdx < (*batchIt).size(0); ++imgIdx){
            torch::Tensor img = (*batchIt)[imgIdx];
            img = Utils::binarize(img, 0.75);
            results.push_back(Utils::tensorToQImage(img, QImage::Format_Grayscale8));
        }
    }

    return results;
}

std::vector<vtkImageData *> FeetSegmentation::predict(vtkMRMLVectorVolumeNode *datasetNode, size_t batchSize)
{
  std::vector<torch::Tensor> batchResults;
  std::vector<vtkImageData *> results;

  auto dataset = vtkFeetSegmentationDataset(datasetNode)
          .map(normalize)
          .map(torch::data::transforms::Stack<>());

  auto data_loader = torch::data::make_data_loader<torch::data::samplers::SequentialSampler>(
      std::move(dataset),
      torch::data::DataLoaderOptions().batch_size(batchSize).workers(2));


  for (auto & batch : *data_loader)
  {
      auto imgs = batch.data.to(device);
      //auto labels = batch.target.squeeze();

      std::cout << imgs.sizes() << std::endl;

      std::vector<torch::jit::IValue> input;
      input.push_back(imgs);

      torch::Tensor outputTensor = model.forward(input).toTensor();
      outputTensor = torch::sigmoid(outputTensor.detach().to(torch::kCPU));

      batchResults.push_back(outputTensor.detach().to(torch::kCPU));
  }

  for (std::vector<torch::Tensor>::iterator batchIt = batchResults.begin();
       batchIt != batchResults.end();
       ++batchIt)
  {
      for (int imgIdx = 0; imgIdx < (*batchIt).size(0); ++imgIdx){
          torch::Tensor img = (*batchIt)[imgIdx];
          img = Utils::binarize(img, 0.75);
          results.push_back(Utils::tensorToVtkImage(img));
      }
  }

  return results;
}

int FeetSegmentation::loadModel(QString modelFile)
{
  try {
      model = torch::jit::load(modelFile.toStdString());
  } catch (const torch::Error& e) {
      std::cerr << "error loading the Torch model" << std::endl;
      return -1;
  }
  return 0;
}
