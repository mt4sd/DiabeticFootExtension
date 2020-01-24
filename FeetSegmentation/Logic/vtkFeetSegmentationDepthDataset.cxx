#include "vtkFeetSegmentationDepthDataset.h"

#include <vtkSmartPointer.h>
#include <vtkImageResize.h>
#include <vtkImageMask.h>

// PCL Includes
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

//Tmp
#include <QDebug>

vtkFeetSegmentationDepthDataset::vtkFeetSegmentationDepthDataset(vtkImageData *depthImg, QSize resize)
  : pc((new pcl::PointCloud<pcl::PointXYZ>())), depthImg(depthImg), maskResult(vtkImageData::New()),
    img2pc(Eigen::Affine3f::Identity())
{

  if (resize != QSize(0,0))
  {
    vtkSmartPointer<vtkImageResize> vtkResize =
      vtkSmartPointer<vtkImageResize>::New();

    vtkResize->SetInputData(depthImg);
    vtkResize->SetOutputDimensions(resize.width(), resize.height(), 1);
    vtkResize->Update();

    this->depthImg = vtkImageData::New();
    this->depthImg->ShallowCopy(vtkResize->GetOutput());
  }

  //Define Image to point cloud Transform
  img2pc.scale(Eigen::Vector3f(1., 1., 1));
  img2pc.rotate (Eigen::AngleAxisf (static_cast<float>(M_PI), Eigen::Vector3f::UnitX()));

  generatePointCloud(this->depthImg);
}

vtkFeetSegmentationDepthDataset::~vtkFeetSegmentationDepthDataset()
{
  depthImg->Delete();
  maskResult->Delete();
}

// ---------------------------------------------------------------
void vtkFeetSegmentationDepthDataset::applyMask(vtkImageData *mask)
{
  vtkSmartPointer<vtkImageMask> maskFilter =
      vtkSmartPointer<vtkImageMask>::New();
  maskFilter->SetInput1Data(depthImg);
  maskFilter->SetInput2Data(mask);
  maskFilter->Update();
  depthImg->DeepCopy(maskFilter->GetOutput());

  generatePointCloud(depthImg);
}

// ---------------------------------------------------------------
void vtkFeetSegmentationDepthDataset::generatePointCloud(vtkImageData *img)
{
  if (pc->size() != 0)
    pcl::io::savePCDFile("preMaskedPCD.pcd", *pc);

  pc->clear();

  uint16_t *data = reinterpret_cast<uint16_t *>(img->GetScalarPointer());
  int *dimensions = img->GetDimensions();

  pcl::PointCloud<pcl::PointXYZ> pointCloud;
  for (int x = 0; x < dimensions[0]; ++x)
  {
    for (int y=0; y < dimensions[1]; ++y)
    {
      uint16_t depthPixel = data[ y * dimensions[0] + x];
      if (depthPixel != 0)
        pc->push_back(pcl::PointXYZ(x, y, depthPixel));
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud(*pc, *pc, img2pc);

  pcl::io::savePCDFile("maskedPCD.pcd", *pc);
}

// ---------------------------------------------------------------
void vtkFeetSegmentationDepthDataset::setInliers(pcl::PointIndices::Ptr indices)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Extract the inliers
  extract.setInputCloud (pc);
  extract.setIndices (indices);
  extract.setNegative (false);
  extract.filter (*cloud_p);

  qDebug() << "Voy a ver que son los indices: " << indices->indices[0];

  //invert point cloud
  pcl::transformPointCloud(*cloud_p, *cloud_p, img2pc.inverse());

  vtkImageData *img = vtkImageData::New();
  img->SetDimensions(depthImg->GetDimensions());
  img->SetSpacing(1.0, 1.0, 1.0);
  img->SetOrigin(.0, .0, .0);

  img->AllocateScalars(VTK_UNSIGNED_CHAR, 1);

  size_t nPixel = depthImg->GetDimensions()[0]*depthImg->GetDimensions()[1];
  uint8_t data[nPixel];
  std::memset(data, 0, nPixel);

  size_t stride = depthImg->GetDimensions()[0];
  for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_p->begin(); it!= cloud_p->end(); ++it)
  {
    pcl::PointXYZ point = *it;
    int x = point._PointXYZ::x;
    int y = point._PointXYZ::y;
    data[ y * stride + x] = 255;
  }

  std::memcpy(img->GetScalarPointer(), data, sizeof(uint8_t) * nPixel);
  maskResult->DeepCopy(img);
}
