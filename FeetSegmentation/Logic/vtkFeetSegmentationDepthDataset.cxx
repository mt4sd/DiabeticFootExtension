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
  : depthImg(vtkImageData::New()), cloud((new PointCloud())),
    maskResult(vtkImageData::New()), img2pc(Eigen::Affine3f::Identity())
{

  if (resize != QSize(0,0))
  {
    vtkSmartPointer<vtkImageResize> vtkResize =
      vtkSmartPointer<vtkImageResize>::New();

    vtkResize->SetInputData(depthImg);
    vtkResize->SetOutputDimensions(resize.width(), resize.height(), 1);
    vtkResize->Update();

    this->depthImg->DeepCopy(vtkResize->GetOutput());
  } else {
    this->depthImg->DeepCopy(depthImg);
  }

  cloud->width = this->depthImg->GetDimensions()[0];
  cloud->height = this->depthImg->GetDimensions()[1];
  cloud->resize(cloud->width * cloud->height);

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
  if (cloud->size() != 0)
    pcl::io::savePCDFile("preMaskedPCD.pcd", *cloud);

  cloud->clear();

  uint16_t *data = reinterpret_cast<uint16_t *>(img->GetScalarPointer());
  int *dimensions = img->GetDimensions();

  pcl::PointCloud<pcl::PointXYZ> pointCloud;
  for (int x = 0; x < dimensions[0]; ++x)
  {
    for (int y=0; y < dimensions[1]; ++y)
    {
      uint16_t depthPixel = data[ y * dimensions[0] + x];
      if (depthPixel != 0)
        cloud->push_back(pcl::PointXYZ(x, y, depthPixel));
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud(*cloud, *cloud, img2pc);

  pcl::io::savePCDFile("maskedPCD.pcd", *cloud);
}

// ---------------------------------------------------------------
void vtkFeetSegmentationDepthDataset::pointCloudToVtkImageData(PointCloud::Ptr pointCloud)
{
  vtkImageData * depthImg = vtkImageData::New();
  depthImg->SetDimensions(pointCloud->width, pointCloud->height, 1);
  depthImg->SetSpacing(1.0, 1.0, 1.0);
  depthImg->SetOrigin(.0, .0, .0);
  depthImg->AllocateScalars(VTK_UNSIGNED_SHORT, 1);

  size_t nPixels = depthImg->GetDimensions()[0]*depthImg->GetDimensions()[1];
  size_t stride = depthImg->GetDimensions()[0];
  uint16_t data[nPixels];

  for(PointCloud::iterator it = pointCloud->begin(); it!= pointCloud->end(); ++it)
  {
    Point point = *it;
    int x = point._PointXYZ::x;
    int y = point._PointXYZ::y;
    data[y * stride + x] = point._PointXYZ::z;
  }

  std::memcpy(depthImg->GetScalarPointer(), data, sizeof(uint16_t) * nPixels);
}

// ---------------------------------------------------------------
void vtkFeetSegmentationDepthDataset::setInliers(pcl::PointIndices::Ptr indices)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (indices);
  extract.setNegative (false);
  extract.filter (*cloud_p);

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

