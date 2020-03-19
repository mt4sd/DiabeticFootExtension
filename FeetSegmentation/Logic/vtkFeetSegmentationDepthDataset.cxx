#include "vtkFeetSegmentationDepthDataset.h"

#include <vtkSmartPointer.h>
#include <vtkImageResize.h>
#include <vtkImageMask.h>

// PCL Includes
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

// Qt Includes
#include <QtConcurrent/QtConcurrentMap>
#include <QFuture>
#include <QVector>


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

  vtkImageToPointCloud(this->depthImg);
}

vtkFeetSegmentationDepthDataset::~vtkFeetSegmentationDepthDataset()
{
  depthImg->Delete();
  maskResult->Delete();
}

// ---------------------------------------------------------------
void vtkFeetSegmentationDepthDataset::applyMask(vtkImageData *mask)
{
  // Tmp
  pcl::io::savePCDFile("preMaskedPCD.pcd", *cloud);

  vtkSmartPointer<vtkImageMask> maskFilter =
      vtkSmartPointer<vtkImageMask>::New();
  maskFilter->SetInput1Data(depthImg);
  maskFilter->SetInput2Data(mask);
  maskFilter->Update();
  depthImg->DeepCopy(maskFilter->GetOutput());

  vtkImageToPointCloud(depthImg);
  pcl::transformPointCloud(*cloud, *cloud, img2pc.inverse());

  // It is necessary to remove the points with Z value to 0
  pcl::PassThrough<Point> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 0.0);
  pass.setNegative(true);
  pass.filter(*cloud);

  pcl::transformPointCloud(*cloud, *cloud, img2pc);

  // Tmp
  pcl::io::savePCDFile("maskedPCD.pcd", *cloud);
}

// ---------------------------------------------------------------
void vtkFeetSegmentationDepthDataset::vtkImageToPointCloud(vtkImageData *img)
{
  cloud->clear();

  cloud->width = img->GetDimensions()[0];
  cloud->height = img->GetDimensions()[1];
  cloud->resize(cloud->width * cloud->height);


  int *dimensions = img->GetDimensions();
  uint16_t *depthData = reinterpret_cast<uint16_t *>(img->GetScalarPointer());

  size_t currentRow = 0;

  // Lambda function (generatePoint)
  std::function<Point(const size_t &wIdx)> generatePoint =
        [ &depthData, &dimensions, &currentRow ](const size_t &wIdx)
  {
    return Point(wIdx, currentRow, depthData[currentRow * dimensions[0] + wIdx]);
  };

  std::vector<int> pixelsIdx_w(dimensions[0]);
  std::iota(pixelsIdx_w.begin(), pixelsIdx_w.end(), 0);
  for (; currentRow < static_cast<size_t>(dimensions[1]); ++currentRow)
  {
    QFuture<Point> mapper = QtConcurrent::mapped(pixelsIdx_w.begin(), pixelsIdx_w.end(), generatePoint);
    QVector<Point> results = mapper.results().toVector();

    Point *pcData = &(cloud->points.data()[currentRow*dimensions[0]]);
    std::copy(results.begin(), results.end(), pcData);
  }

  // Remove points with the value of Z component is 0
  pcl::PassThrough<Point> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 0.0);
  pass.setNegative(true);
  pass.filter(*cloud);

  pcl::transformPointCloud(*cloud, *cloud, img2pc);
}

// ---------------------------------------------------------------
void vtkFeetSegmentationDepthDataset::pointCloudToVtkImage(PointCloud::Ptr pointCloud)
{
  vtkImageData * depthImg = vtkImageData::New();
//  depthImg->SetDimensions(pointCloud->width, pointCloud->height, 1);
  depthImg->SetDimensions(this->depthImg->GetDimensions());
  depthImg->SetSpacing(1.0, 1.0, 1.0);
  depthImg->SetOrigin(.0, .0, .0);
  depthImg->AllocateScalars(VTK_UNSIGNED_SHORT, 1);

  size_t nPixels = depthImg->GetDimensions()[0]*depthImg->GetDimensions()[1];
  size_t stride = depthImg->GetDimensions()[0];

  uint16_t* data = new uint16_t[nPixels];
  std::memset(data, 0, sizeof(uint16_t) * nPixels);

  pcl::transformPointCloud(*cloud, *cloud, img2pc.inverse());

  for(PointCloud::iterator it = pointCloud->begin(); it!= pointCloud->end(); ++it)
  {
    Point point = *it;
    int x = point._PointXYZ::x;
    int y = point._PointXYZ::y;
    data[y * stride + x] = point._PointXYZ::z;
  }

  pcl::transformPointCloud(*cloud, *cloud, img2pc);

  std::memcpy(depthImg->GetScalarPointer(), data, sizeof(uint16_t) * nPixels);

  this->depthImg->DeepCopy(depthImg);
  depthImg->Delete();
  delete[] data;
}

// ---------------------------------------------------------------
void vtkFeetSegmentationDepthDataset::setInliers(std::vector<int> indices)
{
  pcl::IndicesPtr indicesPtr(new std::vector<int>(indices));

  // Create the filtering object
  pcl::ExtractIndices<Point> extract;
  // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (indicesPtr);
  extract.setNegative (false);
  extract.filter (*cloud);

  // Update the vtkImageData representation
  pointCloudToVtkImage(cloud);
}
