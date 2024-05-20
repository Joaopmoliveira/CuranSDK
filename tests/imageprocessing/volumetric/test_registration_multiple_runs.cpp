#include "imageprocessing/IntegratedVolumeReconstructor.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include <optional>

constexpr long volsize = 100;
float spacing[3] = {0.01, 0.01, 0.01};

using imagetype = itk::Image<unsigned char, 3>;

imagetype::Pointer create_cone_image()
{
  itk::Matrix<double> image_orientation;
  itk::Point<double> image_origin;

  image_orientation[0][0] = 1.0;
  image_orientation[1][0] = 0.0;
  image_orientation[2][0] = 0.0;

  image_orientation[0][1] = 0.0;
  image_orientation[1][1] = 1.0;
  image_orientation[2][1] = 0.0;

  image_orientation[0][2] = 0.0;
  image_orientation[1][2] = 0.0;
  image_orientation[2][2] = 1.0;

  image_origin[0] = 0.0;
  image_origin[1] = 0.0;
  image_origin[2] = 0.0;

  imagetype::Pointer image = imagetype::New();
  imagetype::IndexType start;
  start[0] = 0; // first index on X
  start[1] = 0; // first index on Y
  start[2] = 0; // first index on Z

  imagetype::SizeType size;
  size[0] = volsize;  // size along X
  size[1] = volsize; // size along Y
  size[2] = volsize;      // size along Z

  imagetype::RegionType region_1;
  region_1.SetSize(size);
  region_1.SetIndex(start);

  image->SetRegions(region_1);
  image->SetDirection(image_orientation);
  image->SetOrigin(image_origin);
  image->SetSpacing(spacing);
  image->Allocate();

  ImageType::IndexType requestedIndex =
      image->GetRequestedRegion().GetIndex();
  ImageType::SizeType requestedSize =
      image->GetRequestedRegion().GetSize();

  IteratorType outputIt( outputImage, outputImage->GetRequestedRegion() );

  for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt){
    ImageType::IndexType idx = outputIt.GetIndex();
    if(idx[0]*idx[0]+idx[1]*idx[1]<idx[2]*idx[2])
      outputIt.Set(255);
  }

  return image;
}