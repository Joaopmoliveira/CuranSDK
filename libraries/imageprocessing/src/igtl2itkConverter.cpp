#include "imageprocessing/igtl2itkConverter.h"
#include "itkCastImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkImage.h"
#include "itkImportImageFilter.h"

namespace curan {
namespace image {

    void igtl2ITK_im_convert(const igtl::ImageMessage::Pointer& imageMessage, itk::Image<unsigned char, 3>::Pointer& image_to_render) {

        using ImportFilterType = itk::ImportImageFilter<unsigned char, 3>;

        ImportFilterType::RegionType region;
        int width, height, depth = 0;
        imageMessage->GetDimensions(width,height,depth);
        ImportFilterType::SizeType size;
            size[0] = width;
            size[1] = height;
            size[2] = depth;
        region.SetSize(size);
        //std::printf("size (%d %d %d)\n", size[0], size[1], size[2]);
        
        ImportFilterType::IndexType start;
        start.Fill(0);
        region.SetIndex(start);

        auto importFilter = ImportFilterType::New();
        importFilter->SetRegion(region);

        igtl::Matrix4x4 local_mat;
        imageMessage->GetMatrix(local_mat);

        itk::Matrix<double,3,3> itk_matrix;
        for(size_t col = 0; col < 3; ++col)
            for(size_t row = 0; row < 3; ++row)
                itk_matrix(row,col) = local_mat[row][col];

        importFilter->SetDirection(itk_matrix);
        float xx, yy, zz = 0.0;
        imageMessage->GetOrigin(xx, yy, zz);
        const itk::SpacePrecisionType origin[3] = { xx,yy,zz};
        importFilter->SetOrigin(origin);
        //std::printf("origin (%f %f %f)\n", origin[0], origin[1], origin[2]);

        float xxx, yyy, zzz = 0.0;
        imageMessage->GetSpacing(xxx, yyy, zzz);
        const itk::SpacePrecisionType spacing[3] = {xxx,yyy,zzz};
        importFilter->SetSpacing(spacing);
        //std::printf("spacing (%f %f %f)\n", spacing[0], spacing[1], spacing[2]);


        const bool importImageFilterWillOwnTheBuffer = false;
        importFilter->SetImportPointer(static_cast<unsigned char*>(imageMessage->GetScalarPointer()), imageMessage->GetImageSize(), importImageFilterWillOwnTheBuffer);

        using FilterType = itk::CastImageFilter<itk::Image<unsigned char, 3>, itk::Image<unsigned char, 3>>;
        auto filter = FilterType::New();
        filter->SetInput(importFilter->GetOutput());

        try{
            filter->Update();
        } catch (const itk::ExceptionObject& e) {
            std::cerr << "Error: " << e << std::endl;
            return;
        }

        image_to_render = filter->GetOutput();

        

    };

}
}