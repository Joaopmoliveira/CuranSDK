#define STB_IMAGE_IMPLEMENTATION
#include <unordered_map>
#include <optional>
#include <charconv>
#include <functional>

#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkResampleImageFilter.h"
#include "itkScaleTransform.h"
#include "itkAffineTransform.h"

#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"

#include <Eigen/Dense>

std::optional<ImageType::Pointer> get_volume(std::string path)
{
    using ReaderType = itk::ImageSeriesReader<DICOMImageType>;
    auto reader = ReaderType::New();

    using ImageIOType = itk::GDCMImageIO;
    auto dicomIO = ImageIOType::New();

    reader->SetImageIO(dicomIO);

    using NamesGeneratorType = itk::GDCMSeriesFileNames;
    auto nameGenerator = NamesGeneratorType::New();

    nameGenerator->SetUseSeriesDetails(true);
    nameGenerator->AddSeriesRestriction("0008|0021");

    nameGenerator->SetDirectory(path);

    using SeriesIdContainer = std::vector<std::string>;

    const SeriesIdContainer &seriesUID = nameGenerator->GetSeriesUIDs();

    auto seriesItr = seriesUID.begin();
    auto seriesEnd = seriesUID.end();
    while (seriesItr != seriesEnd)
    {
        std::cout << seriesItr->c_str() << std::endl;
        ++seriesItr;
    }

    std::string seriesIdentifier;
    seriesIdentifier = seriesUID.begin()->c_str();

    using FileNamesContainer = std::vector<std::string>;
    FileNamesContainer fileNames;

    fileNames = nameGenerator->GetFileNames(seriesIdentifier);

    reader->SetFileNames(fileNames);

    using RescaleType = itk::RescaleIntensityImageFilter<DICOMImageType, DICOMImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(reader->GetOutput());
    rescale->SetOutputMinimum(0);
    rescale->SetOutputMaximum(itk::NumericTraits<PixelType>::max());

    using FilterType = itk::CastImageFilter<DICOMImageType, ImageType>;
    auto filter = FilterType::New();
    filter->SetInput(rescale->GetOutput());

    try
    {
        filter->Update();
    }
    catch (const itk::ExceptionObject &ex)
    {
        std::cout << ex << std::endl;
        return std::nullopt;
    }

    return filter->GetOutput();
}

int main()
{
    auto volume = get_volume(CURAN_COPIED_RESOURCE_PATH "/dicom_sample/ST983524");
    if (!volume)
        return 1;
    
    Eigen::Matrix<double, 3, 1> orient_along_ac_pc = *pc_point - *ac_point;
    if (orient_along_ac_pc.norm() < 1e-7)
    {
        return 2;
    }
    orient_along_ac_pc.normalize();
    Eigen::Matrix<double, 3, 1> orient_along_ac_midpoint = *midline - *ac_point;
    if (orient_along_ac_midpoint.norm() < 1e-7)
    {
        return 3;
    }
    orient_along_ac_midpoint.normalize();
    Eigen::Matrix<double, 3, 1> orient_perpendic_to_ac_pc_ac_midline = orient_along_ac_pc.cross(orient_along_ac_midpoint);
    if (orient_perpendic_to_ac_pc_ac_midline.norm() < 1e-7)
    {
        return 4;
    }

    orient_perpendic_to_ac_pc_ac_midline.normalize();

    orient_along_ac_midpoint = orient_perpendic_to_ac_pc_ac_midline.cross(orient_along_ac_pc);

    orient_along_ac_midpoint.normalize();

    itk::Matrix<double, 3, 3> rotation_matrix;
    rotation_matrix(0, 0) = orient_along_ac_pc(0, 0);
    rotation_matrix(1, 0) = orient_along_ac_pc(1, 0);
    rotation_matrix(2, 0) = orient_along_ac_pc(2, 0);

    rotation_matrix(0, 1) = orient_perpendic_to_ac_pc_ac_midline[0];
    rotation_matrix(1, 1) = orient_perpendic_to_ac_pc_ac_midline[1];
    rotation_matrix(2, 1) = orient_perpendic_to_ac_pc_ac_midline[2];

    rotation_matrix(0, 2) = orient_along_ac_midpoint[0];
    rotation_matrix(1, 2) = orient_along_ac_midpoint[1];
    rotation_matrix(2, 2) = orient_along_ac_midpoint[2];

    Eigen::Matrix<double, 3, 3> eigen_rotation_matrix;
    for (size_t col = 0; col < 3; ++col)
        for (size_t row = 0; row < 3; ++row)
            eigen_rotation_matrix(row, col) = rotation_matrix(row, col);

    itk::Vector<double, 3> origin;
    origin[0] = (*ac_point)[0];
    origin[1] = (*ac_point)[1];
    origin[2] = (*ac_point)[2];

    using FilterType = itk::ResampleImageFilter<ImageType, ImageType>;
    auto filter = FilterType::New();

    using TransformType = itk::AffineTransform<double, 3>;
    auto transform = TransformType::New();
    transform->SetMatrix(rotation_matrix);

    using InterpolatorType = itk::LinearInterpolateImageFunction<ImageType, double>;
    auto interpolator = InterpolatorType::New();
    filter->SetInterpolator(interpolator);
    filter->SetDefaultPixelValue(0);
    filter->SetTransform(transform);
    auto input = *volume;

    Eigen::Matrix<double, 3, 3> original_rotation_matrix = Eigen::Matrix<double, 3, 3>::Zero();
    auto itk_original_matrix = input->GetDirection();
    for (size_t col = 0; col < 3; ++col)
        for (size_t row = 0; row < 3; ++row)
            original_rotation_matrix(row, col) = itk_original_matrix(row, col);

    auto relative_transformation = original_rotation_matrix.transpose() * eigen_rotation_matrix;

    auto size = input->GetLargestPossibleRegion().GetSize();

    double x = (double)size[0] * input->GetSpacing()[0];
    double y = (double)size[1] * input->GetSpacing()[1];
    double z = (double)size[2] * input->GetSpacing()[2];

    enum transform_index
    {
        index_001 = 0,
        index_010 = 1,
        index_011 = 2,
        index_100 = 3,
        index_101 = 4,
        index_110 = 5,
        index_111 = 6,
        total_number_of_indicies,
    };

    Eigen::Matrix<double, 3, transform_index::total_number_of_indicies> corners = Eigen::Matrix<double, 3, transform_index::total_number_of_indicies>::Zero();
    corners.col(index_001) = Eigen::Matrix<double, 3, 1>{{x, .0, .0}};
    corners.col(index_010) = Eigen::Matrix<double, 3, 1>{{.0, y, .0}};
    corners.col(index_011) = Eigen::Matrix<double, 3, 1>{{x, y, .0}};
    corners.col(index_100) = Eigen::Matrix<double, 3, 1>{{.0, .0, z}};
    corners.col(index_101) = Eigen::Matrix<double, 3, 1>{{x, .0, z}};
    corners.col(index_110) = Eigen::Matrix<double, 3, 1>{{.0, y, z}};
    corners.col(index_111) = Eigen::Matrix<double, 3, 1>{{x, y, z}};

    Eigen::Matrix<double, 3, transform_index::total_number_of_indicies> transformed_corners;

    transformed_corners.col(index_001) = relative_transformation * corners.col(index_001);
    transformed_corners.col(index_010) = relative_transformation * corners.col(index_010);
    transformed_corners.col(index_011) = relative_transformation * corners.col(index_011);
    transformed_corners.col(index_100) = relative_transformation * corners.col(index_100);
    transformed_corners.col(index_101) = relative_transformation * corners.col(index_101);
    transformed_corners.col(index_110) = relative_transformation * corners.col(index_110);
    transformed_corners.col(index_111) = relative_transformation * corners.col(index_111);

    auto minimum = transformed_corners.rowwise().minCoeff();

    transformed_corners.col(index_001) -= minimum;
    transformed_corners.col(index_010) -= minimum;
    transformed_corners.col(index_011) -= minimum;
    transformed_corners.col(index_100) -= minimum;
    transformed_corners.col(index_101) -= minimum;
    transformed_corners.col(index_110) -= minimum;
    transformed_corners.col(index_111) -= minimum;

    auto required_size = transformed_corners.rowwise().maxCoeff();
    Eigen::Matrix<double, 3, 1> required_size_rounded;

    auto spacing = input->GetSpacing();
    auto new_spacing = spacing;
    double minimum_spacing = std::min(std::min(spacing[0], spacing[1]), spacing[2]);

    for (size_t row = 0; row < 3; ++row)
    {
        double rounded = std::ceil(required_size[row] / minimum_spacing);
        required_size_rounded[row] = rounded;
        double spac = rounded / required_size[row];
        new_spacing[row] = spac;
        std::printf("iter (%llu) rounded (%f) spacing (%f)\n", row, rounded, spac);
    }

    filter->SetInput(input);
    filter->SetOutputOrigin(origin);
    filter->SetOutputSpacing(new_spacing);

    try
    {
        filter->Update();
        auto output = filter->GetOutput();
        auto outsize = output->GetLargestPossibleRegion().GetSize();
        auto outspacing = output->GetSpacing();
        map[PanelType::RESAMPLED_VOLUME].update_volume(output);
    }
    catch (...)
    {
        if (config->stack_page != nullptr)
            config->stack_page->stack(create_overlay_with_warning("failed to resample volume to AC-PC"));
    }
}