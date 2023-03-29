#ifndef CURAN_FILTER_ALGORITHMS_HEADER_FILE_
#define CURAN_FILTER_ALGORITHMS_HEADER_FILE_

#include "ImageProcessingDefinitions.h"
#include "itkBinaryThresholdImageFilter.h"
#include "itkImportImageFilter.h"
#include "itkHoughTransform2DCirclesImageFilter.h"
#include "itkImageRegionIterator.h"
#include "itkThresholdImageFilter.h"
#include "itkMinimumMaximumImageCalculator.h"
#include "itkGradientMagnitudeImageFilter.h"
#include "itkDiscreteGaussianImageFilter.h"
#include <vector>
#include "utils/Lockable.h"
#include <memory>

namespace curan {
    namespace image {
        namespace filtering {

            class Implementation {
            public:
                virtual Internal2DImageType* get_output() = 0;
                virtual void set_input(const Internal2DImageType*) = 0;
                virtual Internal2DImageType::Pointer update_and_return_out() = 0;
            };

            class Filter : utils::Lockable<Filter> {

                bool is_updated = true;
                Internal2DImageType::Pointer input;
                std::list<std::shared_ptr<Implementation>> filters;

            public:

                void operator<< (std::shared_ptr<Implementation>);
                Filter();
                bool set_input(Internal2DImageType::Pointer val);
                Internal2DImageType::Pointer get_output();
            };

            class ImportFilter : public Implementation, utils::Lockable<ImportFilter> {
            public:

                using ImportFilterType = itk::ImportImageFilter<char_pixel_type, 2>;

                struct Info {
                    std::array<double, 2> origin;
                    std::array<double, 2> spacing;
                    unsigned char* buffer = nullptr;
                    bool memory_owner = false;
                    long number_of_pixels = 0;
                    ImportFilterType::SizeType size;
                    ImportFilterType::IndexType start;
                };

            private:
                
                ImportFilterType::Pointer filter;

                ImportFilter(Info& info);

            public:
                static std::shared_ptr<ImportFilter> make(Info& info);

                void updateinfo(Info& info);

                Internal2DImageType* get_output() override;

                void set_input(const Internal2DImageType*) override;
                Internal2DImageType::Pointer update_and_return_out() override;
            };

            class CircleFilter : public Implementation, utils::Lockable<CircleFilter> {
            public:

                struct Info {

                };
            private:
                Filter* owner = nullptr;
                using HoughTransformFilterType = itk::HoughTransform2DCirclesImageFilter<unsigned char,unsigned int, double>;

                CircleFilter(Info& info);

                friend Filter;
            public:

                static std::shared_ptr<CircleFilter> make(Info& info);

                void updateinfo(Info& info);

                Internal2DImageType* get_output() override;
                void set_input(const Internal2DImageType*) override;
                Internal2DImageType::Pointer update_and_return_out() override;
            };
            
            class ThreholdFilter : public Implementation, utils::Lockable<ThreholdFilter> {
            public:

                struct Info {
                    char_pixel_type lower_bound = 0;
                    char_pixel_type upper_bound = 255;
                    char_pixel_type outside_value = 0;
                };

            private:

                Filter* owner = nullptr;

                using FilterType = itk::ThresholdImageFilter<Internal2DImageType>;
                FilterType::Pointer filter;
                char_pixel_type lower_bound = 0;
                char_pixel_type upper_bound = 255;

                ThreholdFilter(Info& info);

                friend Filter;

            public:

                static std::shared_ptr<ThreholdFilter> make(Info& info);

                void updateinfo(Info& info);

                Internal2DImageType* get_output() override;
                void set_input(const Internal2DImageType*) override;
                Internal2DImageType::Pointer update_and_return_out() override;
            };

            class CannyFilter : public Implementation, utils::Lockable<CannyFilter> {
            public:

                using real_pixel_type = double;

                struct Info {
                    real_pixel_type variance;
                    real_pixel_type lower_bound;
                    real_pixel_type upper_bound;
                };

            private:

                Filter* owner = nullptr;
                using RealImageType = itk::Image<real_pixel_type, 2>;
                using CastToRealFilterType = itk::CastImageFilter<Internal2DImageType, RealImageType>;
                using CannyFilterType = itk::CannyEdgeDetectionImageFilter<RealImageType, RealImageType>;
                using RescaleFilterType = itk::RescaleIntensityImageFilter<RealImageType, Internal2DImageType>;

                CastToRealFilterType::Pointer cast_to_real;
                CannyFilterType::Pointer canny_filter;
                RescaleFilterType::Pointer cast_to_char;

                CannyFilter(Info& info);

                friend Filter;

            public:

                static std::shared_ptr<CannyFilter> make(Info& info);

                void updateinfo(Info& info);
               
                Internal2DImageType* get_output() override;
                void set_input(const Internal2DImageType*) override;
                Internal2DImageType::Pointer update_and_return_out() override;
            };

            class BinarizeFilter : public Implementation, utils::Lockable<BinarizeFilter> {
            public:

                struct Info {
                    char_pixel_type outside_value = 0;
                    char_pixel_type inside_value = 255;
                    char_pixel_type lower_value = 0;
                    char_pixel_type higher_value = 255;
                };

            private:

                Filter* owner = nullptr;

                char_pixel_type lower_bound = 0;
                char_pixel_type upper_bound = 255;
                using FilterType = itk::BinaryThresholdImageFilter<Internal2DImageType, Internal2DImageType>;
                FilterType::Pointer filter;

                BinarizeFilter(Info& info);

                friend Filter;

            public:

                static std::shared_ptr<BinarizeFilter> make(Info& info);

                void updateinfo(Info& info);

                Internal2DImageType* get_output() override;
                void set_input(const Internal2DImageType*) override;
                Internal2DImageType::Pointer update_and_return_out() override;
            };


        }
    }
}

#endif