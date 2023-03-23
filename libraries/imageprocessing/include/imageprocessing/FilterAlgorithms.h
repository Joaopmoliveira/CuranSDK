#ifndef CURAN_SEGMENTATION_HEADER_FILE_
#define CURAN_SEGMENTATION_HEADER_FILE_

#include "ImageProcessingDefinitions.h"
#include "itkBinaryThresholdImageFilter.h"
#include <vector>
#include "utils/Lockable.h"

namespace curan {
    namespace image {
        namespace filtering {

            class Filter {

                virtual bool update() = 0;
                virtual void get_output() = 0;
                virtual void set_input() = 0;
            };

            class CompositFilter {
                std::vector<std::shared_ptr<Filter>> filters;

            };
            
            class ThreholdFilter : public Filter , utils::Lockable<ThreholdFilter> {
            public:

                struct Info {
                    char_pixel_type lower_bound = 0;
                    char_pixel_type upper_bound = 255;
                };

            private:

                using FilterType = itk::ThresholdImageFilter<InternalImageType>;
                FilterType::Pointer filter;
                char_pixel_type lower_bound = 0;
                char_pixel_type upper_bound = 255;

                ThreholdFilter(Info& info);

            public:

                std::shared_ptr<ThreholdFilter> make(Info& info);

                void submit_image(Image input);

                void updateinfo(Info& info);

                bool update() override;
            };

            class CannyFilter : public Filter, utils::Lockable<CannyFilter> {
            public:

                struct Info {

                };

            private:
                using real_pixel_type = double;
                using CastToRealFilterType = itk::CastImageFilter<char_pixel_type, real_pixel_type>;
                using CannyFilterType = itk::CannyEdgeDetectionImageFilter<real_pixel_type, real_pixel_type>;
                using RescaleFilterType = itk::RescaleIntensityImageFilter<real_pixel_type, char_pixel_type>;

                CannyFilter(Info& info);

            public:

                std::shared_ptr<CannyFilter> make(Info& info);

                void submit_image(Image input);

                void updateinfo(Info& info);
               
                bool update() override;
            };

            class BinarizeFilter : public Filter, utils::Lockable<BinarizeFilter> {
            public:

                struct Info {
                    char_pixel_type lower_bound = 0;
                    char_pixel_type upper_bound = 255;
                };

            private:

                char_pixel_type lower_bound = 0;
                char_pixel_type upper_bound = 255;
                using FilterType = itk::BinaryThresholdImageFilter<InternalImageType, InternalImageType>;
                FilterType::Pointer filter;

                BinarizeFilter(Info& info);

            public:

                std::shared_ptr<BinarizeFilter> make(Info& info);

                void submit_image(Image input);

                void updateinfo(Info& info);

                bool update() override;
            };


        }
    }
}

#endif